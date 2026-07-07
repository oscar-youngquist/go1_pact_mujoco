#!/usr/bin/python3
"""Record Go1 training runs from ROS 2 topics into per-run CSV files."""

from __future__ import annotations

import csv
import atexit
import hashlib
import json
import math
import os
import re
import signal
from array import array
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix


def now_stamp() -> str:
    return datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d_%H-%M-%S")


def iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat()


def topic_slug(topic: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "_", topic.strip("/"))
    return slug.strip("_") or "root"


def primitive(value: Any) -> bool:
    return value is None or isinstance(value, (bool, int, float, str))


def json_default(value: Any) -> Any:
    if isinstance(value, array):
        return list(value)
    if isinstance(value, bytes):
        return list(value)
    if hasattr(value, "tolist"):
        return value.tolist()
    return str(value)


def stamp_to_ns(stamp: Any) -> int | None:
    if stamp is None:
        return None
    if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return None


class TopicCsvWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer: csv.DictWriter[str] | None = None
        self.fieldnames: list[str] = []

    def write(self, row: dict[str, Any]) -> None:
        clean = {key: "" if value is None else value for key, value in row.items()}
        if self.writer is None:
            self.fieldnames = list(clean.keys())
            self.writer = csv.DictWriter(
                self.file,
                fieldnames=self.fieldnames,
                extrasaction="ignore",
            )
            self.writer.writeheader()
        else:
            for key in clean.keys():
                if key not in self.fieldnames:
                    clean.setdefault("_extra_fields_json", "{}")
                    extras = {
                        extra_key: clean[extra_key]
                        for extra_key in clean.keys()
                        if extra_key not in self.fieldnames
                    }
                    clean["_extra_fields_json"] = json.dumps(
                        extras,
                        default=json_default,
                        separators=(",", ":"),
                    )
                    break
        self.writer.writerow(clean)

    def close(self) -> None:
        self.file.flush()
        self.file.close()


class AppendBinaryWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.file = path.open("wb")
        self.offset = 0

    def write(self, value: Any, hash_payload: bool = False) -> tuple[int, int, str]:
        if isinstance(value, array):
            view = memoryview(value)
        elif isinstance(value, memoryview):
            view = value
        elif isinstance(value, (bytes, bytearray)):
            view = memoryview(value)
        else:
            view = memoryview(bytes(value))

        offset = self.offset
        size = view.nbytes
        digest = hashlib.sha256(view).hexdigest() if hash_payload else ""
        self.file.write(view)
        self.offset += size
        return offset, size, digest

    def close(self) -> None:
        self.file.flush()
        self.file.close()


class Go1TrainingRecorder(Node):
    def __init__(self) -> None:
        super().__init__("go1_training_recorder")

        self.run_type = self.declare_parameter("run_type", "").value
        if self.run_type not in {"safe", "fast"}:
            raise RuntimeError("Parameter 'run_type' must be either 'safe' or 'fast'.")

        output_root = Path(
            os.path.expanduser(
                str(self.declare_parameter("output_root", "~/go1_training_runs").value)
            )
        )
        self.include_prefixes = list(
            self.declare_parameter("include_topic_prefixes", ["/go1"]).value
        )
        self.include_tf = bool(self.declare_parameter("include_tf", True).value)
        self.discovery_period_s = float(
            self.declare_parameter("discovery_period_s", 1.0).value
        )
        self.max_inline_sequence = int(
            self.declare_parameter("max_inline_sequence", 128).value
        )
        self.flush_every_n_rows = int(
            self.declare_parameter("flush_every_n_rows", 100).value
        )
        self.record_mode = str(
            self.declare_parameter("record_mode", "selected").value
        ).strip().lower()
        self.gps_topic = str(
            self.declare_parameter("gps_topic", "/go1/sensors/gps/fix").value
        )
        self.orientation_topic = str(
            self.declare_parameter(
                "orientation_topic",
                "/go1/sensors/ellipse/imu/data",
            ).value
        )
        self.rgb_topic = str(
            self.declare_parameter(
                "rgb_topic",
                "/go1/sensors/camera/color/image_raw",
            ).value
        )
        self.depth_topic = str(
            self.declare_parameter(
                "depth_topic",
                "/go1/sensors/camera/depth/image_raw",
            ).value
        )
        self.rgb_info_topic = str(
            self.declare_parameter(
                "rgb_info_topic",
                "/go1/sensors/camera/color/camera_info",
            ).value
        )
        self.depth_info_topic = str(
            self.declare_parameter(
                "depth_info_topic",
                "/go1/sensors/camera/depth/camera_info",
            ).value
        )
        self.record_camera_info = bool(
            self.declare_parameter("record_camera_info", True).value
        )
        self.record_camera_info_once = bool(
            self.declare_parameter("record_camera_info_once", True).value
        )
        self.hash_image_payloads = bool(
            self.declare_parameter("hash_image_payloads", False).value
        )

        self.run_id = f"{now_stamp()}_{self.run_type}"
        self.run_dir = output_root / self.run_id
        self.topic_dir = self.run_dir / "topics"
        self.payload_dir = self.run_dir / "payloads"
        self.topic_dir.mkdir(parents=True, exist_ok=False)
        self.payload_dir.mkdir(parents=True, exist_ok=True)

        self.started_at_iso = iso_now()
        self.started_at_ns = self.get_clock().now().nanoseconds
        self.subscriptions_by_topic: dict[str, Any] = {}
        self.topic_types: dict[str, str] = {}
        self.writers: dict[str, TopicCsvWriter] = {}
        self.row_counts: dict[str, int] = {}
        self.payload_counts: dict[str, int] = {}
        self.binary_writers: dict[str, AppendBinaryWriter] = {}
        self.camera_info_seen: set[str] = set()
        self.closed = False
        self.start_position: dict[str, Any] | None = None
        self.final_position: dict[str, Any] | None = None
        self.start_orientation: dict[str, Any] | None = None
        self.final_orientation: dict[str, Any] | None = None
        self.start_odometry: dict[str, Any] | None = None
        self.final_odometry: dict[str, Any] | None = None

        self.write_metadata()
        self.discovery_timer = None
        if self.record_mode in {"selected", "sensors", "optimized"}:
            self.setup_selected_subscriptions()
        else:
            self.discover_topics()
            self.discovery_timer = self.create_timer(
                self.discovery_period_s,
                self.discover_topics,
            )

        self.get_logger().info(
            f"Recording {self.run_type!r} training run to {self.run_dir} "
            f"with record_mode={self.record_mode!r}"
        )

    def write_metadata(self) -> None:
        metadata = {
            "run_id": self.run_id,
            "run_type": self.run_type,
            "started_at_iso": self.started_at_iso,
            "started_at_ros_time_ns": self.started_at_ns,
            "record_mode": self.record_mode,
            "include_topic_prefixes": self.include_prefixes,
            "include_tf": self.include_tf,
            "max_inline_sequence": self.max_inline_sequence,
            "selected_topics": {
                "gps": self.gps_topic,
                "orientation": self.orientation_topic,
                "rgb": self.rgb_topic,
                "depth": self.depth_topic,
                "rgb_info": self.rgb_info_topic,
                "depth_info": self.depth_info_topic,
            },
            "record_camera_info": self.record_camera_info,
            "record_camera_info_once": self.record_camera_info_once,
            "image_payload_layout": "append_only_binary_with_csv_index",
            "hash_image_payloads": self.hash_image_payloads,
        }
        with (self.run_dir / "metadata.json").open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2)

    def setup_selected_subscriptions(self) -> None:
        selected_topics: list[tuple[str, Any, Any, str]] = [
            (self.gps_topic, NavSatFix, self.record_gps, "sensor_msgs/msg/NavSatFix"),
            (self.orientation_topic, Imu, self.record_orientation, "sensor_msgs/msg/Imu"),
            (self.rgb_topic, Image, lambda msg: self.record_image(self.rgb_topic, msg), "sensor_msgs/msg/Image"),
            (self.depth_topic, Image, lambda msg: self.record_image(self.depth_topic, msg), "sensor_msgs/msg/Image"),
        ]
        if self.record_camera_info:
            selected_topics.extend([
                (self.rgb_info_topic, CameraInfo, lambda msg: self.record_camera_info_msg(self.rgb_info_topic, msg), "sensor_msgs/msg/CameraInfo"),
                (self.depth_info_topic, CameraInfo, lambda msg: self.record_camera_info_msg(self.depth_info_topic, msg), "sensor_msgs/msg/CameraInfo"),
            ])

        for topic, message_type, callback, type_name in selected_topics:
            if not topic:
                continue
            if topic in self.subscriptions_by_topic:
                continue
            subscription = self.create_subscription(
                message_type,
                topic,
                callback,
                self.qos_for_topic(topic),
            )
            self.subscriptions_by_topic[topic] = subscription
            self.topic_types[topic] = type_name
            self.get_logger().info(f"Recording selected topic {topic} [{type_name}]")

    def increment_row_count(self, topic: str) -> None:
        count = self.row_counts.get(topic, 0) + 1
        self.row_counts[topic] = count
        if self.flush_every_n_rows > 0 and count % self.flush_every_n_rows == 0:
            writer = self.writers.get(topic)
            if writer is not None:
                writer.file.flush()
            binary_writer = self.binary_writers.get(topic)
            if binary_writer is not None:
                binary_writer.file.flush()

    def binary_writer_for_topic(self, topic: str) -> AppendBinaryWriter:
        if topic not in self.binary_writers:
            self.binary_writers[topic] = AppendBinaryWriter(
                self.payload_dir / f"{topic_slug(topic)}.bin"
            )
        return self.binary_writers[topic]

    def record_gps(self, msg: NavSatFix) -> None:
        topic = self.gps_topic
        recv_ns = self.get_clock().now().nanoseconds
        status = getattr(getattr(msg, "status", None), "status", None)
        row = {
            "_recv_time_ns": recv_ns,
            "_msg_time_ns": self.message_time_ns(msg),
            "_topic": topic,
            "frame_id": msg.header.frame_id,
            "status": status,
            "service": msg.status.service,
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "position_covariance_type": msg.position_covariance_type,
        }
        for index, value in enumerate(msg.position_covariance):
            row[f"position_covariance.{index:02d}"] = value

        fix = {
            "topic": topic,
            "recv_time_ns": recv_ns,
            "msg_time_ns": self.message_time_ns(msg),
            "status": status,
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
        }
        if self.start_position is None and status is not None and status >= 0:
            self.start_position = fix
        if status is not None and status >= 0:
            self.final_position = fix

        self.writer_for_topic(topic).write(row)
        self.increment_row_count(topic)

    def record_orientation(self, msg: Imu) -> None:
        topic = self.orientation_topic
        recv_ns = self.get_clock().now().nanoseconds
        row = {
            "_recv_time_ns": recv_ns,
            "_msg_time_ns": self.message_time_ns(msg),
            "_topic": topic,
            "frame_id": msg.header.frame_id,
            "orientation_x": msg.orientation.x,
            "orientation_y": msg.orientation.y,
            "orientation_z": msg.orientation.z,
            "orientation_w": msg.orientation.w,
            "angular_velocity_x": msg.angular_velocity.x,
            "angular_velocity_y": msg.angular_velocity.y,
            "angular_velocity_z": msg.angular_velocity.z,
            "linear_acceleration_x": msg.linear_acceleration.x,
            "linear_acceleration_y": msg.linear_acceleration.y,
            "linear_acceleration_z": msg.linear_acceleration.z,
        }
        for index, value in enumerate(msg.orientation_covariance):
            row[f"orientation_covariance.{index:02d}"] = value

        orientation = {
            "topic": topic,
            "recv_time_ns": recv_ns,
            "msg_time_ns": self.message_time_ns(msg),
            "orientation_x": msg.orientation.x,
            "orientation_y": msg.orientation.y,
            "orientation_z": msg.orientation.z,
            "orientation_w": msg.orientation.w,
        }
        if self.start_orientation is None:
            self.start_orientation = orientation
        self.final_orientation = orientation

        self.writer_for_topic(topic).write(row)
        self.increment_row_count(topic)

    def record_camera_info_msg(self, topic: str, msg: CameraInfo) -> None:
        if self.record_camera_info_once and topic in self.camera_info_seen:
            return
        self.camera_info_seen.add(topic)

        row = {
            "_recv_time_ns": self.get_clock().now().nanoseconds,
            "_msg_time_ns": self.message_time_ns(msg),
            "_topic": topic,
            "frame_id": msg.header.frame_id,
            "height": msg.height,
            "width": msg.width,
            "distortion_model": msg.distortion_model,
            "d": json.dumps(list(msg.d), separators=(",", ":")),
            "k": json.dumps(list(msg.k), separators=(",", ":")),
            "r": json.dumps(list(msg.r), separators=(",", ":")),
            "p": json.dumps(list(msg.p), separators=(",", ":")),
            "binning_x": msg.binning_x,
            "binning_y": msg.binning_y,
            "roi_x_offset": msg.roi.x_offset,
            "roi_y_offset": msg.roi.y_offset,
            "roi_height": msg.roi.height,
            "roi_width": msg.roi.width,
            "roi_do_rectify": msg.roi.do_rectify,
        }
        self.writer_for_topic(topic).write(row)
        self.increment_row_count(topic)

    def record_image(self, topic: str, msg: Image) -> None:
        recv_ns = self.get_clock().now().nanoseconds
        payload_writer = self.binary_writer_for_topic(topic)
        offset, size, digest = payload_writer.write(msg.data, self.hash_image_payloads)
        payload_file = str(payload_writer.path.relative_to(self.run_dir))

        row = {
            "_recv_time_ns": recv_ns,
            "_msg_time_ns": self.message_time_ns(msg),
            "_topic": topic,
            "frame_id": msg.header.frame_id,
            "height": msg.height,
            "width": msg.width,
            "encoding": msg.encoding,
            "is_bigendian": msg.is_bigendian,
            "step": msg.step,
            "payload_file": payload_file,
            "payload_offset": offset,
            "payload_size": size,
        }
        if digest:
            row["payload_sha256"] = digest

        self.writer_for_topic(topic).write(row)
        self.increment_row_count(topic)

    def wanted_topic(self, topic: str) -> bool:
        if topic in {"/parameter_events", "/rosout"}:
            return False
        if self.include_tf and topic in {"/tf", "/tf_static"}:
            return True
        return any(
            topic == prefix or topic.startswith(prefix.rstrip("/") + "/")
            for prefix in self.include_prefixes
        )

    def qos_for_topic(self, topic: str) -> QoSProfile:
        if topic == "/tf_static":
            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

    def discover_topics(self) -> None:
        for topic, type_names in self.get_topic_names_and_types():
            if topic in self.subscriptions_by_topic or not self.wanted_topic(topic):
                continue
            if not type_names:
                continue
            type_name = sorted(type_names)[0]
            try:
                message_type = get_message(type_name)
            except Exception as exc:
                self.get_logger().warn(
                    f"Skipping {topic} [{type_name}]: cannot load message type: {exc}"
                )
                continue

            subscription = self.create_subscription(
                message_type,
                topic,
                lambda msg, topic=topic, type_name=type_name: self.record_message(
                    topic,
                    type_name,
                    msg,
                ),
                self.qos_for_topic(topic),
            )
            self.subscriptions_by_topic[topic] = subscription
            self.topic_types[topic] = type_name
            self.get_logger().info(f"Recording topic {topic} [{type_name}]")

    def writer_for_topic(self, topic: str) -> TopicCsvWriter:
        if topic not in self.writers:
            self.writers[topic] = TopicCsvWriter(self.topic_dir / f"{topic_slug(topic)}.csv")
        return self.writers[topic]

    def next_payload_path(self, topic: str, suffix: str) -> Path:
        slug = topic_slug(topic)
        count = self.payload_counts.get(topic, 0)
        self.payload_counts[topic] = count + 1
        directory = self.payload_dir / slug
        directory.mkdir(parents=True, exist_ok=True)
        return directory / f"{count:09d}{suffix}"

    def save_payload(self, topic: str, prefix: str, value: Any) -> dict[str, Any]:
        if isinstance(value, array):
            data = value.tobytes()
        elif isinstance(value, (bytes, bytearray, memoryview)):
            data = bytes(value)
        else:
            data = json.dumps(value, default=json_default).encode("utf-8")

        path = self.next_payload_path(topic, ".bin")
        path.write_bytes(data)
        digest = hashlib.sha256(data).hexdigest()
        return {
            f"{prefix}__payload_file": str(path.relative_to(self.run_dir)),
            f"{prefix}__payload_size": len(data),
            f"{prefix}__payload_sha256": digest,
        }

    def flatten_value(
        self,
        topic: str,
        prefix: str,
        value: Any,
        output: dict[str, Any],
    ) -> None:
        if primitive(value):
            if isinstance(value, float) and not math.isfinite(value):
                output[prefix] = str(value)
            else:
                output[prefix] = value
            return

        if hasattr(value, "get_fields_and_field_types"):
            for field_name in value.get_fields_and_field_types().keys():
                nested_prefix = f"{prefix}.{field_name}" if prefix else field_name
                self.flatten_value(topic, nested_prefix, getattr(value, field_name), output)
            return

        if isinstance(value, (bytes, bytearray, memoryview, array, list, tuple)):
            length = len(value)
            output[f"{prefix}__len"] = length

            is_large_payload = (
                length > self.max_inline_sequence
                or prefix.endswith(".data")
                or prefix == "data"
            )
            if is_large_payload and not (
                isinstance(value, (list, tuple))
                and all(primitive(item) for item in value)
                and length <= self.max_inline_sequence
            ):
                output.update(self.save_payload(topic, prefix, value))
                return

            if all(primitive(item) for item in value) and length <= self.max_inline_sequence:
                for index, item in enumerate(value):
                    output[f"{prefix}.{index:03d}"] = item
                return

            output[prefix] = json.dumps(value, default=json_default, separators=(",", ":"))
            return

        output[prefix] = json.dumps(value, default=json_default, separators=(",", ":"))

    def message_time_ns(self, msg: Any) -> int | None:
        header = getattr(msg, "header", None)
        return stamp_to_ns(getattr(header, "stamp", None))

    def update_positions(self, topic: str, type_name: str, msg: Any, recv_ns: int) -> None:
        if type_name == "sensor_msgs/msg/NavSatFix":
            status = getattr(getattr(msg, "status", None), "status", None)
            fix = {
                "topic": topic,
                "recv_time_ns": recv_ns,
                "msg_time_ns": self.message_time_ns(msg),
                "status": status,
                "latitude": getattr(msg, "latitude", None),
                "longitude": getattr(msg, "longitude", None),
                "altitude": getattr(msg, "altitude", None),
            }
            if self.start_position is None and status is not None and status >= 0:
                self.start_position = fix
            if status is not None and status >= 0:
                self.final_position = fix

        if type_name == "nav_msgs/msg/Odometry":
            pose = getattr(getattr(msg, "pose", None), "pose", None)
            position = getattr(pose, "position", None)
            orientation = getattr(pose, "orientation", None)
            odom = {
                "topic": topic,
                "recv_time_ns": recv_ns,
                "msg_time_ns": self.message_time_ns(msg),
                "position_x": getattr(position, "x", None),
                "position_y": getattr(position, "y", None),
                "position_z": getattr(position, "z", None),
                "orientation_x": getattr(orientation, "x", None),
                "orientation_y": getattr(orientation, "y", None),
                "orientation_z": getattr(orientation, "z", None),
                "orientation_w": getattr(orientation, "w", None),
            }
            if self.start_odometry is None:
                self.start_odometry = odom
            self.final_odometry = odom

    def record_message(self, topic: str, type_name: str, msg: Any) -> None:
        recv_ns = self.get_clock().now().nanoseconds
        self.update_positions(topic, type_name, msg, recv_ns)

        row: dict[str, Any] = {
            "_recv_time_ns": recv_ns,
            "_msg_time_ns": self.message_time_ns(msg),
            "_topic": topic,
            "_type": type_name,
            "_extra_fields_json": "{}",
        }
        self.flatten_value(topic, "", msg, row)

        writer = self.writer_for_topic(topic)
        writer.write(row)
        count = self.row_counts.get(topic, 0) + 1
        self.row_counts[topic] = count
        if self.flush_every_n_rows > 0 and count % self.flush_every_n_rows == 0:
            writer.file.flush()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True

        for writer in self.writers.values():
            writer.close()
        for binary_writer in self.binary_writers.values():
            binary_writer.close()

        ended_at_ns = self.get_clock().now().nanoseconds
        summary = {
            "run_id": self.run_id,
            "run_type": self.run_type,
            "started_at_iso": self.started_at_iso,
            "ended_at_iso": iso_now(),
            "started_at_ros_time_ns": self.started_at_ns,
            "ended_at_ros_time_ns": ended_at_ns,
            "run_dir": str(self.run_dir),
            "topic_count": len(self.writers),
            "row_counts_json": json.dumps(self.row_counts, sort_keys=True),
            "binary_payloads_json": json.dumps(
                {
                    topic: str(writer.path.relative_to(self.run_dir))
                    for topic, writer in self.binary_writers.items()
                },
                sort_keys=True,
            ),
            "start_position_json": json.dumps(self.start_position, sort_keys=True),
            "final_position_json": json.dumps(self.final_position, sort_keys=True),
            "start_orientation_json": json.dumps(self.start_orientation, sort_keys=True),
            "final_orientation_json": json.dumps(self.final_orientation, sort_keys=True),
            "start_odometry_json": json.dumps(self.start_odometry, sort_keys=True),
            "final_odometry_json": json.dumps(self.final_odometry, sort_keys=True),
        }
        with (self.run_dir / "run_summary.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summary.keys()))
            writer.writeheader()
            writer.writerow(summary)

        self.get_logger().info(f"Finished recording run to {self.run_dir}")


def main() -> None:
    rclpy.init()
    node: Go1TrainingRecorder | None = None
    shutdown_requested = False

    def handle_signal(_signum: int, _frame: Any) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        node = Go1TrainingRecorder()
        atexit.register(node.close)
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
