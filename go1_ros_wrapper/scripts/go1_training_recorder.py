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
            self.declare_parameter("flush_every_n_rows", 50).value
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
        self.closed = False
        self.start_position: dict[str, Any] | None = None
        self.final_position: dict[str, Any] | None = None
        self.start_odometry: dict[str, Any] | None = None
        self.final_odometry: dict[str, Any] | None = None

        self.write_metadata()
        self.discover_topics()
        self.discovery_timer = self.create_timer(
            self.discovery_period_s,
            self.discover_topics,
        )

        self.get_logger().info(
            f"Recording {self.run_type!r} training run to {self.run_dir}"
        )

    def write_metadata(self) -> None:
        metadata = {
            "run_id": self.run_id,
            "run_type": self.run_type,
            "started_at_iso": self.started_at_iso,
            "started_at_ros_time_ns": self.started_at_ns,
            "include_topic_prefixes": self.include_prefixes,
            "include_tf": self.include_tf,
            "max_inline_sequence": self.max_inline_sequence,
        }
        with (self.run_dir / "metadata.json").open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2)

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
            "start_position_json": json.dumps(self.start_position, sort_keys=True),
            "final_position_json": json.dumps(self.final_position, sort_keys=True),
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
