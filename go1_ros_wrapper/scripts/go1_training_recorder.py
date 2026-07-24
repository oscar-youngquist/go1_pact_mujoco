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
import socket
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
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix

REMOTE_COMMAND_MAGIC = "RECORD1"


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

        self.requested_run_type = str(
            self.declare_parameter("run_type", "remote").value
        ).strip().lower()
        if self.requested_run_type not in {"safe", "fast", "remote"}:
            raise RuntimeError(
                "Parameter 'run_type' must be 'safe', 'fast', or 'remote'."
            )

        self.output_root = Path(
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
        self.robot_imu_topic = str(
            self.declare_parameter("robot_imu_topic", "/go1/imu").value
        )
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
        self.velocity_command_topic = str(
            self.declare_parameter(
                "velocity_command_topic",
                "/go1/velocity_command",
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
        self.online_master_latest_enabled = bool(
            self.declare_parameter("online_master_latest", True).value
        )
        self.online_master_ellipse_max_dt_ms = float(
            self.declare_parameter("online_master_ellipse_max_dt_ms", 20.0).value
        )
        self.online_master_gps_max_dt_ms = float(
            self.declare_parameter("online_master_gps_max_dt_ms", 1000.0).value
        )
        self.online_master_rgb_max_dt_ms = float(
            self.declare_parameter("online_master_rgb_max_dt_ms", 100.0).value
        )
        self.online_master_depth_max_dt_ms = float(
            self.declare_parameter("online_master_depth_max_dt_ms", 100.0).value
        )
        self.online_master_velocity_command_max_dt_ms = float(
            self.declare_parameter("online_master_velocity_command_max_dt_ms", 50.0).value
        )
        self.remote_control_port = int(
            self.declare_parameter("remote_control_port", 26761).value
        )
        self.remote_control_enabled = (
            bool(self.declare_parameter("remote_control_enabled", False).value)
            or self.requested_run_type == "remote"
        )

        self.subscriptions_by_topic: dict[str, Any] = {}
        self.topic_types: dict[str, str] = {}
        self.node_closed = False
        self.run_active = False
        self.remote_control_socket: socket.socket | None = None
        self.remote_control_timer: Any | None = None
        self.initialize_inactive_run_state()

        if self.remote_control_enabled:
            self.remote_control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.remote_control_socket.setblocking(False)
            self.remote_control_socket.bind(("127.0.0.1", self.remote_control_port))
            self.remote_control_timer = self.create_timer(0.02, self.poll_remote_control)
            self.get_logger().info(
                "Go2 remote recording control armed: SELECT+X=safe, "
                "SELECT+Y=fast, SELECT+B=stop/save."
            )

        if self.requested_run_type in {"safe", "fast"}:
            self.start_run(self.requested_run_type, "launch parameter")

        self.discovery_timer = None
        if self.record_mode in {"selected", "sensors", "optimized"}:
            self.setup_selected_subscriptions()
        else:
            self.discover_topics()
            self.discovery_timer = self.create_timer(
                self.discovery_period_s,
                self.discover_topics,
            )

        if self.run_active:
            self.get_logger().info(
                f"Recording {self.run_type!r} training run to {self.run_dir} "
                f"with record_mode={self.record_mode!r}"
            )
        else:
            self.get_logger().info(
                "Recorder is idle and waiting for a Go2 remote command."
            )

    def initialize_inactive_run_state(self) -> None:
        self.run_type = ""
        self.run_id = ""
        self.run_dir: Path | None = None
        self.topic_dir: Path | None = None
        self.payload_dir: Path | None = None
        self.started_at_iso = ""
        self.started_at_ns = 0
        self.writers: dict[str, TopicCsvWriter] = {}
        self.row_counts: dict[str, int] = {}
        self.payload_counts: dict[str, int] = {}
        self.binary_writers: dict[str, AppendBinaryWriter] = {}
        self.camera_info_seen: set[str] = set()
        self.online_master_path: Path | None = None
        self.latest_online_rows: dict[str, dict[str, Any]] = {}
        self.online_master_file: Any | None = None
        self.online_master_writer: csv.DictWriter[str] | None = None
        self.online_master_rows = 0
        self.online_master_match_counts = {
            "velocity_command": 0,
            "ellipse": 0,
            "gps": 0,
            "rgb": 0,
            "depth": 0,
        }
        self.online_master_valid_counts = {
            "velocity_command": 0,
            "ellipse": 0,
            "gps": 0,
            "rgb": 0,
            "depth": 0,
        }
        self.start_position: dict[str, Any] | None = None
        self.final_position: dict[str, Any] | None = None
        self.start_orientation: dict[str, Any] | None = None
        self.final_orientation: dict[str, Any] | None = None
        self.start_odometry: dict[str, Any] | None = None
        self.final_odometry: dict[str, Any] | None = None

    def unique_run_dir(self, run_type: str) -> tuple[str, Path]:
        base_run_id = f"{now_stamp()}_{run_type}"
        run_id = base_run_id
        run_dir = self.output_root / run_id
        suffix = 2
        while run_dir.exists():
            run_id = f"{base_run_id}_{suffix:02d}"
            run_dir = self.output_root / run_id
            suffix += 1
        return run_id, run_dir

    def start_run(self, run_type: str, reason: str) -> bool:
        if self.run_active:
            self.get_logger().warn(
                f"Ignoring {run_type!r} start request from {reason}; "
                f"already recording {self.run_type!r} to {self.run_dir}."
            )
            return False

        self.initialize_inactive_run_state()
        self.run_type = run_type
        self.run_id, self.run_dir = self.unique_run_dir(run_type)
        self.topic_dir = self.run_dir / "topics"
        self.payload_dir = self.run_dir / "payloads"
        self.topic_dir.mkdir(parents=True, exist_ok=False)
        self.payload_dir.mkdir(parents=True, exist_ok=True)

        self.started_at_iso = iso_now()
        self.started_at_ns = self.get_clock().now().nanoseconds
        self.online_master_path = self.run_dir / "master_latest_online.csv"
        self.run_active = True

        if self.online_master_latest_enabled:
            self.open_online_master_writer()
        self.write_metadata()

        self.get_logger().info(
            f"Started {run_type!r} recording from {reason}: {self.run_dir}"
        )
        return True

    def poll_remote_control(self) -> None:
        if self.remote_control_socket is None:
            return

        while True:
            try:
                payload, _address = self.remote_control_socket.recvfrom(128)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error(f"Go2 remote-control socket failed: {error}")
                return

            fields = payload.decode("ascii", errors="ignore").strip().split()
            if len(fields) != 2 or fields[0] != REMOTE_COMMAND_MAGIC:
                continue

            command = fields[1].lower()
            if command == "safe":
                self.start_run("safe", "Go2 remote SELECT+X")
            elif command == "fast":
                self.start_run("fast", "Go2 remote SELECT+Y")
            elif command == "stop":
                self.stop_run("Go2 remote SELECT+B")

    def write_metadata(self) -> None:
        if self.run_dir is None:
            return
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
                "robot_imu": self.robot_imu_topic,
                "gps": self.gps_topic,
                "orientation": self.orientation_topic,
                "rgb": self.rgb_topic,
                "depth": self.depth_topic,
                "rgb_info": self.rgb_info_topic,
                "depth_info": self.depth_info_topic,
                "velocity_command": self.velocity_command_topic,
            },
            "record_camera_info": self.record_camera_info,
            "record_camera_info_once": self.record_camera_info_once,
            "image_payload_layout": "append_only_binary_with_csv_index",
            "hash_image_payloads": self.hash_image_payloads,
            "online_master_latest": {
                "enabled": self.online_master_latest_enabled,
                "output": "master_latest_online.csv",
                "policy": "latest_received_on_each_robot_imu",
                "time_source": "_msg_time_ns_with_recv_fallback",
                "tolerances_ms": {
                    "ellipse": self.online_master_ellipse_max_dt_ms,
                    "gps": self.online_master_gps_max_dt_ms,
                    "rgb": self.online_master_rgb_max_dt_ms,
                    "depth": self.online_master_depth_max_dt_ms,
                    "velocity_command": self.online_master_velocity_command_max_dt_ms,
                },
            },
        }
        with (self.run_dir / "metadata.json").open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2)

    def setup_selected_subscriptions(self) -> None:
        selected_topics: list[tuple[str, Any, Any, str]] = [
            (self.robot_imu_topic, Imu, self.record_robot_imu, "sensor_msgs/msg/Imu"),
            (self.gps_topic, NavSatFix, self.record_gps, "sensor_msgs/msg/NavSatFix"),
            (self.orientation_topic, Imu, self.record_orientation, "sensor_msgs/msg/Imu"),
            (self.rgb_topic, Image, lambda msg: self.record_image(self.rgb_topic, msg), "sensor_msgs/msg/Image"),
            (self.depth_topic, Image, lambda msg: self.record_image(self.depth_topic, msg), "sensor_msgs/msg/Image"),
            (self.velocity_command_topic, TwistStamped, self.record_velocity_command, "geometry_msgs/msg/TwistStamped"),
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

    def online_master_fieldnames(self) -> list[str]:
        fields = [
            "master_index",
            "master_time_ns",
            "master_recv_time_ns",
            "master_write_time_ns",
            "robot_row",
        ]
        fields += self.online_imu_fieldnames("robot")
        fields += [
            "velocity_command_row",
            "velocity_command_time_ns",
            "velocity_command_dt_ms",
            "velocity_command_valid",
            "velocity_command_frame_id",
            "velocity_command_linear_x",
            "velocity_command_linear_y",
            "velocity_command_linear_z",
            "velocity_command_angular_x",
            "velocity_command_angular_y",
            "velocity_command_angular_z",
        ]
        fields += [
            "ellipse_row",
            "ellipse_time_ns",
            "ellipse_dt_ms",
            "ellipse_valid",
        ]
        fields += self.online_imu_fieldnames("ellipse")
        fields += [
            "gps_row",
            "gps_time_ns",
            "gps_dt_ms",
            "gps_valid",
            "gps_fix_valid",
            "gps_status",
            "gps_service",
            "gps_latitude",
            "gps_longitude",
            "gps_altitude",
        ]
        for prefix in ["rgb", "depth"]:
            fields += [
                f"{prefix}_row",
                f"{prefix}_time_ns",
                f"{prefix}_dt_ms",
                f"{prefix}_valid",
                f"{prefix}_frame_id",
                f"{prefix}_height",
                f"{prefix}_width",
                f"{prefix}_encoding",
                f"{prefix}_step",
                f"{prefix}_payload_file",
                f"{prefix}_payload_offset",
                f"{prefix}_payload_size",
            ]
        return fields

    def online_imu_fieldnames(self, prefix: str) -> list[str]:
        return [
            f"{prefix}_frame_id",
            f"{prefix}_orientation_x",
            f"{prefix}_orientation_y",
            f"{prefix}_orientation_z",
            f"{prefix}_orientation_w",
            f"{prefix}_angular_velocity_x",
            f"{prefix}_angular_velocity_y",
            f"{prefix}_angular_velocity_z",
            f"{prefix}_linear_acceleration_x",
            f"{prefix}_linear_acceleration_y",
            f"{prefix}_linear_acceleration_z",
        ]

    def open_online_master_writer(self) -> None:
        if self.online_master_path is None:
            return
        self.online_master_file = self.online_master_path.open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self.online_master_writer = csv.DictWriter(
            self.online_master_file,
            fieldnames=self.online_master_fieldnames(),
            extrasaction="ignore",
        )
        self.online_master_writer.writeheader()

    def row_time_ns(self, row: dict[str, Any]) -> int | None:
        for key in ("_msg_time_ns", "_recv_time_ns"):
            value = row.get(key)
            if value in (None, ""):
                continue
            try:
                return int(value)
            except (TypeError, ValueError):
                try:
                    return int(float(value))
                except (TypeError, ValueError):
                    continue
        return None

    def row_value(self, row: dict[str, Any] | None, *keys: str) -> Any:
        if row is None:
            return ""
        for key in keys:
            value = row.get(key)
            if value not in (None, ""):
                return value
        return ""

    def update_online_latest(
        self,
        label: str,
        row: dict[str, Any],
        row_index: int,
    ) -> None:
        if not self.online_master_latest_enabled:
            return
        self.latest_online_rows[label] = {
            "row": row,
            "row_index": row_index,
            "time_ns": self.row_time_ns(row),
        }

    def online_master_tolerance_ms(self, label: str) -> float:
        if label == "ellipse":
            return self.online_master_ellipse_max_dt_ms
        if label == "gps":
            return self.online_master_gps_max_dt_ms
        if label == "rgb":
            return self.online_master_rgb_max_dt_ms
        if label == "depth":
            return self.online_master_depth_max_dt_ms
        if label == "velocity_command":
            return self.online_master_velocity_command_max_dt_ms
        return 0.0

    def add_online_match_metadata(
        self,
        output: dict[str, Any],
        label: str,
        latest: dict[str, Any] | None,
        master_time_ns: int | None,
    ) -> bool:
        if latest is None:
            output[f"{label}_row"] = ""
            output[f"{label}_time_ns"] = ""
            output[f"{label}_dt_ms"] = ""
            output[f"{label}_valid"] = 0
            return False

        sample_time_ns = latest.get("time_ns")
        output[f"{label}_row"] = latest.get("row_index", "")
        output[f"{label}_time_ns"] = "" if sample_time_ns is None else sample_time_ns
        self.online_master_match_counts[label] += 1

        if master_time_ns is None or sample_time_ns is None:
            output[f"{label}_dt_ms"] = ""
            output[f"{label}_valid"] = 0
            return False

        dt_ns = master_time_ns - int(sample_time_ns)
        output[f"{label}_dt_ms"] = f"{dt_ns / 1_000_000.0:.3f}"
        max_dt_ns = int(self.online_master_tolerance_ms(label) * 1_000_000.0)
        valid = 0 <= dt_ns <= max_dt_ns
        output[f"{label}_valid"] = 1 if valid else 0
        if valid:
            self.online_master_valid_counts[label] += 1
        return valid

    def add_online_imu_fields(
        self,
        output: dict[str, Any],
        prefix: str,
        row: dict[str, Any] | None,
    ) -> None:
        output[f"{prefix}_frame_id"] = self.row_value(row, "frame_id", "header.frame_id")
        output[f"{prefix}_orientation_x"] = self.row_value(row, "orientation_x", "orientation.x")
        output[f"{prefix}_orientation_y"] = self.row_value(row, "orientation_y", "orientation.y")
        output[f"{prefix}_orientation_z"] = self.row_value(row, "orientation_z", "orientation.z")
        output[f"{prefix}_orientation_w"] = self.row_value(row, "orientation_w", "orientation.w")
        output[f"{prefix}_angular_velocity_x"] = self.row_value(row, "angular_velocity_x", "angular_velocity.x")
        output[f"{prefix}_angular_velocity_y"] = self.row_value(row, "angular_velocity_y", "angular_velocity.y")
        output[f"{prefix}_angular_velocity_z"] = self.row_value(row, "angular_velocity_z", "angular_velocity.z")
        output[f"{prefix}_linear_acceleration_x"] = self.row_value(row, "linear_acceleration_x", "linear_acceleration.x")
        output[f"{prefix}_linear_acceleration_y"] = self.row_value(row, "linear_acceleration_y", "linear_acceleration.y")
        output[f"{prefix}_linear_acceleration_z"] = self.row_value(row, "linear_acceleration_z", "linear_acceleration.z")

    def add_online_gps_fields(
        self,
        output: dict[str, Any],
        prefix: str,
        row: dict[str, Any] | None,
    ) -> None:
        status = self.row_value(row, "status", "status.status")
        output[f"{prefix}_status"] = status
        output[f"{prefix}_service"] = self.row_value(row, "service", "status.service")
        output[f"{prefix}_latitude"] = self.row_value(row, "latitude")
        output[f"{prefix}_longitude"] = self.row_value(row, "longitude")
        output[f"{prefix}_altitude"] = self.row_value(row, "altitude")
        try:
            output[f"{prefix}_fix_valid"] = 1 if int(status) >= 0 else 0
        except (TypeError, ValueError):
            output[f"{prefix}_fix_valid"] = 0

    def add_online_image_fields(
        self,
        output: dict[str, Any],
        prefix: str,
        row: dict[str, Any] | None,
    ) -> None:
        output[f"{prefix}_frame_id"] = self.row_value(row, "frame_id", "header.frame_id")
        output[f"{prefix}_height"] = self.row_value(row, "height")
        output[f"{prefix}_width"] = self.row_value(row, "width")
        output[f"{prefix}_encoding"] = self.row_value(row, "encoding")
        output[f"{prefix}_step"] = self.row_value(row, "step")
        output[f"{prefix}_payload_file"] = self.row_value(row, "payload_file", "data__payload_file")
        output[f"{prefix}_payload_offset"] = self.row_value(row, "payload_offset")
        output[f"{prefix}_payload_size"] = self.row_value(row, "payload_size", "data__payload_size")

    def add_online_twist_fields(
        self,
        output: dict[str, Any],
        prefix: str,
        row: dict[str, Any] | None,
    ) -> None:
        output[f"{prefix}_frame_id"] = self.row_value(row, "frame_id", "header.frame_id")
        output[f"{prefix}_linear_x"] = self.row_value(row, "linear_x", "twist.linear.x")
        output[f"{prefix}_linear_y"] = self.row_value(row, "linear_y", "twist.linear.y")
        output[f"{prefix}_linear_z"] = self.row_value(row, "linear_z", "twist.linear.z")
        output[f"{prefix}_angular_x"] = self.row_value(row, "angular_x", "twist.angular.x")
        output[f"{prefix}_angular_y"] = self.row_value(row, "angular_y", "twist.angular.y")
        output[f"{prefix}_angular_z"] = self.row_value(row, "angular_z", "twist.angular.z")

    def write_online_master_latest(
        self,
        robot_row: dict[str, Any],
        robot_row_index: int,
    ) -> None:
        if not self.online_master_latest_enabled or self.online_master_writer is None:
            return

        master_time_ns = self.row_time_ns(robot_row)
        output: dict[str, Any] = {
            "master_index": self.online_master_rows,
            "master_time_ns": "" if master_time_ns is None else master_time_ns,
            "master_recv_time_ns": self.row_value(robot_row, "_recv_time_ns"),
            "master_write_time_ns": self.get_clock().now().nanoseconds,
            "robot_row": robot_row_index,
        }
        self.add_online_imu_fields(output, "robot", robot_row)

        for label in ["velocity_command", "ellipse", "gps", "rgb", "depth"]:
            latest = self.latest_online_rows.get(label)
            valid = self.add_online_match_metadata(output, label, latest, master_time_ns)
            row = latest["row"] if latest is not None and valid else None
            if label == "velocity_command":
                self.add_online_twist_fields(output, label, row)
            elif label == "ellipse":
                self.add_online_imu_fields(output, label, row)
            elif label == "gps":
                self.add_online_gps_fields(output, label, row)
            else:
                self.add_online_image_fields(output, label, row)

        clean = {key: "" if value is None else value for key, value in output.items()}
        self.online_master_writer.writerow(clean)
        self.online_master_rows += 1
        if (
            self.flush_every_n_rows > 0
            and self.online_master_file is not None
            and self.online_master_rows % self.flush_every_n_rows == 0
        ):
            self.online_master_file.flush()

    def increment_row_count(self, topic: str) -> int:
        row_index = self.row_counts.get(topic, 0)
        count = row_index + 1
        self.row_counts[topic] = count
        if self.flush_every_n_rows > 0 and count % self.flush_every_n_rows == 0:
            writer = self.writers.get(topic)
            if writer is not None:
                writer.file.flush()
            binary_writer = self.binary_writers.get(topic)
            if binary_writer is not None:
                binary_writer.file.flush()
            if self.online_master_file is not None:
                self.online_master_file.flush()
        return row_index

    def binary_writer_for_topic(self, topic: str) -> AppendBinaryWriter:
        if topic not in self.binary_writers:
            self.binary_writers[topic] = AppendBinaryWriter(
                self.payload_dir / f"{topic_slug(topic)}.bin"
            )
        return self.binary_writers[topic]

    def record_velocity_command(self, msg: TwistStamped) -> None:
        if not self.run_active:
            return
        topic = self.velocity_command_topic
        row = {
            "_recv_time_ns": self.get_clock().now().nanoseconds,
            "_msg_time_ns": self.message_time_ns(msg),
            "_topic": topic,
            "frame_id": msg.header.frame_id,
            "linear_x": msg.twist.linear.x,
            "linear_y": msg.twist.linear.y,
            "linear_z": msg.twist.linear.z,
            "angular_x": msg.twist.angular.x,
            "angular_y": msg.twist.angular.y,
            "angular_z": msg.twist.angular.z,
        }
        self.writer_for_topic(topic).write(row)
        row_index = self.increment_row_count(topic)
        self.update_online_latest("velocity_command", row, row_index)

    def record_gps(self, msg: NavSatFix) -> None:
        if not self.run_active:
            return
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
        row_index = self.increment_row_count(topic)
        self.update_online_latest("gps", row, row_index)

    def imu_row(self, topic: str, msg: Imu, recv_ns: int) -> dict[str, Any]:
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
        return row

    def record_robot_imu(self, msg: Imu) -> None:
        if not self.run_active:
            return
        topic = self.robot_imu_topic
        recv_ns = self.get_clock().now().nanoseconds
        row = self.imu_row(topic, msg, recv_ns)
        self.writer_for_topic(topic).write(row)
        row_index = self.increment_row_count(topic)
        self.write_online_master_latest(row, row_index)

    def record_orientation(self, msg: Imu) -> None:
        if not self.run_active:
            return
        topic = self.orientation_topic
        recv_ns = self.get_clock().now().nanoseconds
        row = self.imu_row(topic, msg, recv_ns)

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
        row_index = self.increment_row_count(topic)
        self.update_online_latest("ellipse", row, row_index)

    def record_camera_info_msg(self, topic: str, msg: CameraInfo) -> None:
        if not self.run_active:
            return
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
        if not self.run_active:
            return
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
        row_index = self.increment_row_count(topic)
        if topic == self.rgb_topic:
            self.update_online_latest("rgb", row, row_index)
        elif topic == self.depth_topic:
            self.update_online_latest("depth", row, row_index)

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
        if self.topic_dir is None:
            raise RuntimeError("Cannot create a topic writer before a recording run has started.")
        if topic not in self.writers:
            self.writers[topic] = TopicCsvWriter(self.topic_dir / f"{topic_slug(topic)}.csv")
        return self.writers[topic]

    def next_payload_path(self, topic: str, suffix: str) -> Path:
        if self.payload_dir is None:
            raise RuntimeError("Cannot create a payload path before a recording run has started.")
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
        if not self.run_active:
            return
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

    def write_online_master_metadata(self, ended_at_ns: int) -> None:
        if not self.online_master_latest_enabled or self.run_dir is None or self.online_master_path is None:
            return
        metadata = {
            "run_id": self.run_id,
            "run_type": self.run_type,
            "started_at_iso": self.started_at_iso,
            "ended_at_iso": iso_now(),
            "started_at_ros_time_ns": self.started_at_ns,
            "ended_at_ros_time_ns": ended_at_ns,
            "output": str(self.online_master_path),
            "policy": "latest_received_on_each_robot_imu",
            "time_source": "_msg_time_ns_with_recv_fallback",
            "topics": {
                "robot": self.robot_imu_topic,
                "velocity_command": self.velocity_command_topic,
                "ellipse": self.orientation_topic,
                "gps": self.gps_topic,
                "rgb": self.rgb_topic,
                "depth": self.depth_topic,
            },
            "tolerances_ms": {
                "velocity_command": self.online_master_velocity_command_max_dt_ms,
                "ellipse": self.online_master_ellipse_max_dt_ms,
                "gps": self.online_master_gps_max_dt_ms,
                "rgb": self.online_master_rgb_max_dt_ms,
                "depth": self.online_master_depth_max_dt_ms,
            },
            "master_rows": self.online_master_rows,
            "match_counts": self.online_master_match_counts,
            "valid_counts": self.online_master_valid_counts,
            "note": (
                "Low-latency online master. Sensor fields are populated only from "
                "latest received samples whose timestamps are not newer than the "
                "robot IMU timestamp and are within tolerance."
            ),
        }
        with (self.run_dir / "master_latest_online.metadata.json").open(
            "w",
            encoding="utf-8",
        ) as stream:
            json.dump(metadata, stream, indent=2)

    def stop_run(self, reason: str) -> bool:
        if not self.run_active:
            self.get_logger().info(f"Ignoring stop request from {reason}; recorder is idle.")
            return False

        if self.run_dir is None:
            self.run_active = False
            self.initialize_inactive_run_state()
            return False

        run_dir = self.run_dir

        for writer in self.writers.values():
            writer.close()
        for binary_writer in self.binary_writers.values():
            binary_writer.close()

        ended_at_ns = self.get_clock().now().nanoseconds
        if self.online_master_file is not None:
            self.online_master_file.flush()
            self.online_master_file.close()
            self.online_master_file = None
        self.write_online_master_metadata(ended_at_ns)

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
            "online_master_latest_enabled": self.online_master_latest_enabled,
            "online_master_latest_rows": self.online_master_rows,
            "online_master_latest_path": (
                str(self.online_master_path.relative_to(self.run_dir))
                if self.online_master_latest_enabled
                else ""
            ),
        }
        with (run_dir / "run_summary.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summary.keys()))
            writer.writeheader()
            writer.writerow(summary)

        self.get_logger().info(f"Finished recording run from {reason}: {run_dir}")
        self.run_active = False
        self.initialize_inactive_run_state()
        return True

    def close(self) -> None:
        if self.node_closed:
            return
        self.node_closed = True
        if self.run_active:
            self.stop_run("node shutdown")
        if self.remote_control_socket is not None:
            self.remote_control_socket.close()
            self.remote_control_socket = None


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
