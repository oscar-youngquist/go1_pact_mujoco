#!/usr/bin/python3
"""Generate a 100 Hz master synchronization CSV from a Go1 training run directory."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def topic_slug(topic: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "_", topic.strip("/"))
    return slug.strip("_") or "root"


def parse_int(value: Any) -> int | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(text)
    except ValueError:
        try:
            return int(float(text))
        except ValueError:
            return None


def parse_float(value: Any) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        result = float(text)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def stringify(value: Any) -> str:
    if value is None:
        return ""
    return str(value)


def row_time_ns(row: dict[str, str], prefer_msg_time: bool) -> int | None:
    if prefer_msg_time:
        return parse_int(row.get("_msg_time_ns")) or parse_int(row.get("_recv_time_ns"))
    return parse_int(row.get("_recv_time_ns")) or parse_int(row.get("_msg_time_ns"))


def get_any(row: dict[str, str], *keys: str) -> str:
    for key in keys:
        value = row.get(key)
        if value not in (None, ""):
            return value
    return ""


@dataclass
class TimedRow:
    time_ns: int
    row_index: int
    row: dict[str, str]


@dataclass
class TopicStream:
    label: str
    topic: str
    path: Path
    rows: list[TimedRow]
    times: list[int]


@dataclass
class Match:
    item: TimedRow
    dt_ns: int
    valid: bool


def read_metadata(run_dir: Path) -> dict[str, Any]:
    path = run_dir / "metadata.json"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def selected_topic(metadata: dict[str, Any], key: str, fallback: str) -> str:
    selected = metadata.get("selected_topics", {})
    if isinstance(selected, dict):
        value = selected.get(key)
        if isinstance(value, str) and value:
            return value
    return fallback


def topic_csv_path(run_dir: Path, topic: str) -> Path:
    return run_dir / "topics" / f"{topic_slug(topic)}.csv"


def load_stream(
    run_dir: Path,
    label: str,
    topic: str,
    prefer_msg_time: bool,
) -> TopicStream:
    path = topic_csv_path(run_dir, topic)
    if not path.exists():
        return TopicStream(label=label, topic=topic, path=path, rows=[], times=[])

    rows: list[TimedRow] = []
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row_index, row in enumerate(reader):
            time_ns = row_time_ns(row, prefer_msg_time)
            if time_ns is None:
                continue
            rows.append(TimedRow(time_ns=time_ns, row_index=row_index, row=row))

    rows.sort(key=lambda item: item.time_ns)
    return TopicStream(
        label=label,
        topic=topic,
        path=path,
        rows=rows,
        times=[item.time_ns for item in rows],
    )


def match_stream(
    stream: TopicStream,
    master_time_ns: int,
    policy: str,
    max_dt_ns: int,
) -> Match | None:
    if not stream.rows:
        return None

    index = bisect.bisect_right(stream.times, master_time_ns)
    candidates: list[TimedRow] = []

    if policy == "latest":
        if index == 0:
            return None
        candidates.append(stream.rows[index - 1])
    elif policy == "nearest":
        if index > 0:
            candidates.append(stream.rows[index - 1])
        if index < len(stream.rows):
            candidates.append(stream.rows[index])
    else:
        raise ValueError(f"Unsupported match policy: {policy}")

    if not candidates:
        return None

    item = min(candidates, key=lambda candidate: abs(master_time_ns - candidate.time_ns))
    dt_ns = master_time_ns - item.time_ns
    return Match(item=item, dt_ns=dt_ns, valid=abs(dt_ns) <= max_dt_ns)


def add_match_metadata(out: dict[str, Any], prefix: str, match: Match | None) -> None:
    if match is None:
        out[f"{prefix}_row"] = ""
        out[f"{prefix}_time_ns"] = ""
        out[f"{prefix}_dt_ms"] = ""
        out[f"{prefix}_valid"] = 0
        return

    out[f"{prefix}_row"] = match.item.row_index
    out[f"{prefix}_time_ns"] = match.item.time_ns
    out[f"{prefix}_dt_ms"] = f"{match.dt_ns / 1_000_000.0:.3f}"
    out[f"{prefix}_valid"] = 1 if match.valid else 0


def add_imu_fields(out: dict[str, Any], prefix: str, row: dict[str, str] | None) -> None:
    source = row or {}
    out[f"{prefix}_frame_id"] = get_any(source, "frame_id", "header.frame_id")
    out[f"{prefix}_orientation_x"] = get_any(source, "orientation_x", "orientation.x")
    out[f"{prefix}_orientation_y"] = get_any(source, "orientation_y", "orientation.y")
    out[f"{prefix}_orientation_z"] = get_any(source, "orientation_z", "orientation.z")
    out[f"{prefix}_orientation_w"] = get_any(source, "orientation_w", "orientation.w")
    out[f"{prefix}_angular_velocity_x"] = get_any(source, "angular_velocity_x", "angular_velocity.x")
    out[f"{prefix}_angular_velocity_y"] = get_any(source, "angular_velocity_y", "angular_velocity.y")
    out[f"{prefix}_angular_velocity_z"] = get_any(source, "angular_velocity_z", "angular_velocity.z")
    out[f"{prefix}_linear_acceleration_x"] = get_any(source, "linear_acceleration_x", "linear_acceleration.x")
    out[f"{prefix}_linear_acceleration_y"] = get_any(source, "linear_acceleration_y", "linear_acceleration.y")
    out[f"{prefix}_linear_acceleration_z"] = get_any(source, "linear_acceleration_z", "linear_acceleration.z")


def add_gps_fields(out: dict[str, Any], prefix: str, row: dict[str, str] | None) -> None:
    source = row or {}
    out[f"{prefix}_status"] = get_any(source, "status", "status.status")
    out[f"{prefix}_service"] = get_any(source, "service", "status.service")
    out[f"{prefix}_latitude"] = get_any(source, "latitude")
    out[f"{prefix}_longitude"] = get_any(source, "longitude")
    out[f"{prefix}_altitude"] = get_any(source, "altitude")


def add_image_fields(out: dict[str, Any], prefix: str, row: dict[str, str] | None) -> None:
    source = row or {}
    out[f"{prefix}_frame_id"] = get_any(source, "frame_id", "header.frame_id")
    out[f"{prefix}_height"] = get_any(source, "height")
    out[f"{prefix}_width"] = get_any(source, "width")
    out[f"{prefix}_encoding"] = get_any(source, "encoding")
    out[f"{prefix}_step"] = get_any(source, "step")
    out[f"{prefix}_payload_file"] = get_any(source, "payload_file", "data__payload_file")
    out[f"{prefix}_payload_offset"] = get_any(source, "payload_offset")
    out[f"{prefix}_payload_size"] = get_any(source, "payload_size", "data__payload_size")


def add_twist_fields(out: dict[str, Any], prefix: str, row: dict[str, str] | None) -> None:
    source = row or {}
    out[f"{prefix}_frame_id"] = get_any(source, "frame_id", "header.frame_id")
    out[f"{prefix}_linear_x"] = get_any(source, "linear_x", "twist.linear.x")
    out[f"{prefix}_linear_y"] = get_any(source, "linear_y", "twist.linear.y")
    out[f"{prefix}_linear_z"] = get_any(source, "linear_z", "twist.linear.z")
    out[f"{prefix}_angular_x"] = get_any(source, "angular_x", "twist.angular.x")
    out[f"{prefix}_angular_y"] = get_any(source, "angular_y", "twist.angular.y")
    out[f"{prefix}_angular_z"] = get_any(source, "angular_z", "twist.angular.z")


def master_fieldnames() -> list[str]:
    fields = [
        "master_index",
        "master_time_ns",
        "master_recv_time_ns",
    ]
    for prefix in ["robot"]:
        fields += [
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
    for prefix in ["ellipse"]:
        fields += [
            f"{prefix}_row",
            f"{prefix}_time_ns",
            f"{prefix}_dt_ms",
            f"{prefix}_valid",
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
    fields += [
        "gps_row",
        "gps_time_ns",
        "gps_dt_ms",
        "gps_valid",
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


def tolerance_ns(ms: float) -> int:
    return int(ms * 1_000_000.0)


def generate_master(args: argparse.Namespace) -> dict[str, Any]:
    run_dir = Path(args.run_dir).expanduser().resolve()
    metadata = read_metadata(run_dir)
    prefer_msg_time = not args.use_recv_time

    topics = {
        "robot": args.robot_imu_topic or selected_topic(metadata, "robot_imu", "/go1/imu"),
        "velocity_command": args.velocity_command_topic or selected_topic(metadata, "velocity_command", "/go1/velocity_command"),
        "ellipse": args.ellipse_imu_topic or selected_topic(metadata, "orientation", "/go1/sensors/ellipse/imu/data"),
        "gps": args.gps_topic or selected_topic(metadata, "gps", "/go1/sensors/gps/fix"),
        "rgb": args.rgb_topic or selected_topic(metadata, "rgb", "/go1/sensors/camera/color/image_raw"),
        "depth": args.depth_topic or selected_topic(metadata, "depth", "/go1/sensors/camera/depth/image_raw"),
    }

    streams = {
        label: load_stream(run_dir, label, topic, prefer_msg_time)
        for label, topic in topics.items()
    }

    robot = streams["robot"]
    if not robot.rows:
        raise RuntimeError(
            f"No robot IMU rows found at {robot.path}. "
            "Record /go1/imu first or pass --robot-imu-topic."
        )

    tolerances = {
        "velocity_command": tolerance_ns(args.velocity_command_max_dt_ms),
        "ellipse": tolerance_ns(args.ellipse_max_dt_ms),
        "gps": tolerance_ns(args.gps_max_dt_ms),
        "rgb": tolerance_ns(args.rgb_max_dt_ms),
        "depth": tolerance_ns(args.depth_max_dt_ms),
    }
    policies = {
        "velocity_command": args.velocity_command_policy,
        "ellipse": args.ellipse_policy,
        "gps": args.gps_policy,
        "rgb": args.rgb_policy,
        "depth": args.depth_policy,
    }

    output_path = Path(args.output).expanduser().resolve() if args.output else run_dir / "master_100hz.csv"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    match_counts = {label: 0 for label in ["velocity_command", "ellipse", "gps", "rgb", "depth"]}
    valid_counts = {label: 0 for label in ["velocity_command", "ellipse", "gps", "rgb", "depth"]}

    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=master_fieldnames(), extrasaction="ignore")
        writer.writeheader()

        for master_index, master in enumerate(robot.rows):
            out: dict[str, Any] = {
                "master_index": master_index,
                "master_time_ns": master.time_ns,
                "master_recv_time_ns": get_any(master.row, "_recv_time_ns"),
            }
            add_imu_fields(out, "robot", master.row)

            for label in ["velocity_command", "ellipse", "gps", "rgb", "depth"]:
                match = match_stream(
                    streams[label],
                    master.time_ns,
                    policies[label],
                    tolerances[label],
                )
                add_match_metadata(out, label, match)
                if match is not None:
                    match_counts[label] += 1
                    if match.valid:
                        valid_counts[label] += 1

                row = match.item.row if match is not None and match.valid else None
                if label == "velocity_command":
                    add_twist_fields(out, label, row)
                elif label == "ellipse":
                    add_imu_fields(out, label, row)
                elif label == "gps":
                    add_gps_fields(out, label, row)
                else:
                    add_image_fields(out, label, row)

            writer.writerow({key: stringify(value) for key, value in out.items()})

    summary = {
        "run_dir": str(run_dir),
        "output": str(output_path),
        "time_source": "_recv_time_ns" if args.use_recv_time else "_msg_time_ns_with_recv_fallback",
        "topics": topics,
        "source_csvs": {label: str(stream.path) for label, stream in streams.items()},
        "row_counts": {label: len(stream.rows) for label, stream in streams.items()},
        "match_policies": policies,
        "tolerances_ms": {
            "velocity_command": args.velocity_command_max_dt_ms,
            "ellipse": args.ellipse_max_dt_ms,
            "gps": args.gps_max_dt_ms,
            "rgb": args.rgb_max_dt_ms,
            "depth": args.depth_max_dt_ms,
        },
        "match_counts": match_counts,
        "valid_counts": valid_counts,
    }
    summary_path = output_path.with_suffix(".metadata.json")
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
    return summary


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate master_100hz.csv by aligning native-rate sensor recordings to robot IMU rows."
    )
    parser.add_argument("run_dir", help="Training run directory containing metadata.json and topics/*.csv")
    parser.add_argument("--output", help="Output CSV path. Defaults to RUN_DIR/master_100hz.csv")
    parser.add_argument("--use-recv-time", action="store_true", help="Use _recv_time_ns instead of message header time")

    parser.add_argument("--robot-imu-topic", default="", help="Master robot IMU topic. Defaults to metadata or /go1/imu")
    parser.add_argument("--velocity-command-topic", default="", help="Velocity command topic")
    parser.add_argument("--ellipse-imu-topic", default="", help="Ellipse IMU/orientation topic")
    parser.add_argument("--gps-topic", default="", help="GPS topic")
    parser.add_argument("--rgb-topic", default="", help="RGB image topic")
    parser.add_argument("--depth-topic", default="", help="Depth image topic")

    parser.add_argument("--velocity-command-policy", choices=["nearest", "latest"], default="latest")
    parser.add_argument("--ellipse-policy", choices=["nearest", "latest"], default="nearest")
    parser.add_argument("--gps-policy", choices=["nearest", "latest"], default="latest")
    parser.add_argument("--rgb-policy", choices=["nearest", "latest"], default="nearest")
    parser.add_argument("--depth-policy", choices=["nearest", "latest"], default="nearest")

    parser.add_argument("--velocity-command-max-dt-ms", type=float, default=50.0)
    parser.add_argument("--ellipse-max-dt-ms", type=float, default=20.0)
    parser.add_argument("--gps-max-dt-ms", type=float, default=1000.0)
    parser.add_argument("--rgb-max-dt-ms", type=float, default=100.0)
    parser.add_argument("--depth-max-dt-ms", type=float, default=100.0)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    summary = generate_master(args)
    print(f"Wrote {summary['output']}")
    print(f"Wrote {Path(summary['output']).with_suffix('.metadata.json')}")
    print(json.dumps({
        "row_counts": summary["row_counts"],
        "valid_counts": summary["valid_counts"],
    }, indent=2))


if __name__ == "__main__":
    main()
