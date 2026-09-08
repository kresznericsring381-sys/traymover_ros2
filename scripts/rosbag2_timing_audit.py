#!/usr/bin/env python3
"""Audit rosbag2 timing without running ros2 bag play.

This separates three clocks that are easy to conflate while debugging replay:

* rosbag2 record timestamp, used by the player for scheduling
* message header timestamp, used by FAST-LIO and diagnostics
* PointCloud2 per-point time field, used by FAST-LIO for scan undistortion
"""

import argparse
import math
import os
import re
import struct
import sys
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import PointCloud2, PointField
except ImportError as exc:  # pragma: no cover - intended for remote ROS 2 hosts
    print(
        "ROS 2 Python modules are required. Run after sourcing /opt/ros/humble/setup.bash "
        "and install/setup.bash.",
        file=sys.stderr,
    )
    raise SystemExit(str(exc))


DEFAULT_TOPICS = [
    "/point_cloud_raw",
    "/imu/data_raw",
    "/camera/camera/color/image_raw",
    "/camera/camera/depth/image_rect_raw",
]

DATATYPES = {
    PointField.INT8: ("b", 1, "INT8"),
    PointField.UINT8: ("B", 1, "UINT8"),
    PointField.INT16: ("h", 2, "INT16"),
    PointField.UINT16: ("H", 2, "UINT16"),
    PointField.INT32: ("i", 4, "INT32"),
    PointField.UINT32: ("I", 4, "UINT32"),
    PointField.FLOAT32: ("f", 4, "FLOAT32"),
    PointField.FLOAT64: ("d", 8, "FLOAT64"),
}


@dataclass
class TopicStats:
    name: str
    type_name: str
    count: int = 0
    record_first: Optional[float] = None
    record_last: Optional[float] = None
    header_first: Optional[float] = None
    header_last: Optional[float] = None
    last_record: Optional[float] = None
    last_header: Optional[float] = None
    record_gaps: List[float] = field(default_factory=list)
    header_gaps: List[float] = field(default_factory=list)
    record_backward: int = 0
    header_backward: int = 0
    header_missing: int = 0
    point_min: Optional[int] = None
    point_max: Optional[int] = None
    point_total: int = 0
    cloud_fields: List[str] = field(default_factory=list)
    point_time_field: Optional[str] = None
    point_time_min: Optional[float] = None
    point_time_max: Optional[float] = None
    point_time_samples: int = 0
    point_time_clouds: int = 0

    def update_record_time(self, timestamp_ns: int) -> None:
        timestamp = timestamp_ns / 1.0e9
        if self.record_first is None:
            self.record_first = timestamp
        if self.last_record is not None:
            gap = timestamp - self.last_record
            self.record_gaps.append(gap)
            if gap < 0.0:
                self.record_backward += 1
        self.last_record = timestamp
        self.record_last = timestamp

    def update_header_time(self, timestamp: Optional[float]) -> None:
        if timestamp is None:
            self.header_missing += 1
            return
        if self.header_first is None:
            self.header_first = timestamp
        if self.last_header is not None:
            gap = timestamp - self.last_header
            self.header_gaps.append(gap)
            if gap < 0.0:
                self.header_backward += 1
        self.last_header = timestamp
        self.header_last = timestamp

    def update_point_count(self, points: int) -> None:
        self.point_total += points
        self.point_min = points if self.point_min is None else min(self.point_min, points)
        self.point_max = points if self.point_max is None else max(self.point_max, points)

    def update_point_time(self, values: Sequence[float]) -> None:
        if not values:
            return
        local_min = min(values)
        local_max = max(values)
        self.point_time_min = local_min if self.point_time_min is None else min(self.point_time_min, local_min)
        self.point_time_max = local_max if self.point_time_max is None else max(self.point_time_max, local_max)
        self.point_time_samples += len(values)
        self.point_time_clouds += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir")
    parser.add_argument("--topics", nargs="*", default=DEFAULT_TOPICS)
    parser.add_argument("--gap-threshold-sec", type=float, default=0.15)
    parser.add_argument("--sample-clouds", type=int, default=5)
    parser.add_argument("--sample-points", type=int, default=2048)
    parser.add_argument("--max-messages-per-topic", type=int, default=0)
    parser.add_argument("--storage-id", default="")
    return parser.parse_args()


def infer_storage_id(bag_dir: str) -> str:
    metadata = os.path.join(bag_dir, "metadata.yaml")
    try:
        with open(metadata, "r", encoding="utf-8") as stream:
            text = stream.read()
    except OSError:
        return "sqlite3"

    match = re.search(r"^\s*storage_identifier:\s*['\"]?([^'\"\s]+)", text, re.MULTILINE)
    if match:
        return match.group(1)
    return "sqlite3"


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


def header_stamp_sec(msg) -> Optional[float]:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return stamp_to_sec(stamp)


def percentile(values: Sequence[float], pct: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * pct))
    return ordered[idx]


def gap_summary(gaps: Sequence[float], threshold: float) -> Tuple[int, Optional[float], Optional[float], Optional[float]]:
    if not gaps:
        return 0, None, None, None
    large_count = sum(1 for gap in gaps if gap > threshold or gap < 0.0)
    return large_count, max(gaps), percentile(gaps, 0.50), percentile(gaps, 0.95)


def format_rate(count: int, first: Optional[float], last: Optional[float]) -> str:
    if count < 2 or first is None or last is None or last <= first:
        return "-"
    return f"{(count - 1) / (last - first):.3f} Hz over {last - first:.3f} s"


def point_field_name(field) -> str:
    entry = DATATYPES.get(field.datatype)
    datatype = entry[2] if entry else f"UNKNOWN({field.datatype})"
    return f"{field.name}@{field.offset}:{datatype}[{field.count}]"


def choose_time_field(msg: PointCloud2) -> Optional[str]:
    names = {field.name for field in msg.fields}
    for candidate in ("time", "t", "timestamp", "offset_time"):
        if candidate in names:
            return candidate
    return None


def sample_indices(total: int, limit: int) -> Iterable[int]:
    if total <= 0 or limit <= 0:
        return []
    count = min(total, limit)
    if count == total:
        return range(total)
    if count == 1:
        return [0]
    return sorted({int(round(i * (total - 1) / (count - 1))) for i in range(count)})


def sample_point_field(msg: PointCloud2, field_name: str, limit: int) -> List[float]:
    target = next((field for field in msg.fields if field.name == field_name), None)
    if target is None:
        return []
    entry = DATATYPES.get(target.datatype)
    if entry is None:
        return []

    code, size, _ = entry
    total = int(msg.width) * int(msg.height)
    endian = ">" if msg.is_bigendian else "<"
    fmt = endian + code
    data = bytes(msg.data)
    values = []
    for index in sample_indices(total, limit):
        row = index // max(1, int(msg.width))
        col = index % max(1, int(msg.width))
        offset = int(row) * int(msg.row_step) + int(col) * int(msg.point_step) + int(target.offset)
        if offset + size <= len(data):
            values.append(float(struct.unpack_from(fmt, data, offset)[0]))
    return values


def likely_time_unit(max_offset: Optional[float]) -> str:
    if max_offset is None or not math.isfinite(max_offset):
        return "unknown"
    if max_offset <= 0.2:
        return "seconds -> FAST-LIO timestamp_unit=0"
    if max_offset <= 200.0:
        return "milliseconds -> FAST-LIO timestamp_unit=1"
    if max_offset <= 200000.0:
        return "microseconds -> FAST-LIO timestamp_unit=2"
    return "nanoseconds -> FAST-LIO timestamp_unit=3"


def open_reader(bag_dir: str, storage_id: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def audit(args: argparse.Namespace) -> Dict[str, TopicStats]:
    storage_id = args.storage_id or infer_storage_id(args.bag_dir)
    reader = open_reader(args.bag_dir, storage_id)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    selected_topics = [topic for topic in args.topics if topic in topic_types]
    missing_topics = [topic for topic in args.topics if topic not in topic_types]

    print(f"bag={args.bag_dir}")
    print(f"storage_id={storage_id}")
    if missing_topics:
        print(f"missing_topics={','.join(missing_topics)}")
    if not selected_topics:
        return {}

    if hasattr(reader, "set_filter") and hasattr(rosbag2_py, "StorageFilter"):
        try:
            storage_filter = rosbag2_py.StorageFilter(topics=selected_topics)
        except TypeError:
            storage_filter = rosbag2_py.StorageFilter()
            storage_filter.topics = selected_topics
        reader.set_filter(storage_filter)

    message_types = {}
    for topic in selected_topics:
        try:
            message_types[topic] = get_message(topic_types[topic])
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            print(f"skip_topic={topic} reason=message_type_unavailable type={topic_types[topic]} error={exc}")

    stats = {topic: TopicStats(topic, topic_types[topic]) for topic in selected_topics}
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        topic_stats = stats[topic]
        if args.max_messages_per_topic > 0 and topic_stats.count >= args.max_messages_per_topic:
            continue

        msg = deserialize_message(data, message_types[topic])
        topic_stats.count += 1
        topic_stats.update_record_time(timestamp_ns)
        topic_stats.update_header_time(header_stamp_sec(msg))

        if isinstance(msg, PointCloud2):
            points = int(msg.width) * int(msg.height)
            topic_stats.update_point_count(points)
            if not topic_stats.cloud_fields:
                topic_stats.cloud_fields = [point_field_name(field) for field in msg.fields]
                topic_stats.point_time_field = choose_time_field(msg)
            if topic_stats.point_time_field and topic_stats.point_time_clouds < args.sample_clouds:
                values = sample_point_field(msg, topic_stats.point_time_field, args.sample_points)
                topic_stats.update_point_time(values)

    return stats


def print_stats(stats: Dict[str, TopicStats], threshold: float) -> None:
    for topic in sorted(stats):
        entry = stats[topic]
        print("")
        print(f"topic={entry.name}")
        print(f"type={entry.type_name}")
        print(f"messages={entry.count}")
        print(f"record_rate={format_rate(entry.count, entry.record_first, entry.record_last)}")
        print(f"header_rate={format_rate(entry.count - entry.header_missing, entry.header_first, entry.header_last)}")

        record_large, record_max, record_p50, record_p95 = gap_summary(entry.record_gaps, threshold)
        header_large, header_max, header_p50, header_p95 = gap_summary(entry.header_gaps, threshold)
        print(
            "record_gaps="
            f"bad>{threshold:.3f}s_or_backward:{record_large} "
            f"backward:{entry.record_backward} max:{record_max if record_max is not None else '-'} "
            f"p50:{record_p50 if record_p50 is not None else '-'} p95:{record_p95 if record_p95 is not None else '-'}"
        )
        print(
            "header_gaps="
            f"bad>{threshold:.3f}s_or_backward:{header_large} "
            f"backward:{entry.header_backward} missing:{entry.header_missing} "
            f"max:{header_max if header_max is not None else '-'} "
            f"p50:{header_p50 if header_p50 is not None else '-'} p95:{header_p95 if header_p95 is not None else '-'}"
        )

        if entry.cloud_fields:
            avg_points = entry.point_total / entry.count if entry.count else 0.0
            print(f"pointcloud_fields={','.join(entry.cloud_fields)}")
            print(f"point_count=min:{entry.point_min} max:{entry.point_max} avg:{avg_points:.1f}")
            if entry.point_time_field:
                print(
                    f"point_time_field={entry.point_time_field} "
                    f"sample_clouds={entry.point_time_clouds} samples={entry.point_time_samples} "
                    f"min:{entry.point_time_min} max:{entry.point_time_max} "
                    f"likely_unit:{likely_time_unit(entry.point_time_max)}"
                )
            else:
                print("point_time_field=missing")


def main() -> int:
    args = parse_args()
    stats = audit(args)
    print_stats(stats, args.gap_threshold_sec)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
