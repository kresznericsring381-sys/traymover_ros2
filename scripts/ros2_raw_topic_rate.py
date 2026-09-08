#!/usr/bin/env python3
"""Measure ROS 2 topic delivery using raw serialized subscriptions.

`ros2 topic hz` deserializes every message in Python. For large PointCloud2
messages that can become the bottleneck being measured. This probe subscribes
with raw=True so it measures DDS/player delivery with much lower overhead.
"""

import argparse
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("topic")
    parser.add_argument("--type", default="sensor_msgs/msg/PointCloud2")
    parser.add_argument("--interval-sec", type=float, default=2.0)
    parser.add_argument("--qos", choices=("sensor_data", "reliable"), default="sensor_data")
    return parser.parse_args()


def make_qos(name):
    if name == "reliable":
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def main():
    args = parse_args()
    msg_type = get_message(args.type)
    interval_sec = max(0.25, args.interval_sec)

    rclpy.init()
    node = rclpy.create_node("raw_topic_rate_probe")

    counts = {"total": 0, "interval": 0, "bytes": 0}
    last_msg_wall = {"time": None, "min_gap": None, "max_gap": None}
    started = time.monotonic()
    last_report = started

    def on_msg(serialized_msg):
        now = time.monotonic()
        previous = last_msg_wall["time"]
        if previous is not None:
            gap = now - previous
            last_msg_wall["min_gap"] = gap if last_msg_wall["min_gap"] is None else min(last_msg_wall["min_gap"], gap)
            last_msg_wall["max_gap"] = gap if last_msg_wall["max_gap"] is None else max(last_msg_wall["max_gap"], gap)
        last_msg_wall["time"] = now
        counts["total"] += 1
        counts["interval"] += 1
        counts["bytes"] += len(serialized_msg)

    node.create_subscription(msg_type, args.topic, on_msg, make_qos(args.qos), raw=True)
    print(f"topic={args.topic} type={args.type} qos={args.qos} raw=true")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_report >= interval_sec:
                elapsed = now - last_report
                total_elapsed = now - started
                rate = counts["interval"] / elapsed if elapsed > 0.0 else 0.0
                avg_size = counts["bytes"] / counts["interval"] if counts["interval"] else 0.0
                min_gap = last_msg_wall["min_gap"]
                max_gap = last_msg_wall["max_gap"]
                print(
                    f"average rate: {rate:.3f} Hz total={counts['total']} "
                    f"elapsed={total_elapsed:.1f}s avg_serialized_bytes={avg_size:.0f} "
                    f"min_gap={'-' if min_gap is None else f'{min_gap:.3f}s'} "
                    f"max_gap={'-' if max_gap is None else f'{max_gap:.3f}s'}"
                )
                counts["interval"] = 0
                counts["bytes"] = 0
                last_msg_wall["min_gap"] = None
                last_msg_wall["max_gap"] = None
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
