#!/usr/bin/env python3
"""Persist high-signal diagnostics for the FAST-LIO odometry chain."""

import argparse
import csv
import os
import signal
import threading
import time
from datetime import datetime, timezone

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log as RosoutLog
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--interval-sec', type=float, default=2.0)
    return parser.parse_args()


class FastlioDiagnostics:
    def __init__(self, args):
        self.node = rclpy.create_node(
            'fastlio_diagnostics',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.output_dir = os.path.abspath(args.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
        self.interval_sec = max(0.5, args.interval_sec)
        self.lock = threading.Lock()
        self.started_wall = time.time()
        self.last_rosout_wall = 0.0
        self.counts = {
            'odom': 0,
            'tf': 0,
        }
        self.last_seen_wall = {}
        self.last_odom = None
        self.last_tf = {}
        self.last_state = None
        self._closed = False

        self.events = open(os.path.join(self.output_dir, 'fastlio_events.log'), 'a', buffering=1)
        self.rosout = open(os.path.join(self.output_dir, 'fastlio_rosout.log'), 'a', buffering=1)
        self.metrics = open(os.path.join(self.output_dir, 'fastlio_metrics.csv'), 'a', newline='', buffering=1)
        self.metrics_writer = csv.writer(self.metrics)
        if os.path.getsize(os.path.join(self.output_dir, 'fastlio_metrics.csv')) == 0:
            self.metrics_writer.writerow([
                'wall_time', 'ros_time', 'elapsed_sec', 'event',
                'odom_count', 'odom_age_sec', 'tf_count',
                'camera_init_body_age_sec', 'odom_frame_age_sec',
                'state',
            ])

        qos = rclpy.qos.qos_profile_sensor_data
        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, qos)
        self.node.create_subscription(TFMessage, '/tf', self.on_tf, qos)
        self.node.create_subscription(RosoutLog, '/rosout', self.on_rosout, 100)
        self.timer = self.node.create_timer(self.interval_sec, self.report)
        self.log_event('START', 'monitor started odom_topic=%s interval_sec=%.1f output_dir=%s' % (
            args.odom_topic, self.interval_sec, self.output_dir))

    def now_wall(self):
        return time.time()

    def ros_time(self):
        now = self.node.get_clock().now()
        return now.nanoseconds / 1.0e9

    def stamp_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1.0e9

    def mark(self, key):
        now = self.now_wall()
        self.counts[key] += 1
        self.last_seen_wall[key] = now
        return now

    def log_event(self, event, message):
        wall = datetime.now(timezone.utc).isoformat()
        self.events.write('%s event=%s %s\n' % (wall, event, message))

    def on_odom(self, msg):
        with self.lock:
            self.mark('odom')
            self.last_odom = (
                msg.header.frame_id, msg.child_frame_id,
                msg.pose.pose.position.x, msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                self.stamp_sec(msg.header.stamp),
            )
            if self.counts['odom'] == 1:
                self.log_event('FIRST_ODOM', 'frame=%s child=%s x=%.3f y=%.3f z=%.3f stamp=%.6f' % (
                    self.last_odom[0], self.last_odom[1], self.last_odom[2],
                    self.last_odom[3], self.last_odom[4], self.last_odom[5]))

    def on_tf(self, msg):
        with self.lock:
            self.mark('tf')
            for transform in msg.transforms:
                key = (transform.header.frame_id, transform.child_frame_id)
                if key in (('camera_init', 'body'), ('odom', 'camera_init')):
                    is_first_transform = key not in self.last_tf
                    t = transform.transform.translation
                    self.last_tf[key] = (self.now_wall(), t.x, t.y, t.z)
                    if key == ('camera_init', 'body') and is_first_transform:
                        self.log_event('FIRST_FASTLIO_TF', 'x=%.3f y=%.3f z=%.3f' % (
                            t.x, t.y, t.z))

    def on_rosout(self, msg):
        if msg.name not in ('fastlio_mapping', '/fastlio_mapping') and 'fast' not in msg.name.lower():
            return
        with self.lock:
            wall = datetime.now(timezone.utc).isoformat()
            text = msg.msg.replace('\n', '\\n')
            self.rosout.write('%s level=%s node=%s %s\n' % (wall, msg.level, msg.name, text))
            now = self.now_wall()
            if now - self.last_rosout_wall >= 0.25:
                self.last_rosout_wall = now
                lowered = msg.msg.lower()
                if any(token in lowered for token in ('odom', 'imu', 'cloud', 'path', 'publish', 'init')):
                    self.log_event('ROSOUT', 'level=%s node=%s %s' % (
                        msg.level, msg.name, msg.msg.replace('\n', ' ')))

    def age(self, key):
        last = self.last_seen_wall.get(key)
        return '' if last is None else '%.3f' % (self.now_wall() - last)

    def tf_age(self, key):
        value = self.last_tf.get(key)
        return '' if value is None else '%.3f' % (self.now_wall() - value[0])

    def report(self):
        with self.lock:
            now_wall = self.now_wall()
            ros_time = self.ros_time()
            elapsed = now_wall - self.started_wall
            odom_age = self.age('odom')
            camera_init_body_age = self.tf_age(('camera_init', 'body'))
            odom_frame_age = self.tf_age(('odom', 'camera_init'))

            if self.counts['odom'] == 0:
                state = 'WAIT_ODOM'
            elif ('camera_init', 'body') not in self.last_tf:
                state = 'WAIT_FASTLIO_TF'
            elif odom_age and float(odom_age) > max(3.0, self.interval_sec * 2.0):
                state = 'STALE_ODOM'
            else:
                state = 'ACTIVE'

            if state != self.last_state:
                self.log_event('STATE_CHANGE', '%s -> %s' % (self.last_state or 'START', state))
                self.last_state = state

            self.metrics_writer.writerow([
                datetime.now(timezone.utc).isoformat(), '%.6f' % ros_time,
                '%.3f' % elapsed, 'REPORT', self.counts['odom'], odom_age or '-',
                self.counts['tf'], camera_init_body_age or '-', odom_frame_age or '-',
                state,
            ])
            last_odom = '-' if self.last_odom is None else '%.3f,%.3f,%.3f' % self.last_odom[2:5]
            self.log_event('REPORT', 'state=%s odom=%d age=%s pose=%s tf=%d camera_init_body_age=%s odom_frame_age=%s' % (
                state, self.counts['odom'], odom_age or '-', last_odom,
                self.counts['tf'], camera_init_body_age or '-', odom_frame_age or '-'))

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.log_event('STOP', 'monitor stopped')
        self.events.close()
        self.rosout.close()
        self.metrics.close()
        self.node.destroy_node()


def main():
    args = parse_args()
    rclpy.init()
    diagnostics = FastlioDiagnostics(args)

    def shutdown_handler(signum, frame):  # noqa: ARG001
        diagnostics.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    try:
        while rclpy.ok():
            rclpy.spin_once(diagnostics.node, timeout_sec=0.2)
    finally:
        diagnostics.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
