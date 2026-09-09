#!/usr/bin/env python3
"""Persist high-signal diagnostics for the FAST-LIO + NDT localization chain."""

import argparse
import csv
import os
import signal
import threading
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Log as RosoutLog
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--cloud-topic', default='/point_cloud_localization')
    parser.add_argument('--interval-sec', type=float, default=5.0)
    return parser.parse_args()


class NdtDiagnostics:
    def __init__(self, args):
        self.node = rclpy.create_node(
            'ndt_diagnostics',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.output_dir = os.path.abspath(args.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
        self.interval_sec = max(1.0, args.interval_sec)
        self.lock = threading.Lock()
        self.started_wall = time.time()
        self.last_rosout_wall = 0.0
        self.lifecycle_state = 'unknown'
        self.lifecycle_future = None
        self.counts = {
            'cloud': 0,
            'odom': 0,
            'initialpose': 0,
            'pcl_pose': 0,
            'tf': 0,
        }
        self.last_seen_wall = {}
        self.last_ros_time = 0.0
        self.last_cloud_stamp = None
        self.last_cloud_points = None
        self.last_odom = None
        self.last_pcl_pose = None
        self.last_initialpose = None
        self.last_tf = {}
        self.last_state = None
        self._closed = False

        self.events = open(os.path.join(self.output_dir, 'ndt_events.log'), 'a', buffering=1)
        self.rosout = open(os.path.join(self.output_dir, 'ndt_rosout.log'), 'a', buffering=1)
        self.metrics = open(os.path.join(self.output_dir, 'ndt_metrics.csv'), 'a', newline='', buffering=1)
        self.metrics_writer = csv.writer(self.metrics)
        if os.path.getsize(os.path.join(self.output_dir, 'ndt_metrics.csv')) == 0:
            self.metrics_writer.writerow([
                'wall_time', 'ros_time', 'elapsed_sec', 'event',
                'cloud_count', 'cloud_age_sec', 'cloud_points',
                'odom_count', 'odom_age_sec', 'pcl_pose_count',
                'pcl_pose_age_sec', 'initialpose_count', 'tf_count',
                'map_odom_age_sec', 'odom_base_age_sec', 'lifecycle_state',
            ])

        qos = rclpy.qos.qos_profile_sensor_data
        self.node.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, qos)
        self.node.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.node.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/pcl_pose', self.on_pcl_pose, 10)
        self.node.create_subscription(TFMessage, '/tf', self.on_tf, qos)
        self.node.create_subscription(RosoutLog, '/rosout', self.on_rosout, 100)
        self.lifecycle_client = self.node.create_client(
            GetState, '/lidar_localization/get_state')
        self.timer = self.node.create_timer(self.interval_sec, self.report)
        self.log_event('START', 'monitor started cloud_topic=%s interval_sec=%.1f output_dir=%s' % (
            args.cloud_topic, self.interval_sec, self.output_dir))

    def now_wall(self):
        return time.time()

    def ros_time(self):
        now = self.node.get_clock().now()
        self.last_ros_time = now.nanoseconds / 1.0e9
        return self.last_ros_time

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

    def on_cloud(self, msg):
        with self.lock:
            self.mark('cloud')
            self.last_cloud_stamp = self.stamp_sec(msg.header.stamp)
            self.last_cloud_points = int(msg.width) * int(msg.height)
            if self.counts['cloud'] == 1:
                self.log_event('FIRST_CLOUD', 'frame=%s stamp=%.6f points=%d' % (
                    msg.header.frame_id, self.last_cloud_stamp, self.last_cloud_points))

    def on_odom(self, msg):
        with self.lock:
            self.mark('odom')
            self.last_odom = (
                msg.header.frame_id, msg.child_frame_id,
                msg.pose.pose.position.x, msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
            if self.counts['odom'] == 1:
                self.log_event('FIRST_ODOM', 'frame=%s child=%s x=%.3f y=%.3f z=%.3f' % (
                    self.last_odom[0], self.last_odom[1], self.last_odom[2],
                    self.last_odom[3], self.last_odom[4]))

    def on_initialpose(self, msg):
        with self.lock:
            self.mark('initialpose')
            pose = msg.pose.pose
            self.last_initialpose = (pose.position.x, pose.position.y, pose.position.z)
            self.log_event('INITIALPOSE', 'x=%.3f y=%.3f z=%.3f frame=%s' % (
                pose.position.x, pose.position.y, pose.position.z, msg.header.frame_id))

    def on_pcl_pose(self, msg):
        with self.lock:
            self.mark('pcl_pose')
            pose = msg.pose.pose
            self.last_pcl_pose = (pose.position.x, pose.position.y, pose.position.z)
            self.log_event('NDT_POSE', 'count=%d x=%.3f y=%.3f z=%.3f frame=%s' % (
                self.counts['pcl_pose'], pose.position.x, pose.position.y,
                pose.position.z, msg.header.frame_id))

    def on_tf(self, msg):
        with self.lock:
            self.mark('tf')
            for transform in msg.transforms:
                key = (transform.header.frame_id, transform.child_frame_id)
                if key in (('map', 'odom'), ('odom', 'base_link'), ('camera_init', 'body')):
                    is_first_transform = key not in self.last_tf
                    t = transform.transform.translation
                    self.last_tf[key] = (self.now_wall(), t.x, t.y, t.z)
                    if key == ('map', 'odom') and is_first_transform:
                        self.log_event('FIRST_MAP_ODOM', 'x=%.3f y=%.3f z=%.3f' % (
                            t.x, t.y, t.z))

    def on_rosout(self, msg):
        if msg.name not in ('lidar_localization', '/lidar_localization') and \
                'ndt' not in msg.name.lower() and 'localization' not in msg.name.lower():
            return
        with self.lock:
            wall = datetime.now(timezone.utc).isoformat()
            text = msg.msg.replace('\n', '\\n')
            self.rosout.write('%s level=%s node=%s %s\n' % (wall, msg.level, msg.name, text))
            now = self.now_wall()
            if now - self.last_rosout_wall >= 0.25:
                self.last_rosout_wall = now
                lowered = msg.msg.lower()
                if any(token in lowered for token in (
                        'ndt', 'fitness', 'converg', 'initial', 'jump', 'tf', 'cloud',
                        'map->odom', 'map to odom')):
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
            self.request_lifecycle_state()
            now_wall = self.now_wall()
            ros_time = self.ros_time()
            elapsed = now_wall - self.started_wall
            cloud_age = self.age('cloud')
            odom_age = self.age('odom')
            pcl_age = self.age('pcl_pose')
            map_odom_age = self.tf_age(('map', 'odom'))
            odom_base_age = self.tf_age(('odom', 'base_link'))
            self.metrics_writer.writerow([
                datetime.now(timezone.utc).isoformat(), '%.6f' % ros_time,
                '%.3f' % elapsed, 'REPORT', self.counts['cloud'], cloud_age,
                self.last_cloud_points if self.last_cloud_points is not None else '',
                self.counts['odom'], odom_age, self.counts['pcl_pose'], pcl_age,
                self.counts['initialpose'], self.counts['tf'], map_odom_age,
                odom_base_age, self.lifecycle_state,
            ])

            missing = []
            for key, label in (
                    ('cloud', 'cloud'), ('odom', 'odom'), ('tf', 'tf')):
                if key not in self.last_seen_wall:
                    missing.append(label)
            if self.counts['initialpose'] == 0:
                state = 'WAIT_INITIALPOSE'
            elif self.counts['pcl_pose'] == 0:
                state = 'WAIT_NDT_POSE'
            elif ('map', 'odom') not in self.last_tf:
                state = 'WAIT_MAP_ODOM_TF'
            elif missing:
                state = 'MISSING_' + '_'.join(missing).upper()
            else:
                state = 'ACTIVE'
            if state != self.last_state:
                self.log_event('STATE_CHANGE', '%s -> %s' % (self.last_state or 'START', state))
                self.last_state = state
            map_odom = self.last_tf.get(('map', 'odom'))
            map_odom_position = '-' if map_odom is None else '%.3f,%.3f,%.3f' % map_odom[1:]
            ndt_position = '-' if self.last_pcl_pose is None else '%.3f,%.3f,%.3f' % self.last_pcl_pose
            self.log_event('REPORT', 'state=%s cloud=%d age=%s points=%s odom=%d age=%s '
                           'initialpose=%d pcl_pose=%d age=%s ndt_pose=%s tf=%d map_odom_age=%s '
                           'map_odom=%s odom_base_age=%s lifecycle=%s' % (
                               state, self.counts['cloud'], cloud_age or '-',
                               self.last_cloud_points if self.last_cloud_points is not None else '-',
                               self.counts['odom'], odom_age or '-', self.counts['initialpose'],
                               self.counts['pcl_pose'], pcl_age or '-', ndt_position,
                               self.counts['tf'], map_odom_age or '-', map_odom_position,
                               odom_base_age or '-', self.lifecycle_state))

    def request_lifecycle_state(self):
        if self.lifecycle_future is not None:
            if not self.lifecycle_future.done():
                return
            try:
                self.lifecycle_state = self.lifecycle_future.result().current_state.label
            except Exception as exc:  # Service can disappear during shutdown.
                self.lifecycle_state = 'error:%s' % exc
            self.lifecycle_future = None
        if self.lifecycle_client.service_is_ready():
            self.lifecycle_future = self.lifecycle_client.call_async(GetState.Request())

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
    monitor = NdtDiagnostics(args)

    def stop_handler(signum, frame):
        monitor.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        rclpy.spin(monitor.node)
    finally:
        monitor.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
