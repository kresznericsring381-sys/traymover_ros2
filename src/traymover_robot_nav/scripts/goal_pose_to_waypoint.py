#!/usr/bin/env python3
"""Bridge RViz '2D Goal Pose' (PoseStamped on /goal_pose) to CMU
local_planner's /way_point (PointStamped).

CMU local_planner subscribes /way_point and reads point.x, point.y
directly as world-frame coordinates, comparing them with the vehicle
pose taken from /state_estimation. Both must be in the same frame.

FAST_LIO /Odometry (remapped to /odom and fed to CMU as /state_estimation)
lives in `camera_init`, but RViz's 2D Goal Pose publishes in `map`. After
NDT correction, map != camera_init by a few cm. We therefore transform
the incoming goal into the target frame (default: camera_init) before
republishing, so CMU's internal math is self-consistent.
"""
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class GoalPoseToWaypoint(Node):
    def __init__(self):
        super().__init__('goal_pose_to_waypoint')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('waypoint_topic', '/way_point')
        self.declare_parameter('target_frame', 'camera_init')

        goal_topic = self.get_parameter('goal_topic').value
        wp_topic = self.get_parameter('waypoint_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PoseStamped, goal_topic, self.on_goal, 10)
        self.pub = self.create_publisher(PointStamped, wp_topic, 10)
        self.get_logger().info(
            f"bridging {goal_topic} (PoseStamped) -> {wp_topic} "
            f"(PointStamped, frame={self.target_frame})")

    def on_goal(self, msg: PoseStamped):
        src_pt = PointStamped()
        src_pt.header = msg.header
        src_pt.point.x = msg.pose.position.x
        src_pt.point.y = msg.pose.position.y
        src_pt.point.z = msg.pose.position.z

        if msg.header.frame_id == self.target_frame:
            out = src_pt
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id,
                    rclpy.time.Time(), timeout=Duration(seconds=0.5))
            except TransformException as exc:
                self.get_logger().warn(
                    f"TF {msg.header.frame_id}->{self.target_frame} "
                    f"unavailable: {exc}")
                return
            out = do_transform_point(src_pt, tf)
            out.header.frame_id = self.target_frame

        self.pub.publish(out)
        self.get_logger().info(
            f"forwarded goal ({out.point.x:.2f}, {out.point.y:.2f}) "
            f"in {out.header.frame_id}")


def main():
    rclpy.init()
    node = GoalPoseToWaypoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
