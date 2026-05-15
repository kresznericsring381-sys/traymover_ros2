#!/usr/bin/env python3
"""Bridge CMU pathFollower's /cmd_vel (TwistStamped) to base /cmd_vel
(Twist), gated on having received a recent /way_point.

Why the gate: CMU localPlanner runs with autonomyMode=true + a default
goalX/goalY of (0, 0). As soon as the robot's FAST_LIO pose drifts off
origin (IMU init noise, any motion at all), the planner treats (0, 0) as
an implicit goal and emits a /path. pathFollower then drives at
autonomySpeed *without any user command*. CMU's reference setup relies
on a joystick enable button to prevent that; we don't have one wired.

This node therefore holds /cmd_vel at zero until the first /way_point
has been published, and re-gates to zero if no /way_point
has been seen for `waypoint_timeout` seconds (so if the user stops
sending goals, the robot doesn't keep drifting toward a stale target).
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, PointStamped
from rclpy.node import Node


ZERO_TWIST = Twist()


class TwistStampedToTwist(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')
        self.declare_parameter('in_topic', '/cmd_vel_stamped')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('waypoint_topic', '/way_point')
        # Seconds since last /way_point before re-gating to zero. Set to
        # 0 or negative to disable re-gating (still requires first goal).
        self.declare_parameter('waypoint_timeout', 0.0)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        wp_topic = self.get_parameter('waypoint_topic').value
        self.timeout = float(self.get_parameter('waypoint_timeout').value)

        self.last_waypoint_time = None
        self.goal_received = False

        self.create_subscription(
            PointStamped, wp_topic, self.on_waypoint, 10)
        self.create_subscription(
            TwistStamped, in_topic, self.on_twist_stamped, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)

        self.get_logger().info(
            f"bridging {in_topic} (TwistStamped) -> {out_topic} (Twist); "
            f"gated on {wp_topic}, timeout={self.timeout}s")

    def on_waypoint(self, msg: PointStamped):
        self.goal_received = True
        self.last_waypoint_time = self.get_clock().now()
        self.get_logger().info(
            f"waypoint received ({msg.point.x:.2f}, {msg.point.y:.2f}) "
            f"in {msg.header.frame_id}; opening cmd_vel gate")

    def on_twist_stamped(self, msg: TwistStamped):
        if not self.goal_received:
            self.pub.publish(ZERO_TWIST)
            return
        if self.timeout > 0.0 and self.last_waypoint_time is not None:
            dt = (self.get_clock().now() - self.last_waypoint_time).nanoseconds * 1e-9
            if dt > self.timeout:
                self.pub.publish(ZERO_TWIST)
                return
        self.pub.publish(msg.twist)


def main():
    rclpy.init()
    node = TwistStampedToTwist()
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
