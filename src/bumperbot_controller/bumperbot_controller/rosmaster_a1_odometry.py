#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class RosmasterA1Odometry(Node):
    def __init__(self):
        super().__init__('rosmaster_a1_odometry')

        # --- Namespace-aware frame IDs ---
        ns = self.get_namespace().strip("/")

        self.odom_frame = f"{ns}/odom"
        self.base_frame = f"{ns}/base_footprint"

        # 1 encoder count = 0.1 mm
        self.meters_per_count = 0.0001
        self.wheel_base = 0.2 # Distance between front and rear axle

        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.l_prev, self.r_prev = None, None
        self.prev_time = self.get_clock().now()
        self.vx, self.wz = 0.0, 0.0

        self.sub_joint = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Odometry started | {self.odom_frame} -> {self.base_frame}")

    def joint_callback(self, msg: JointState):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9

        if dt < 0.001 or dt > 1.0:
            self.prev_time = now
            return

        try:
            idx_l = msg.name.index("left_rear_wheel_joint")
            idx_r = msg.name.index("right_rear_wheel_joint")
            idx_s = msg.name.index("left_steering_hinge_joint")
            l_curr   = msg.position[idx_l]
            r_curr   = msg.position[idx_r]
            steer_rad = msg.position[idx_s]
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Joint names mismatch: {e}", throttle_duration_sec=5.0)
            return

        if self.l_prev is None:
            self.l_prev, self.r_prev = l_curr, r_curr
            self.prev_time = now
            return

        d_l  = (l_curr - self.l_prev) * self.meters_per_count
        d_r  = (r_curr - self.r_prev) * self.meters_per_count
        dist = (d_l + d_r) / 2.0

        steer_rad = max(min(steer_rad, 0.52), -0.52)
        if abs(steer_rad) < 0.001:
            d_theta = 0.0
        else:
            d_theta = (dist * math.tan(steer_rad)) / self.wheel_base

        self.vx = dist / dt
        self.wz = d_theta / dt

        avg_theta = self.theta + (d_theta / 2.0)
        self.x    += dist * math.cos(avg_theta)
        self.y    += dist * math.sin(avg_theta)
        self.theta = math.atan2(
            math.sin(self.theta + d_theta),
            math.cos(self.theta + d_theta)
        )

        self.l_prev, self.r_prev, self.prev_time = l_curr, r_curr, now
        self.publish_odom(now)

    def publish_odom(self, now):
        q = Quaternion()
        q.x, q.y = 0.0, 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        o = Odometry()
        o.header.stamp    = now.to_msg()
        o.header.frame_id = self.odom_frame
        o.child_frame_id  = self.base_frame
        o.pose.pose.position.x  = self.x
        o.pose.pose.position.y  = self.y
        o.pose.pose.orientation = q
        o.twist.twist.linear.x  = self.vx
        o.twist.twist.angular.z = self.wz
        self.pub_odom.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = RosmasterA1Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()