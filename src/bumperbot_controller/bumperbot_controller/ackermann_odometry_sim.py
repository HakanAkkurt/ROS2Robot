#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
from rclpy.time import Time

class AckermannOdomSim(Node):
    def __init__(self):
        super().__init__("ackermann_odometry_sim")

        # --- Parameters ---
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.033).value
        self.wheel_base = self.declare_parameter("wheel_base", 0.2).value

        # --- Robot State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left = None
        self.prev_right = None
        self.prev_time = None

        # --- ROS 2 Interfaces ---
        self.sub_joint = self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Ackermann Odometry Sim started with RK2 integration.")

    def joint_cb(self, msg: JointState):
        # Map joint names to their current positions
        joint_map = dict(zip(msg.name, msg.position))
        # Always use simulation time from the message header
        now = Time.from_msg(msg.header.stamp)

        try:
            steer_rad = joint_map["left_steering_hinge_joint"]
            
            pos_left = joint_map["left_rear_wheel_joint"]
            pos_right = joint_map["right_rear_wheel_joint"]
            
        except KeyError:
            return

        # Initialize on first callback
        if self.prev_left is None:
            self.prev_left, self.prev_right = pos_left, pos_right
            self.prev_time = now
            return

        # 1. Calculate Time Delta (dt)
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # 2. Calculate Distance Delta (Radiant to Meters)
        d_l = (pos_left - self.prev_left) * self.wheel_radius
        d_r = (pos_right - self.prev_right) * self.wheel_radius
        dist = (d_l + d_r) / 2.0

        # 3. Calculate Heading Delta (Ackermann Kinematics)
        # Limit steer_rad to avoid tan() issues with noise
        steer_rad = max(min(steer_rad, 0.55), -0.55)
        if abs(steer_rad) < 0.001:
            d_theta = 0.0
        else:
            d_theta = (dist * math.tan(steer_rad)) / self.wheel_base

        # 4. Pose Integration using Runge-Kutta 2 (Midpoint Method)
        # This significantly reduces drift during turns compared to Euler
        avg_theta = self.theta + (d_theta / 2.0)
        self.x += dist * math.cos(avg_theta)
        self.y += dist * math.sin(avg_theta)
        self.theta += d_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate Velocities for the Odometry message
        vx = dist / dt
        wz = d_theta / dt

        # Update previous state
        self.prev_left, self.prev_right, self.prev_time = pos_left, pos_right, now

        # 5. Publish TF and Odometry
        self.publish_data(now, vx, wz)

    def publish_data(self, stamp, vx, wz):
        stamp_msg = stamp.to_msg()
        
        # Create Quaternion from heading
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # Broadcast Transform (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = stamp_msg
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdomSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()