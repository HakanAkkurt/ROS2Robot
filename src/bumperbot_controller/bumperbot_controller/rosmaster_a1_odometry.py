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
        
        # 1 Encoder Count = 0.1 mm
        self.meters_per_count = 0.0001  
        self.wheel_base = 0.2  # Distance between front and rear axle
        
        # Robot Pose
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        
        # Previous encoder values for delta calculation
        self.l_prev, self.r_prev = None, None
        self.prev_time = self.get_clock().now()
        
        # Velocities
        self.vx = 0.0
        self.wz = 0.0
        
        # ROS 2 Communication
        self.sub_joint = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10
        )
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_callback(self, msg: JointState):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        
        # Check time plausibility
        if dt < 0.001 or dt > 1.0:
            self.prev_time = now
            return
        
        try:
            # Match joint names from Hardware Node
            idx_l = msg.name.index("left_rear_wheel_joint")
            idx_r = msg.name.index("right_rear_wheel_joint")
            idx_s = msg.name.index("left_steering_hinge_joint")
            
            l_curr = msg.position[idx_l]
            r_curr = msg.position[idx_r]
            steer_rad = msg.position[idx_s]
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Joint names mismatch: {e}", throttle_duration_sec=5.0)
            return
        
        # Initialize previous values on first run
        if self.l_prev is None:
            self.l_prev, self.r_prev = l_curr, r_curr
            self.prev_time = now
            return

        # ========================================
        # 1. DELTA CALCULATION
        # ========================================
        d_l = (l_curr - self.l_prev) * self.meters_per_count
        d_r = (r_curr - self.r_prev) * self.meters_per_count
        
        # Average distance traveled by the rear axle
        dist = (d_l + d_r) / 2.0
        
        # ========================================
        # 2. ACKERMANN KINEMATICS
        # ========================================
        # Limit steering angle to prevent tan(90) explosion and noise
        steer_rad = max(min(steer_rad, 0.52), -0.52)

        if abs(steer_rad) < 0.001:
            # Driving straight
            d_theta = 0.0
        else:
            # Standard Ackermann Odometry formula:
            d_theta = (dist * math.tan(steer_rad)) / self.wheel_base

        # ========================================
        # 3. VELOCITIES
        # ========================================
        self.vx = dist / dt
        self.wz = d_theta / dt
        
        # ========================================
        # 4. POSE INTEGRATION (Runge-Kutta 2)
        # ========================================
        # We use the average angle during the movement for better precision
        avg_theta = self.theta + (d_theta / 2.0)
        
        self.x += dist * math.cos(avg_theta)
        self.y += dist * math.sin(avg_theta)
        self.theta += d_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # ========================================
        # 5. STORAGE & PUBLISHING
        # ========================================
        self.l_prev = l_curr
        self.r_prev = r_curr
        self.prev_time = now
        
        self.publish_odom(now)

    def publish_odom(self, now):
        # Create Quaternion from Yaw
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        
        # TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        
        # Odometry Message
        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_footprint"
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = q
        o.twist.twist.linear.x = self.vx
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