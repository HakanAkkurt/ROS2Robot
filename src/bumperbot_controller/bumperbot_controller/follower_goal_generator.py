#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class FollowerGoalGenerator(Node):
    def __init__(self):
        super().__init__('follower_goal_generator')

        # --- Parameters ---
        self.declare_parameter('leader_namespace',  'leader')
        self.declare_parameter('follower_namespace', 'follower')
        self.declare_parameter('follow_distance', 0.5)

        self.leader_ns   = self.get_parameter('leader_namespace').get_parameter_value().string_value
        self.follower_ns = self.get_parameter('follower_namespace').get_parameter_value().string_value
        self.distance    = self.get_parameter('follow_distance').get_parameter_value().double_value

        # Follower navigates in leader's map frame (shared map concept)
        self.shared_map_frame = f'{self.leader_ns}/map'

        odom_topic = f'/{self.leader_ns}/odom'
        goal_topic = f'/{self.follower_ns}/goal_pose'

        # --- ROS 2 Interfaces ---
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.listener_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped, goal_topic, 10)

        self.get_logger().info('FollowerGoalGenerator started')
        self.get_logger().info(f'  Subscribing to:  {odom_topic}')
        self.get_logger().info(f'  Publishing to:   {goal_topic}')
        self.get_logger().info(f'  Shared map frame: {self.shared_map_frame}')
        self.get_logger().info(f'  Follow distance: {self.distance}m')

    def listener_callback(self, msg: Odometry):
        x        = msg.pose.pose.position.x
        y        = msg.pose.pose.position.y
        z_orient = msg.pose.pose.orientation.z
        w_orient = msg.pose.pose.orientation.w

        # Calculate yaw from quaternion
        yaw = 2.0 * math.atan2(z_orient, w_orient)

        # Build goal directly in leader/map frame
        # Both robots share this frame via /tf, no transformation needed
        goal_msg = PoseStamped()
        goal_msg.header.stamp    = msg.header.stamp
        goal_msg.header.frame_id = self.shared_map_frame
        goal_msg.pose.position.x  = x - (self.distance * math.cos(yaw))
        goal_msg.pose.position.y  = y - (self.distance * math.sin(yaw))
        goal_msg.pose.position.z  = 0.0
        goal_msg.pose.orientation = msg.pose.pose.orientation

        self.publisher_.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerGoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()