#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class FollowerGoalGenerator(Node):
    def __init__(self):
        super().__init__('follower_goal_generator')

        self.subscription = self.create_subscription(
            Odometry,
            '/leader/odom',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(PoseStamped, '/leader/goal_pose', 10)
        self.get_logger().info('Goal Generator with 0.5m Offset started!')

    def listener_callback(self, msg):
        new_goal = PoseStamped()
        new_goal.header = msg.header
        new_goal.header.frame_id = 'odom'

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z_orient = msg.pose.pose.orientation.z
        w_orient = msg.pose.pose.orientation.w

        yaw = 2.0 * math.atan2(z_orient, w_orient)

        distance = 0.5 
        new_goal.pose.position.x = x - (distance * math.cos(yaw))
        new_goal.pose.position.y = y - (distance * math.sin(yaw))
        new_goal.pose.position.z = 0.0
        
        new_goal.pose.orientation = msg.pose.pose.orientation

        self.publisher_.publish(new_goal)

def main(args=None):
    rclpy.init(args=args)
    follower_goal_generator = FollowerGoalGenerator()
    rclpy.spin(follower_goal_generator)
    follower_goal_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()