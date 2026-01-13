#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import math

class FollowMeNode(Node):
    def __init__(self):
        super().__init__('follow_me_node')
        
        # Publisher to the Nav2 goal topic
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribe to your YOLO detections
        self.subscription = self.create_subscription(
            String, '/camera/yolo_detections', self.listener_callback, 10)

        # Settings
        self.image_width = 640.0
        # Estimated distance constant (needs tuning based on camera)
        self.dist_constant = 150000.0 
        self.last_goal_time = self.get_clock().now()

        self.get_logger().info('Nav2 Follow Node started!')

    def listener_callback(self, msg):
        detections = json.loads(msg.data)
        
        for det in detections:
            if det['class'] == 'person':
                x_center, y_center, w, h = det['bbox']
                area = w * h
                
                # 1. Estimate Distance (Z) based on bounding box area
                # Distance is inversely proportional to the area
                distance = 1.5 * (40000.0 / area) 

                # 2. Calculate horizontal angle (Yaw)
                # Angle offset from camera center
                angle_offset = ( (self.image_width / 2) - x_center ) * 0.001 

                # 3. Create Pose in robot coordinate system (base_link)
                # We want the robot to stop 0.8m in front of you
                target_dist = max(0.0, distance - 0.8)
                
                # Limit goal updates to every 1 second to not overwhelm Nav2
                now = self.get_clock().now()
                if (now - self.last_goal_time).nanoseconds > 1e9:
                    self.send_nav2_goal(target_dist, angle_offset)
                    self.last_goal_time = now
                break

    def send_nav2_goal(self, dist, angle):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "base_link" # Goal is relative to the robot

        # Target X/Y relative to robot
        goal.pose.position.x = dist * math.cos(angle)
        goal.pose.position.y = dist * math.sin(angle)
        
        # We don't care about rotation of the person, just face them
        goal.pose.orientation.w = 1.0 

        self.get_logger().info(f'Sending Nav2 Goal: {dist:.2f}m ahead')
        self.goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Follow Me Node stopping...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()