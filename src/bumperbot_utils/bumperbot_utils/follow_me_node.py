#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
import json
import math

class FollowMeNode(Node):
    def __init__(self):
        super().__init__('follow_me_node')
        
        # --- Publishers ---
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.servo_pan_pub = self.create_publisher(Float32, '/servo_s2', 10)
        self.servo_tilt_pub = self.create_publisher(Float32, '/servo_s3', 10)
        
        # --- Subscribers ---
        # Subscribing to YOLO detections sent as a JSON string
        self.subscription = self.create_subscription(
            String, '/camera/yolo_detections', self.listener_callback, 10)

        # --- Settings & Calibration ---
        self.img_w = 640.0
        self.img_h = 480.0
        self.center_x = self.img_w / 2.0
        self.center_y = self.img_h / 2.0
        
        # Current Servo states
        self.current_pan = 90.0
        self.current_tilt = 45.0
        
        # P-Gain for visual servoing (speed of camera tracking)
        self.pan_gain = 0.075
        self.tilt_gain = 0.075

        self.last_goal_time = self.get_clock().now()

        self.center_servos()
        
        self.get_logger().info('Enhanced FollowMe Node (with Camera Tracking) started!')

    def center_servos(self):
        """Sends initial center positions to the servos"""
        self.servo_pan_pub.publish(Float32(data=float(self.current_pan)))
        self.servo_tilt_pub.publish(Float32(data=float(self.current_tilt)))
        self.get_logger().info(f'Servos initialized to Pan:{self.current_pan} Tilt:{self.current_tilt}')

    def listener_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        
        for det in detections:
            if det['class'] == 'cell phone':
                x_c, y_c, w, h = det['bbox']
                area = w * h

                # --- SETTINGS ---
                deadzone_x = 25.0
                deadzone_y = 15.0  
                smoothing_factor = 0.3
                
                # CAMERA TRACKING (Servos)
                # These remain fully dynamic (tracking X and Y)
                error_x = self.center_x - x_c
                error_y = self.center_y - y_c 

                target_pan = self.current_pan
                target_tilt = self.current_tilt

                if abs(error_x) > deadzone_x:
                    target_pan += error_x * self.pan_gain
                if abs(error_y) > deadzone_y:
                    target_tilt += error_y * self.tilt_gain

                new_pan = (smoothing_factor * target_pan) + ((1.0 - smoothing_factor) * self.current_pan)
                new_tilt = (smoothing_factor * target_tilt) + ((1.0 - smoothing_factor) * self.current_tilt)

                if abs(new_pan - self.current_pan) > 0.5 or abs(new_tilt - self.current_tilt) > 0.5:
                    self.current_pan = max(10.0, min(170.0, new_pan))
                    self.current_tilt = max(10.0, min(170.0, new_tilt))
                    self.servo_pan_pub.publish(Float32(data=float(self.current_pan)))
                    self.servo_tilt_pub.publish(Float32(data=float(self.current_tilt)))

                # ROBOT MOVEMENT (Nav2) 
                # Calculating distance based on area (calibrated for cell phone)
                # For a 'person' object 40000
                raw_distance = 1.0 * (5000.0 / area) 
                
                # Safety clamp for small room, robot moves max 2.5m
                clamped_dist = max(0.0, min(2.5, raw_distance))
                
                # We want to maintain a 0.6m distance from the object
                # positive = move forward, negative = move backward
                target_offset_x = clamped_dist - 0.6

                now = self.get_clock().now()
                if (now - self.last_goal_time).nanoseconds > 6e8:
                    # Send a goal if the required movement is more than 10cm
                    if abs(target_offset_x) > 0.1:
                        self.send_nav2_goal_linear(target_offset_x)
                        self.last_goal_time = now
                break

    def send_nav2_goal_linear(self, x_dist):
        """Sends a PoseStamped goal strictly on the robot's X-axis"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "base_link" 

        # x_dist > 0: Goal is in front
        # x_dist < 0: Goal is behind
        goal.pose.position.x = float(x_dist)
        goal.pose.position.y = 0.0 # Strict linear tracking
        goal.pose.position.z = 0.0
        
        # Keep neutral orientation relative to base_link
        goal.pose.orientation.w = 1.0 

        direction = "FORWARD" if x_dist > 0 else "BACKWARD"
        self.get_logger().info(f'Nav2 Linear Goal: {direction} {abs(x_dist):.2f}m')
        self.goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()