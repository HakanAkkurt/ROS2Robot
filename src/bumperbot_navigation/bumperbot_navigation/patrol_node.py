#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import math
import random

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # --- Settings ---
        self.default_min_dist = 4.0        # Preferred long distance
        self.last_goal_x = 0.0
        self.last_goal_y = 0.0
        self.costmap = None
        self.goal_active = False
        self.initial_exploration_done = False 
        
        # ROS Interfaces
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Patrol Node started!")

    def costmap_callback(self, msg):
        self.costmap = msg

    def is_valid(self, x, y, required_dist, allow_unknown=False):
        """Check if point is far enough and safe."""
        if self.costmap is None: return False
        
        # 1. Check against the provided distance threshold
        if math.hypot(x - self.last_goal_x, y - self.last_goal_y) < required_dist:
            return False

        # 2. Map cost check
        info = self.costmap.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        
        if 0 <= col < info.width and 0 <= row < info.height:
            cost = self.costmap.data[row * info.width + col]
            if cost == 0: return True
            if allow_unknown and cost == -1: return True
        return False

    def timer_callback(self):
        if self.goal_active or self.costmap is None:
            return

        # Step 1: Try finding a goal with the preferred 4.0m distance
        found = self.search_logic(self.default_min_dist, allow_unknown=False)
        
        # Step 2: If no large white goal found, try smaller white goals (for small rooms)
        if not found:
            found = self.search_logic(required_dist=1.0, allow_unknown=False)

        # Step 3: If still nothing and at start, try unknown space
        if not found and not self.initial_exploration_done:
            self.get_logger().info("Searching in unknown space...")
            found = self.search_logic(required_dist=1.0, allow_unknown=True)

    def search_logic(self, required_dist, allow_unknown):
        """Standard search loop with a specific distance requirement."""
        info = self.costmap.info
        for _ in range(300):
            rx = random.uniform(info.origin.position.x, info.origin.position.x + info.width * info.resolution)
            ry = random.uniform(info.origin.position.y, info.origin.position.y + info.height * info.resolution)

            if self.is_valid(rx, ry, required_dist, allow_unknown):
                self.send_goal(rx, ry)
                return True
        return False

    def send_goal(self, x, y):
        self.goal_active = True
        self.last_goal_x, self.last_goal_y = x, y
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y = x, y
        goal_msg.pose.pose.orientation.w = 1.0 

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.response_cb)

    def response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_active = False
            return
        
        self.initial_exploration_done = True
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.goal_active = False
        self.get_logger().info("Goal reached.")

def main():
    rclpy.init()
    rclpy.spin(PatrolNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()