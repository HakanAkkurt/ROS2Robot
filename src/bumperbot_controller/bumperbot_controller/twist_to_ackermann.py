#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToAckermann(Node):
    def __init__(self):

        super().__init__('twist_to_ackermann')

        self.pub = self.create_publisher(TwistStamped, '/ackermann_controller/reference', 10)
        
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb_twist, 10)

    def cb_twist(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist = msg
        self.pub.publish(ts)

def main():
    rclpy.init()
    node = TwistToAckermann()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
