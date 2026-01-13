#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

class RosmasterJoystickTeleop(Node):
    def __init__(self):
        super().__init__("rosmaster_a1_joystick_teleop")

        # --- Publishers ---
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel_joy", 10)
        self.pub_servo_s1 = self.create_publisher(Float32, "/servo_s1", 10)
        self.pub_servo_s2 = self.create_publisher(Float32, "/servo_s2", 10)
        self.pub_servo_s3 = self.create_publisher(Float32, "/servo_s3", 10)
        self.pub_buzzer = self.create_publisher(Bool, "/Buzzer", 10)

        # --- Subscriber ---
        self.sub_joy = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # --- Config: Steering (S1) ---
        self.servo_s1_center = 90.0
        self.servo_s1_min = 50.0 # 60.0
        self.servo_s1_max = 130.0 # 120.0
        self.current_steering = self.servo_s1_center
        
        # --- Config: Camera (S2/S3) ---
        self.servo_s2_center = 90.0
        self.servo_s3_center = 45.0
        self.servo_min = 10.0
        self.servo_max = 170.0
        self.cam_step = 4.0
        self.current_pan = self.servo_s2_center
        self.current_tilt = self.servo_s3_center

        # --- Speed Settings ---
        self.max_linear_speed = 0.5
        self.axis_linear = 1
        self.axis_steering = 2
        self.deadzone = 0.1

        self.was_moving = False
        
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 1.0)

        # Auto reset servos
        self.reset_timer = self.create_timer(0.5, self.perform_auto_reset)


    def perform_auto_reset(self):
        """Publishes center positions once at startup and destroys the timer"""
        if self.pub_servo_s1.get_subscription_count() > 0:
            self.pub_servo_s1.publish(Float32(data=float(self.servo_s1_center)))
            self.pub_servo_s2.publish(Float32(data=float(self.servo_s2_center)))
            self.pub_servo_s3.publish(Float32(data=float(self.servo_s3_center)))

            self.reset_timer.cancel()


    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return value

    def joy_callback(self, msg: Joy):

        max_v = self.get_parameter("max_linear_vel").value
        max_w = self.get_parameter("max_angular_vel").value

        vx = msg.axes[self.axis_linear] * max_v
        vth = msg.axes[self.axis_steering] * max_w

        # Invert steering sign when reversing
        if vx < -0.01:
            vth = vth * -1.0

        # Deadzone for Joystick
        is_moving = abs(vx) > 0.02 or abs(vth) > 0.02

        twist = Twist()
        if is_moving:

            twist.linear.x = vx
            twist.angular.z = vth
            self.pub_cmd_vel.publish(twist)

            self.was_moving = True
        else:

            if self.was_moving:
                # No movement, send stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_cmd_vel.publish(twist)
                self.was_moving = False
            else:
                pass

        
        # 3. CAMERA CONTROL (D-Pad)
        arrow_h = msg.axes[6] if len(msg.axes) > 6 else 0.0
        arrow_v = msg.axes[7] if len(msg.axes) > 7 else 0.0

        if abs(arrow_h) > 0.1:
            self.current_pan += (arrow_h * self.cam_step)
            self.current_pan = max(self.servo_min, min(self.servo_max, self.current_pan))
            self.pub_servo_s2.publish(Float32(data=float(self.current_pan)))
            
        if abs(arrow_v) > 0.1:
            self.current_tilt += (arrow_v * self.cam_step)
            self.current_tilt = max(self.servo_min, min(self.servo_max, self.current_tilt))
            self.pub_servo_s3.publish(Float32(data=float(self.current_tilt)))

        # 4. BUTTONS
        if len(msg.buttons) > 0:
            self.pub_buzzer.publish(Bool(data=bool(msg.buttons[0])))

        # Manual Reset (Button 7)
        if len(msg.buttons) > 7 and msg.buttons[7] == 1:
            self.current_steering = self.servo_s1_center
            self.current_pan = self.servo_s2_center
            self.current_tilt = self.servo_s3_center
            
            self.pub_servo_s1.publish(Float32(data=float(self.current_steering)))
            self.pub_servo_s2.publish(Float32(data=float(self.current_pan)))
            self.pub_servo_s3.publish(Float32(data=float(self.current_tilt)))

def main(args=None):
    rclpy.init(args=args)
    node = RosmasterJoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()