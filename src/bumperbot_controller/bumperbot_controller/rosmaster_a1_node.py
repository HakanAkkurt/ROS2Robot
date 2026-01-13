#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu, MagneticField, Image
from std_msgs.msg import Float32, Int32, Bool, Header
import math
from cv_bridge import CvBridge
import cv2
from Rosmaster_Lib import Rosmaster

class RosmasterA1Node(Node):
    def __init__(self):
        super().__init__('rosmaster_a1_node')
        self.car = Rosmaster()
        self.car.set_car_type(1)  # Ackermann Mode
        self.car.create_receive_threading()

        # --- Configuration Parameters ---
        self.wheelbase = 0.2          # Distance between front and rear axles (meters)
        self.steering_gain = 0.75     # Multiplier for URDF joint visualization
        self.servo_center_deg = 90.0  # Physical center position of the steering servo
        self.servo_min_deg = 50.0     # Left mechanical limit
        self.servo_max_deg = 130.0    # Right mechanical limit
        
        # --- Anti-Stutter & Smoothing ---
        self.steer_filter_alpha = 0.25 # Low-pass filter for servo (0.0 to 1.0)
        self.min_speed_steer = 0.05    # Min speed threshold for geometric steering
        self.last_vx = 0.0
        self.last_steer_sent = 90
        
        # --- Magnetometer Filter & Normalization ---
        self.alpha_mag = 0.1           # Mag low-pass filter
        self.mag_filt = [0.0, 0.0, 0.0]
        self.mag_conversion_factor = 1e-6 # Convert raw microTesla to Tesla (ROS2 standard)

        # --- Internal Steering State ---
        self.target_steer_deg = self.servo_center_deg
        self.current_steer_deg = self.servo_center_deg

        # --- Subscribers ---
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.create_subscription(Int32, "RGBLight", self.rgb_callback, 10)
        self.create_subscription(Bool, "Buzzer", self.buzzer_callback, 10)
        
        # Manual Servo overrides (e.g. from Joystick)
        self.create_subscription(Float32, "/servo_s1", self.s1_callback, 10)
        self.create_subscription(Float32, "/servo_s2", self.s2_callback, 10)
        self.create_subscription(Float32, "/servo_s3", self.s3_callback, 10)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0) # Open default camera
        # Set resolution to 640x480 for better performance on Pi
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # --- Publishers ---
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.pub_mag = self.create_publisher(MagneticField, "/imu/mag", 10)
        self.pub_voltage = self.create_publisher(Float32, "/voltage", 10)
        self.pub_version = self.create_publisher(Float32, "/edition", 10)
        self.pub_image = self.create_publisher(Image, "/camera/image_raw", 10)

        # --- Timers ---
        # 50Hz: Critical for smooth steering transitions
        self.create_timer(0.02, self.control_loop)       
        # 20Hz: Sensor data throughput
        self.create_timer(0.05, self.publish_sensor_data) 
        self.create_timer(0.05, self.publish_camera_data)

        self.get_logger().info("### Rosmaster A1 Node started! ###")

    def publish_camera_data(self):
        """Reads frame from USB camera and publishes it as ROS2 Image."""
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS2 message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"
            self.pub_image.publish(img_msg)

    def cmd_vel_callback(self, msg: Twist):
        """Processes incoming velocity commands from Nav2 or Multiplexer."""
        vx = float(msg.linear.x)
        vth = float(msg.angular.z)

        # 1. MOTOR CONTROL
        # Update motor speed if there is movement or we need to stop
        if abs(vx) > 0.001 or abs(self.last_vx) > 0.001:
            self.car.set_car_motion(vx, 0.0, 0.0)
            self.last_vx = vx

        # 2. STEERING CALCULATION (Ackermann Fix)
        if abs(vx) > 0.05:
            # Driving: Calculate angle based on wheelbase geometry
            angle_rad = math.atan(self.wheelbase * vth / vx)
            angle_deg = math.degrees(angle_rad)
        else:
            # Stopped/Slow: Map Angular Z directly to steering (for joystick feel)
            # Typically vth is -1.0 to 1.0, so we scale to ~45 degrees
            angle_deg = vth * 45.0 

        # 3. APPLY LIMITS
        # Calculate final servo position (90 is neutral)
        target = self.servo_center_deg - angle_deg
        self.target_steer_deg = max(self.servo_min_deg, min(self.servo_max_deg, target))

    def control_loop(self):
        """Maintains smooth hardware updates via low-pass filtering."""
        # Smoothing the steering to prevent servo jitter
        self.current_steer_deg = (self.steer_filter_alpha * self.target_steer_deg) + \
                                 ((1.0 - self.steer_filter_alpha) * self.current_steer_deg)
        
        val_to_send = int(self.current_steer_deg)
        
        # Send to hardware only if integer value changes to optimize serial bandwidth
        if val_to_send != self.last_steer_sent:
            self.car.set_pwm_servo(1, val_to_send)
            self.last_steer_sent = val_to_send

    def publish_sensor_data(self):
        """Gathers and publishes IMU, Mag, and Joint data."""
        now = self.get_clock().now().to_msg()

        # 1. IMU Data
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        
        imu_msg = Imu(header=Header(stamp=now, frame_id="imu_link"))
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        self.pub_imu.publish(imu_msg)

        # 2. Magnetometer Data (Filtered)
        mag_data = self.car.get_magnetometer_data()
        if mag_data and len(mag_data) == 3:
            mag_msg = MagneticField(header=Header(stamp=now, frame_id="imu_link"))
            for i in range(3):
                self.mag_filt[i] = (self.alpha_mag * float(mag_data[i])) + \
                                   ((1.0 - self.alpha_mag) * self.mag_filt[i])
            
            mag_msg.magnetic_field.x = self.mag_filt[0] * self.mag_conversion_factor
            mag_msg.magnetic_field.y = self.mag_filt[1] * self.mag_conversion_factor
            mag_msg.magnetic_field.z = self.mag_filt[2] * self.mag_conversion_factor
            self.pub_mag.publish(mag_msg)

        # 3. Joint States (Wheel and Steering visualization)
        enc = self.car.get_motor_encoder()
        if enc and len(enc) >= 4:
            js = JointState(header=Header(stamp=now))
            js.name = ["left_rear_wheel_joint", 
                       "right_rear_wheel_joint", 
                       "left_steering_hinge_joint", 
                       "right_steering_hinge_joint"]
            
            # Convert physical steering degrees to radians for URDF
            steer_rad = math.radians(self.servo_center_deg - self.current_steer_deg) * self.steering_gain
            js.position = [-float(enc[1]), -float(enc[3]), steer_rad, steer_rad]
            self.pub_joint.publish(js)

        # 4. System Status
        volts = self.car.get_battery_voltage()
        if volts: self.pub_voltage.publish(Float32(data=float(volts)))

    # --- Utility Callbacks ---
    def rgb_callback(self, msg: Int32): 
        # Sets LED strip to red (255,0,0) with brightness from msg
        self.car.set_colorful_lamps(int(msg.data), 255, 0, 0)

    def buzzer_callback(self, msg: Bool): 
        self.car.set_beep(100 if msg.data else 0)

    def s1_callback(self, msg: Float32): 
        self.target_steer_deg = float(msg.data)

    def s2_callback(self, msg: Float32): 
        self.car.set_pwm_servo(2, int(msg.data))

    def s3_callback(self, msg: Float32): 
        self.car.set_pwm_servo(3, int(msg.data))

    def __del__(self):
        """Clean up camera on exit."""
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = RosmasterA1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()