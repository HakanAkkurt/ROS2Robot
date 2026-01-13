#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import json
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class YoloNavigationMonitor(Node):
    def __init__(self):
        super().__init__('yolo_navigation_monitor')
        self.bridge = CvBridge()

        # Allows toggling the GUI window via ROS2 parameters or launch files
        self.declare_parameter('use_display', True)
        self.use_display = self.get_parameter('use_display').get_parameter_value().bool_value
        
        # Check if an X11 display is actually available (prevents crash on headless Pi)
        if self.use_display and "DISPLAY" not in os.environ:
            self.get_logger().warn('No Monitor detected (DISPLAY env missing). Disabling CV2 Window.')
            self.use_display = False
        
        # Get path to the weights file located in the package share directory
        package_share_dir = get_package_share_directory('bumperbot_controller')
        model_path = os.path.join(package_share_dir, 'yolo', 'yolov8n.pt')
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        
        # Input: Raw camera frames
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10)

        # Output: Detection metadata as JSON String (lightweight for network)
        self.data_pub = self.create_publisher(String, '/camera/yolo_detections', 10)
        
        # Output: Processed image with bounding boxes (for remote viewing)
        self.image_pub = self.create_publisher(Image, '/camera/yolo_result', 10)
        
        self.get_logger().info('YOLO Detection Node started. Use_display is set to: ' + str(self.use_display))

    def image_callback(self, msg):
        # 1. Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Run YOLO inference
        # stream=True uses a generator for better memory efficiency
        results = self.model(frame, stream=True, conf=0.5, verbose=False)

        detections = []
        annotated_frame = frame

        # 3. Process results
        for r in results:
            # Create a frame with visual bounding boxes
            annotated_frame = r.plot()
            
            for box in r.boxes:
                # Extract coordinates [x_center, y_center, width, height]
                b = box.xywh[0].tolist() 
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model.names[cls]

                # Append to list for JSON publishing
                detections.append({
                    "class": name,
                    "conf": conf,
                    "bbox": b
                })

        # 4. Publish detection data (JSON)
        # Even if empty, we publish an empty list to keep the subscriber updated
        data_msg = String()
        data_msg.data = json.dumps(detections)
        self.data_pub.publish(data_msg)

        # 5. Publish annotated image
        # Useful for RViz or rqt_image_view on a remote PC
        img_out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        img_out_msg.header = msg.header
        self.image_pub.publish(img_out_msg)

        # 6. Local Visualization
        if self.use_display:
            cv2.imshow("Robot View - YOLOv8", annotated_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNavigationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('YOLO Node stopped by user.')
    finally:
        # Cleanup
        if node.use_display:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()