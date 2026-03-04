#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from hailo_platform import (VDevice, HEF, ConfigureParams, InputVStreamParams, 
                            OutputVStreamParams, HailoStreamInterface, InferVStreams)

class HailoYoloNode(Node):
    def __init__(self):
        super().__init__('hailo_yolo_node')
        self.bridge = CvBridge()
        
        # COCO Class Labels
        self.labels = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'Kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
            'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
            'hair drier', 'toothbrush'
        ]

        # 1. Hailo Hardware Setup
        self.model_path = "/ros2_ws/models/yolov8s_h8l.hef"
        self.target = VDevice()
        self.hef = HEF(self.model_path)
        
        # Configure with PCIe interface for Raspberry Pi 5
        configure_params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.target.configure(self.hef, configure_params)[0]
        self.network_group_params = self.network_group.create_params()
        
        self.input_vstreams_params = InputVStreamParams.make_from_network_group(self.network_group)
        self.output_vstreams_params = OutputVStreamParams.make_from_network_group(self.network_group)
        
        self.input_info = self.hef.get_input_vstream_infos()[0]
        self.model_h, self.model_w, _ = self.input_info.shape

        # Enter persistent contexts for performance
        self.activation_context = self.network_group.activate(self.network_group_params)
        self.activation_context.__enter__()
        self.pipeline = InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params)
        self.pipeline_context = self.pipeline.__enter__()

        # 2. ROS Setup
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/camera/yolo_detections', 10)

        self.frame_count = 0
        self.process_every_n_frames = 2
        
        self.get_logger().info("### Hailo YOLO Node started.###")

    def image_callback(self, msg):

        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
    
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            orig_h, orig_w = frame.shape[:2]
            
            # Pre-Processing: Resize and Convert to RGB for Hailo
            resized = cv2.resize(frame, (self.model_w, self.model_h))
            input_data = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            # Inference
            input_dict = {self.input_info.name: np.expand_dims(input_data, axis=0)}
            raw_results = self.pipeline_context.infer(input_dict)

            # Access the post-process output key
            output_data = raw_results.get('yolov8s/yolov8_nms_postprocess')
            if output_data is None:
                return

            # Parse inhomogeneous list structure: (1, 80) classes
            for class_idx, class_results in enumerate(output_data[0]): 
                if len(class_results) == 0:
                    continue

                for det in class_results:
                    # Expected format: [ymin, xmin, ymax, xmax, confidence, (optional) class_id]
                    if len(det) < 5:
                        continue
                    
                    confidence = float(det[4])
                    if confidence > 0.45:
                        ymin, xmin, ymax, xmax = det[0:4]
                        
                        # Determine label name
                        label_name = "Unknown"
                        if len(det) > 5:
                            label_name = self.labels[int(det[5])] if int(det[5]) < len(self.labels) else f"ID:{int(det[5])}"
                        else:
                            # Fallback to loop index if class_id is not in the sub-array
                            label_name = self.labels[class_idx] if class_idx < len(self.labels) else f"ID:{class_idx}"

                        # Scale normalized coordinates to pixel values
                        start_point = (int(xmin * orig_w), int(ymin * orig_h))
                        end_point = (int(xmax * orig_w), int(ymax * orig_h))
                        
                        # Drawing on frame
                        cv2.rectangle(frame, start_point, end_point, (0, 255, 0), 2)
                        
                        display_text = f"{label_name} {confidence:.2f}"
                        
                        cv2.putText(frame, display_text, 
                                    (start_point[0], max(25, start_point[1]-10)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Publish annotated image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f"Callback error: {str(e)}")

    def destroy_node(self):
        try:
            self.pipeline_context.__exit__(None, None, None)
            self.activation_context.__exit__(None, None, None)
        except:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = HailoYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()