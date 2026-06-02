#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import cv2
import numpy as np
from cv_bridge import CvBridge

from hailo_platform import (VDevice, HEF, ConfigureParams, InputVStreamParams, 
                            OutputVStreamParams, HailoStreamInterface, InferVStreams)

class HailoYoloNode(Node):
    def __init__(self):
        super().__init__('hailo_yolo_node')
        self.bridge = CvBridge()
        
        # COCO Labels
        self.labels = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
                       'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                       'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                       'skis', 'snowboard', 'sports ball', 'Kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
                       'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                       'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                       'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                       'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
                       'hair drier', 'toothbrush']

        # 1. Hailo Hardware Setup
        self.model_path = "/ros2_ws/models/yolov8s.hef"
        self.target = VDevice()
        self.hef = HEF(self.model_path)
        
        # Configure for PCIe interface (Raspberry Pi 5)
        configure_params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.target.configure(self.hef, configure_params)[0]
        self.network_group_params = self.network_group.create_params()
        
        self.input_info = self.hef.get_input_vstream_infos()[0]
        self.model_h, self.model_w, _ = self.input_info.shape

        # Activate the model context
        self.activation_context = self.network_group.activate(self.network_group_params)
        self.activation_context.__enter__()
        self.pipeline = InferVStreams(self.network_group, 
                                     InputVStreamParams.make_from_network_group(self.network_group), 
                                     OutputVStreamParams.make_from_network_group(self.network_group))
        self.pipeline_context = self.pipeline.__enter__()

        # 2. ROS Setup (Standard queue size 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        
        # Publisher for STANDARD Raw Image (Easy to see in RViz)
        self.image_pub = self.create_publisher(Image, 'camera/yolo_result', 10)
        
        # Publisher for detection metadata (JSON string for FollowMe node)
        self.data_pub = self.create_publisher(String, 'camera/yolo_detections', 10)

        # self.frame_count = 0
        # self.process_every_n_frames = 2

        self.get_logger().info("### Hailo YOLO Node started! ###")

    def image_callback(self, msg):

        # self.frame_count += 1
        # if self.frame_count % self.process_every_n_frames != 0:
        #     return
        
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            orig_h, orig_w = frame.shape[:2]
            
            # Hailo Pre-Processing: Resize and Convert to RGB
            resized = cv2.resize(frame, (self.model_w, self.model_h))
            input_data = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            # Inference execution
            raw_results = self.pipeline_context.infer({self.input_info.name: np.expand_dims(input_data, axis=0)})
            output_data = raw_results.get('yolov8s/yolov8_nms_postprocess')

            detections_list = []

            if output_data is not None:
                for class_idx, class_results in enumerate(output_data[0]):
                    for det in class_results:
                        if len(det) < 5: continue
                        confidence = float(det[4])
                        
                        if confidence > 0.45:
                            # Hailo coordinate format: [ymin, xmin, ymax, xmax]
                            ymin, xmin, ymax, xmax = det[0:4]
                            label = self.labels[class_idx] if class_idx < len(self.labels) else "unknown"

                            # Convert normalized coordinates to pixel values
                            x1, y1 = xmin * orig_w, ymin * orig_h
                            x2, y2 = xmax * orig_w, ymax * orig_h
                            
                            # Calculate Center-X, Center-Y, Width, Height for FollowMe logic
                            w_box = x2 - x1
                            h_box = y2 - y1
                            x_c = x1 + (w_box / 2.0)
                            y_c = y1 + (h_box / 2.0)

                            # Append data for JSON publishing
                            detections_list.append({
                                "class": label,
                                "conf": confidence,
                                "bbox": [float(x_c), float(y_c), float(w_box), float(h_box)]
                            })

                            # Draw bounding boxes for visualization
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(frame, f"{label} {confidence:.2f}", (int(x1), int(y1-10)), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 1. Publish JSON Detection Data for FollowMeNode
            data_msg = String()
            data_msg.data = json.dumps(detections_list)
            self.data_pub.publish(data_msg)

            small_frame = cv2.resize(frame, (320, 240))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(small_frame, encoding='bgr8'))
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

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