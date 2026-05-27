#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.25)
        
        camera_topic = self.get_parameter('camera_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.detection_pub = self.create_publisher(
            Image, detection_topic, 10)
        
        self.get_logger().info(f'Detector Node initialized: subscribing to {camera_topic}')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        detections = self.run_inference(cv_image)
        
        vis_image = self.draw_detections(cv_image, detections)
        
        detection_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
        detection_msg.header = msg.header
        self.detection_pub.publish(detection_msg)
    
    def run_inference(self, image):
        detections = []
        h, w = image.shape[:2]
        
        if np.random.random() > 0.7:
            x = np.random.randint(100, w - 100)
            y = np.random.randint(100, h - 100)
            box_w = np.random.randint(50, 150)
            box_h = np.random.randint(50, 150)
            confidence = np.random.uniform(0.6, 0.95)
            detections.append({
                'bbox': [x, y, box_w, box_h],
                'confidence': confidence,
                'class': 'armor'
            })
        
        return detections
    
    def draw_detections(self, image, detections):
        for det in detections:
            x, y, w, h = det['bbox']
            conf = det['confidence']
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, f'{det["class"]}: {conf:.2f}', 
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
