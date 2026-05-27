#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('track_topic', '/tracked_targets')
        self.declare_parameter('track_buffer', 30)
        
        detection_topic = self.get_parameter('detection_topic').value
        track_topic = self.get_parameter('track_topic').value
        self.track_buffer = self.get_parameter('track_buffer').value
        
        self.bridge = CvBridge()
        
        self.detection_sub = self.create_subscription(
            Image, detection_topic, self.detection_callback, 10)
        self.track_pub = self.create_publisher(
            Point, track_topic, 10)
        
        self.tracks = {}
        self.next_id = 0
        
        self.get_logger().info(f'Tracker Node initialized')
    
    def detection_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        detections = self.parse_detections(cv_image)
        self.update_tracks(detections)
        
        if len(self.tracks) > 0:
            best_track = max(self.tracks.values(), key=lambda t: t['confidence'])
            target_point = Point()
            target_point.x = best_track['center_x']
            target_point.y = best_track['center_y']
            target_point.z = 0.0
            self.track_pub.publish(target_point)
    
    def parse_detections(self, image):
        detections = []
        return detections
    
    def update_tracks(self, detections):
        for det in detections:
            matched = False
            for track_id, track in self.tracks.items():
                dist = np.sqrt((det['center_x'] - track['center_x'])**2 + 
                              (det['center_y'] - track['center_y'])**2)
                if dist < 50:
                    track['center_x'] = det['center_x']
                    track['center_y'] = det['center_y']
                    track['confidence'] = det['confidence']
                    matched = True
                    break
            
            if not matched:
                self.tracks[self.next_id] = {
                    'center_x': det['center_x'],
                    'center_y': det['center_y'],
                    'confidence': det['confidence']
                }
                self.next_id += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
