#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math

class BallisticsNode(Node):
    def __init__(self):
        super().__init__('ballistics_node')
        
        self.declare_parameter('target_topic', '/tracked_targets')
        self.declare_parameter('gimbal_topic', '/gimbal_cmd')
        self.declare_parameter('bullet_speed', 25.0)
        self.declare_parameter('camera_height', 0.3)
        
        target_topic = self.get_parameter('target_topic').value
        gimbal_topic = self.get_parameter('gimbal_topic').value
        self.bullet_speed = self.get_parameter('bullet_speed').value
        self.camera_height = self.get_parameter('camera_height').value
        
        self.target_sub = self.create_subscription(
            Point, target_topic, self.target_callback, 10)
        self.gimbal_pub = self.create_publisher(
            Twist, gimbal_topic, 10)
        
        self.get_logger().info(f'Ballistics Node initialized: bullet_speed={self.bullet_speed}m/s')
    
    def target_callback(self, msg):
        target_x = msg.x
        target_y = msg.y
        target_z = msg.z
        
        distance = math.sqrt(target_x**2 + target_y**2 + (target_z - self.camera_height)**2)
        
        flight_time = distance / self.bullet_speed
        
        gravity_compensation = 0.5 * 9.8 * flight_time**2
        
        pitch = math.atan2(target_z - self.camera_height + gravity_compensation, 
                          math.sqrt(target_x**2 + target_y**2))
        yaw = math.atan2(target_y, target_x)
        
        gimbal_cmd = Twist()
        gimbal_cmd.angular.x = pitch
        gimbal_cmd.angular.z = yaw
        
        self.gimbal_pub.publish(gimbal_cmd)
        self.get_logger().debug(f'Gimbal cmd: pitch={math.degrees(pitch):.2f}deg, yaw={math.degrees(yaw):.2f}deg')

def main(args=None):
    rclpy.init(args=args)
    node = BallisticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
