#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        self.declare_parameter('gimbal_topic', '/gimbal_cmd')
        self.declare_parameter('servo_topic', '/servo_cmd')
        self.declare_parameter('pitch_kp', 0.8)
        self.declare_parameter('yaw_kp', 0.8)
        self.declare_parameter('pitch_max', 30.0)
        self.declare_parameter('yaw_max', 45.0)
        
        gimbal_topic = self.get_parameter('gimbal_topic').value
        servo_topic = self.get_parameter('servo_topic').value
        self.pitch_kp = self.get_parameter('pitch_kp').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.pitch_max = math.radians(self.get_parameter('pitch_max').value)
        self.yaw_max = math.radians(self.get_parameter('yaw_max').value)
        
        self.gimbal_sub = self.create_subscription(
            Twist, gimbal_topic, self.gimbal_callback, 10)
        self.servo_pub = self.create_publisher(
            Twist, servo_topic, 10)
        
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        self.get_logger().info(f'Servo Controller initialized')
    
    def gimbal_callback(self, msg):
        target_pitch = msg.angular.x
        target_yaw = msg.angular.z
        
        pitch_error = target_pitch - self.current_pitch
        yaw_error = target_yaw - self.current_yaw
        
        pitch_cmd = self.current_pitch + self.pitch_kp * pitch_error
        yaw_cmd = self.current_yaw + self.yaw_kp * yaw_error
        
        pitch_cmd = max(-self.pitch_max, min(self.pitch_max, pitch_cmd))
        yaw_cmd = max(-self.yaw_max, min(self.yaw_max, yaw_cmd))
        
        self.current_pitch = pitch_cmd
        self.current_yaw = yaw_cmd
        
        servo_cmd = Twist()
        servo_cmd.angular.x = pitch_cmd
        servo_cmd.angular.z = yaw_cmd
        
        self.servo_pub.publish(servo_cmd)
        self.get_logger().debug(f'Servo cmd: pitch={math.degrees(pitch_cmd):.2f}deg, yaw={math.degrees(yaw_cmd):.2f}deg')

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
