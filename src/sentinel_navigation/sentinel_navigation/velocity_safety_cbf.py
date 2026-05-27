#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class VelocitySafetyCBF(Node):
    def __init__(self):
        super().__init__('velocity_safety_cbf')
        
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('max_decel', 1.0)
        self.declare_parameter('cbf_gain', 2.0)
        
        self.safe_distance = self.get_parameter('safe_distance').value
        self.max_decel = self.get_parameter('max_decel').value
        self.cbf_gain = self.get_parameter('cbf_gain').value
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.safe_vel_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        
        self.latest_scan = None
        self.latest_cmd_vel = None
        
        self.timer = self.create_timer(0.02, self.apply_cbf_filter)
        
        self.get_logger().info(f'CBF Safety Filter initialized: safe_distance={self.safe_distance}m')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
    
    def compute_min_distance_in_direction(self, velocity_angle):
        if self.latest_scan is None:
            return float('inf')
        
        min_dist = float('inf')
        ranges = np.array(self.latest_scan.ranges)
        angles = np.arange(len(ranges)) * self.latest_scan.angle_increment + self.latest_scan.angle_min
        
        angle_range = math.pi / 3
        mask = np.abs(angles - velocity_angle) < angle_range
        valid_ranges = ranges[mask]
        valid_ranges = valid_ranges[valid_ranges > self.latest_scan.range_min]
        
        if len(valid_ranges) > 0:
            min_dist = np.min(valid_ranges)
        
        return min_dist
    
    def apply_cbf_filter(self):
        if self.latest_cmd_vel is None:
            return
        
        safe_vel = Twist()
        safe_vel.angular = self.latest_cmd_vel.angular
        
        if abs(self.latest_cmd_vel.linear.x) < 0.001:
            safe_vel.linear = self.latest_cmd_vel.linear
            self.safe_vel_pub.publish(safe_vel)
            return
        
        velocity_angle = 0.0 if self.latest_cmd_vel.linear.x > 0 else math.pi
        
        min_dist = self.compute_min_distance_in_direction(velocity_angle)
        
        if min_dist < self.safe_distance:
            scale_factor = max(0.0, (min_dist / self.safe_distance) ** 2)
            safe_vel.linear.x = self.latest_cmd_vel.linear.x * scale_factor
            self.get_logger().debug(f'CBF activated: dist={min_dist:.2f}m, scale={scale_factor:.2f}')
        else:
            h = min_dist - self.safe_distance
            dh_dt = -abs(self.latest_cmd_vel.linear.x)
            
            alpha_h = self.cbf_gain * h
            
            if dh_dt + alpha_h < 0:
                safe_vel.linear.x = self.latest_cmd_vel.linear.x * 0.5
            else:
                safe_vel.linear.x = self.latest_cmd_vel.linear.x
        
        safe_vel.linear.y = self.latest_cmd_vel.linear.y
        safe_vel.linear.z = self.latest_cmd_vel.linear.z
        
        self.safe_vel_pub.publish(safe_vel)

def main(args=None):
    rclpy.init(args=args)
    node = VelocitySafetyCBF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
