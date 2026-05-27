#!/usr/bin/env python3
"""
增强版CBF安全过滤器 - 支持多方向检测和旋转约束
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class EnhancedCBFSafetyFilter(Node):
    def __init__(self):
        super().__init__('enhanced_cbf_safety_filter')
        
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('critical_distance', 0.3)
        self.declare_parameter('cbf_alpha', 2.0)
        self.declare_parameter('max_linear_vel', 1.5)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('num_sectors', 8)
        
        self.safe_distance = self.get_parameter('safe_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.cbf_alpha = self.get_parameter('cbf_alpha').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.num_sectors = self.get_parameter('num_sectors').value
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.safe_vel_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        
        self.latest_scan = None
        self.latest_cmd_vel = None
        
        self.timer = self.create_timer(0.02, self.apply_cbf_filter)
        
        self.get_logger().info(f'Enhanced CBF Filter initialized: safe_dist={self.safe_distance}m')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
    
    def compute_sector_distances(self):
        """将激光扫描分成多个扇区，计算每个扇区的最小距离"""
        if self.latest_scan is None:
            return [float('inf')] * self.num_sectors
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.arange(len(ranges)) * self.latest_scan.angle_increment + self.latest_scan.angle_min
        
        sector_angles = np.linspace(-np.pi, np.pi, self.num_sectors + 1)
        sector_distances = []
        
        for i in range(self.num_sectors):
            mask = (angles >= sector_angles[i]) & (angles < sector_angles[i+1])
            valid_ranges = ranges[mask]
            valid_ranges = valid_ranges[(valid_ranges > self.latest_scan.range_min) & 
                                        (valid_ranges < self.latest_scan.range_max)]
            
            if len(valid_ranges) > 0:
                sector_distances.append(np.min(valid_ranges))
            else:
                sector_distances.append(float('inf'))
        
        return sector_distances
    
    def compute_cbf_scale(self, distance):
        """基于CBF计算安全缩放因子"""
        if distance >= self.safe_distance:
            return 1.0
        elif distance <= self.critical_distance:
            return 0.0
        else:
            h = distance - self.critical_distance
            h_max = self.safe_distance - self.critical_distance
            scale = (h / h_max) ** 2
            return scale
    
    def compute_velocity_direction_sector(self, vx, vy):
        """根据速度方向确定所在扇区"""
        if abs(vx) < 0.001 and abs(vy) < 0.001:
            return -1
        
        velocity_angle = math.atan2(vy, vx)
        sector_size = 2.0 * np.pi / self.num_sectors
        sector_idx = int((velocity_angle + np.pi) / sector_size) % self.num_sectors
        return sector_idx
    
    def check_angular_safety(self, angular_vel, sector_distances):
        """检查旋转是否安全"""
        if abs(angular_vel) < 0.01:
            return True, angular_vel
        
        left_sectors = [sector_distances[i % self.num_sectors] 
                       for i in range(self.num_sectors // 4, 3 * self.num_sectors // 4)]
        right_sectors = [sector_distances[i % self.num_sectors] 
                        for i in range(5 * self.num_sectors // 4, 7 * self.num_sectors // 4)]
        
        if angular_vel > 0:
            min_dist = min(left_sectors)
        else:
            min_dist = min(right_sectors)
        
        if min_dist < self.critical_distance:
            scale = self.compute_cbf_scale(min_dist)
            safe_angular = angular_vel * scale
            return False, safe_angular
        
        return True, angular_vel
    
    def apply_cbf_filter(self):
        if self.latest_cmd_vel is None:
            return
        
        safe_vel = Twist()
        
        if self.latest_scan is None:
            safe_vel = self.latest_cmd_vel
            self.safe_vel_pub.publish(safe_vel)
            return
        
        sector_distances = self.compute_sector_distances()
        
        vx = self.latest_cmd_vel.linear.x
        vy = self.latest_cmd_vel.linear.y
        omega = self.latest_cmd_vel.angular.z
        
        if abs(vx) > 0.001 or abs(vy) > 0.001:
            forward_sector = self.compute_velocity_direction_sector(vx, vy)
            
            if forward_sector >= 0 and forward_sector < len(sector_distances):
                min_dist = sector_distances[forward_sector]
                
                scale = self.compute_cbf_scale(min_dist)
                
                safe_vel.linear.x = vx * scale
                safe_vel.linear.y = vy * scale
            else:
                safe_vel.linear.x = vx
                safe_vel.linear.y = vy
        else:
            safe_vel.linear.x = 0.0
            safe_vel.linear.y = 0.0
        
        is_safe, safe_omega = self.check_angular_safety(omega, sector_distances)
        safe_vel.angular.z = safe_omega
        
        safe_vel.linear.x = np.clip(safe_vel.linear.x, -self.max_linear_vel, self.max_linear_vel)
        safe_vel.linear.y = np.clip(safe_vel.linear.y, -self.max_linear_vel, self.max_linear_vel)
        safe_vel.angular.z = np.clip(safe_vel.angular.z, -self.max_angular_vel, self.max_angular_vel)
        
        self.safe_vel_pub.publish(safe_vel)
        
        if abs(vx - safe_vel.linear.x) > 0.01 or abs(omega - safe_vel.angular.z) > 0.01:
            self.get_logger().debug(
                f'CBF activated: linear_scale={abs(safe_vel.linear.x/(vx+0.001)):.2f}, '
                f'angular_scale={abs(safe_vel.angular.z/(omega+0.001)):.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedCBFSafetyFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
