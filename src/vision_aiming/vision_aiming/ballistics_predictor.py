#!/usr/bin/env python3
"""
弹道预测模型 - 匀速模型和重力补偿
"""
import numpy as np
import math
from typing import Tuple

class TargetState:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.timestamp = 0.0
        self.confidence = 0.0

class BallisticsPredictor:
    def __init__(self, bullet_speed=25.0, gravity=9.8, camera_height=0.3):
        self.bullet_speed = bullet_speed
        self.gravity = gravity
        self.camera_height = camera_height
        
        self.target_state = TargetState()
        self.velocity_filter_alpha = 0.3
        
        self.last_position = None
        self.last_timestamp = None
    
    def update_target(self, x: float, y: float, z: float, timestamp: float, confidence: float = 1.0):
        """更新目标状态"""
        new_position = np.array([x, y, z])
        
        if self.last_position is not None and self.last_timestamp is not None:
            dt = timestamp - self.last_timestamp
            if dt > 0.0 and dt < 0.5:
                measured_velocity = (new_position - self.last_position) / dt
                
                self.target_state.velocity = (
                    self.velocity_filter_alpha * measured_velocity +
                    (1 - self.velocity_filter_alpha) * self.target_state.velocity
                )
        
        self.target_state.position = new_position
        self.target_state.timestamp = timestamp
        self.target_state.confidence = confidence
        
        self.last_position = new_position
        self.last_timestamp = timestamp
    
    def predict_constant_velocity(self, predict_time: float) -> Tuple[float, float, float]:
        """匀速模型预测"""
        predicted_position = (
            self.target_state.position +
            self.target_state.velocity * predict_time
        )
        
        return tuple(predicted_position)
    
    def predict_with_gravity(self, predict_time: float) -> Tuple[float, float, float]:
        """考虑重力的预测模型"""
        target_x, target_y, target_z = self.target_state.position
        target_vx, target_vy, target_vz = self.target_state.velocity
        
        pred_x = target_x + target_vx * predict_time
        pred_y = target_y + target_vy * predict_time
        pred_z = target_z + target_vz * predict_time
        
        return pred_x, pred_y, pred_z
    
    def solve_pitch_angle(self, distance: float, height_diff: float) -> float:
        """
        求解发射俯仰角（考虑重力）
        返回角度（弧度）
        """
        if distance < 0.1:
            return 0.0
        
        v = self.bullet_speed
        g = self.gravity
        
        discriminant = v**4 - g * (g * distance**2 + 2 * height_diff * v**2)
        
        if discriminant < 0:
            return math.atan2(height_diff, distance)
        
        sqrt_discriminant = math.sqrt(discriminant)
        
        tan_angle1 = (v**2 + sqrt_discriminant) / (g * distance)
        tan_angle2 = (v**2 - sqrt_discriminant) / (g * distance)
        
        angle1 = math.atan(tan_angle1)
        angle2 = math.atan(tan_angle2)
        
        if abs(angle1) < abs(angle2):
            return angle1
        else:
            return angle2
    
    def compute_gimbal_angles(self, predict_time: float = 0.05) -> Tuple[float, float, float]:
        """
        计算云台控制角度
        返回: (pitch, yaw, 预测距离)
        """
        pred_x, pred_y, pred_z = self.predict_with_gravity(predict_time)
        
        pred_z = pred_z - self.camera_height
        
        distance = math.sqrt(pred_x**2 + pred_y**2 + pred_z**2)
        
        yaw = math.atan2(pred_y, pred_x)
        
        horizontal_dist = math.sqrt(pred_x**2 + pred_y**2)
        pitch = self.solve_pitch_angle(horizontal_dist, pred_z)
        
        return pitch, yaw, distance
    
    def estimate_flight_time(self, distance: float) -> float:
        """估算飞行时间"""
        if distance < 0.1:
            return 0.0
        return distance / self.bullet_speed
    
    def iterative_prediction(self, max_iterations: int = 5) -> Tuple[float, float]:
        """
        迭代预测：考虑飞行时间的预测
        """
        distance = np.linalg.norm(self.target_state.position)
        
        for i in range(max_iterations):
            flight_time = self.estimate_flight_time(distance)
            
            pred_x, pred_y, pred_z = self.predict_with_gravity(flight_time)
            pred_z = pred_z - self.camera_height
            
            new_distance = math.sqrt(pred_x**2 + pred_y**2 + pred_z**2)
            
            if abs(new_distance - distance) < 0.01:
                break
            
            distance = new_distance
        
        horizontal_dist = math.sqrt(pred_x**2 + pred_y**2)
        pitch = self.solve_pitch_angle(horizontal_dist, pred_z)
        yaw = math.atan2(pred_y, pred_x)
        
        return pitch, yaw

def main():
    predictor = BallisticsPredictor(bullet_speed=25.0, gravity=9.8, camera_height=0.3)
    
    predictor.update_target(3.0, 1.0, 0.5, timestamp=0.0)
    predictor.update_target(3.1, 1.05, 0.5, timestamp=0.05)
    predictor.update_target(3.2, 1.1, 0.5, timestamp=0.1)
    
    pitch, yaw, dist = predictor.compute_gimbal_angles()
    print(f"单次预测:")
    print(f"  Pitch: {math.degrees(pitch):.2f}°")
    print(f"  Yaw: {math.degrees(yaw):.2f}°")
    print(f"  Distance: {dist:.2f}m")
    
    pitch_iter, yaw_iter = predictor.iterative_prediction()
    print(f"\n迭代预测:")
    print(f"  Pitch: {math.degrees(pitch_iter):.2f}°")
    print(f"  Yaw: {math.degrees(yaw_iter):.2f}°")

if __name__ == '__main__':
    main()
