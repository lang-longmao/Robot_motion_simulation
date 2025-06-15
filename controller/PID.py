from .controller import TrajectoryController
import numpy as np
import sys
sys.path.append("..") 
from base import RobotState, Point
from typing import Tuple



class PIDController(TrajectoryController):
    """PID轨迹跟踪控制器"""
    def __init__(self, kp: float = 2.0, ki: float = 0.001, kd: float = 0.1):
        super().__init__()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.previous_error = 0
        
    def compute_control(self, current_state: RobotState, 
                       target_point: Point, dt: float) -> Tuple[float, float]:
        # 位置误差
        error_x = target_point.x - current_state.x
        error_y = target_point.y - current_state.y
        distance_error = np.sqrt(error_x**2 + error_y**2)
        
        # 角度误差
        target_angle = np.arctan2(error_y, error_x)
        angle_error = target_angle - current_state.theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # 归一化到[-π, π]
        
        # PID控制
        self.integral_error += distance_error * dt
        derivative_error = (distance_error - self.previous_error) / dt if dt > 0 else 0
        
        # 线速度控制
        v_cmd = self.kp * distance_error + self.ki * self.integral_error + self.kd * derivative_error
        v_cmd = np.clip(v_cmd, 0, 12)  # 限制最大速度
        
        # 角速度控制
        w_cmd = 3.0 * angle_error  # 简单比例控制
        w_cmd = np.clip(w_cmd, -2, 2)  # 限制最大角速度
        
        self.previous_error = distance_error
        self.error_history.append(distance_error)
        self.control_history.append((v_cmd, w_cmd))
        
        return v_cmd, w_cmd