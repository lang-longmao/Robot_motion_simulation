import numpy as np
import sys 
sys.path.append("..") 
from base import RobotState, Point
from typing import Tuple
from abc import ABC, abstractmethod

class TrajectoryController(ABC):
    """轨迹跟踪控制器抽象基类"""
    def __init__(self):
        self.control_history = []
        self.error_history = []
        
    @abstractmethod
    def compute_control(self, current_state: RobotState, 
                       target_point: Point, dt: float) -> Tuple[float, float]:
        """计算控制输入"""
        pass
    
    def get_statistics(self) -> dict:
        """获取控制统计信息"""
        if not self.error_history:
            return {'avg_error': 0, 'max_error': 0, 'final_error': 0}
        
        errors = [abs(e) for e in self.error_history]
        return {
            'avg_error': np.mean(errors),
            'max_error': np.max(errors),
            'final_error': errors[-1] if errors else 0
        }