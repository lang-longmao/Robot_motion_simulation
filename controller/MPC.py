from .controller import TrajectoryController
import numpy as np
import sys
sys.path.append("..") 
from base import RobotState, Point
from typing import Tuple, List



class MPCController(TrajectoryController):
    """优化后的模型预测控制器"""
    def __init__(self, horizon: int = 5):
        super().__init__()
        self.horizon = horizon
        self.max_v = 12.0  # 最大线速度
        self.max_w = 1.5   # 最大角速度
        
    def predict_trajectory(self, state: RobotState, controls: List[Tuple[float, float]], 
                          dt: float) -> List[RobotState]:
        """预测轨迹"""
        trajectory = [state]
        current = RobotState(state.x, state.y, state.theta, state.v, state.w)
        
        for v, w in controls:
            # 运动学模型
            current.x += v * np.cos(current.theta) * dt
            current.y += v * np.sin(current.theta) * dt
            current.theta += w * dt
            current.v = v
            current.w = w
            trajectory.append(RobotState(current.x, current.y, current.theta, v, w))
        
        return trajectory
    
    def compute_control(self, current_state: RobotState, 
                       target_point: Point, dt: float) -> Tuple[float, float]:
        """计算最优控制输入"""
        best_cost = float('inf')
        best_control = (0, 0)
        
        # 计算到目标的距离和角度误差
        dx = target_point.x - current_state.x
        dy = target_point.y - current_state.y
        distance_to_target = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)
        angle_error = target_angle - current_state.theta
        
        # 归一化角度误差到[-π, π]
        while angle_error > np.pi:
            angle_error -= 2 * np.pi
        while angle_error < -np.pi:
            angle_error += 2 * np.pi
        
        # 自适应网格搜索范围
        if distance_to_target > 5.0:
            # 远距离时优先考虑高速度
            v_candidates = np.linspace(self.max_v * 0.8, self.max_v, 8)
        elif distance_to_target > 2.0:
            # 中等距离时使用中等速度
            v_candidates = np.linspace(self.max_v * 0.5, self.max_v * 0.8, 8)
        else:
            # 近距离时使用较低速度
            v_candidates = np.linspace(0.5, self.max_v * 0.5, 6)
        
        # 角速度候选值基于角度误差调整
        if abs(angle_error) > 0.5:
            # 角度误差较大时增加角速度范围
            w_candidates = np.linspace(-self.max_w, self.max_w, 9)
        else:
            # 角度误差较小时减少角速度范围
            w_candidates = np.linspace(-self.max_w * 0.7, self.max_w * 0.7, 7)
        
        for v in v_candidates:
            for w in w_candidates:
                # 预测轨迹（使用递减的控制输入模拟更现实的控制策略）
                controls = []
                for i in range(self.horizon):
                    # 控制输入随时间递减，模拟到达目标后的减速
                    decay_factor = max(0.7, 1.0 - i * 0.1)
                    controls.append((v * decay_factor, w * decay_factor))
                
                trajectory = self.predict_trajectory(current_state, controls, dt)
                
                # 改进的成本函数
                cost = self.calculate_cost(trajectory, target_point, distance_to_target, v, w)
                
                if cost < best_cost:
                    best_cost = cost
                    best_control = (v, w)
        
        # 记录历史数据
        distance_error = distance_to_target
        self.error_history.append(distance_error)
        self.control_history.append(best_control)
        
        return best_control
    
    def calculate_cost(self, trajectory: List[RobotState], target_point: Point, 
                      initial_distance: float, v: float, w: float) -> float:
        """计算轨迹成本"""
        cost = 0.0
        
        # 权重参数
        distance_weight = 1.0
        control_weight = 0.01  # 显著降低控制成本权重
        speed_bonus_weight = 0.5  # 速度奖励权重
        smoothness_weight = 0.05  # 平滑性权重
        
        for i, state in enumerate(trajectory[1:], 1):
            # 1. 距离成本 - 使用平方根避免过度惩罚远距离
            dist = np.sqrt((state.x - target_point.x)**2 + (state.y - target_point.y)**2)
            cost += distance_weight * np.sqrt(dist)
            
            # 2. 终点奖励 - 鼓励快速接近目标
            if i == len(trajectory) - 1:  # 最后一个状态
                cost += distance_weight * dist * 2.0
        
        # 3. 控制成本 - 大幅降低权重
        cost += control_weight * (v**2 + w**2)
        
        # 4. 速度奖励 - 鼓励使用较高速度（当距离较远时）
        if initial_distance > 1.0:
            speed_bonus = speed_bonus_weight * (self.max_v - v) / self.max_v
            cost += speed_bonus
        
        # 5. 角速度平滑性 - 避免过度转向
        cost += smoothness_weight * abs(w)
        
        return cost