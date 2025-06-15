from .controller import TrajectoryController
import numpy as np
import sys
sys.path.append("..") 
from base import RobotState, Point
from typing import Tuple
import scipy



class LQRController(TrajectoryController):
    """线性二次调节器控制器"""
    def __init__(self):
        super().__init__()
        # LQR权重矩阵
        self.Q = np.diag([15.0, 15.0, 5.0])   # 状态权重 [x, y, theta]
        self.R = np.diag([0.01, 0.1])       # 控制权重 [v, w] - 降低线速度惩罚
    
        
        # 机器人动力学参数
        self.dt = 0.05  # 默认时间步长
        
        # 速度限制
        self.max_linear_velocity = 12.0
        self.max_angular_velocity = 1.5
        
        # 预计算的增益矩阵（将在首次使用时计算）
        self.K = None
        
    def _compute_system_matrices(self, current_state: RobotState, dt: float):
        """计算线性化系统矩阵"""
        theta = current_state.theta
        v = max(current_state.v, 0.1)  # 避免除零，使用最小速度
        
        # 状态矩阵 A (线性化后的)
        A = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0, 1]
        ])
        
        # 控制矩阵 B
        B = np.array([
            [np.cos(theta) * dt, 0],
            [np.sin(theta) * dt, 0],
            [0, dt]
        ])
        
        return A, B
    
    def _solve_riccati(self, A: np.ndarray, B: np.ndarray) -> np.ndarray:
        """求解代数Riccati方程获得最优增益矩阵K"""
        try:
            # 求解离散时间代数Riccati方程
            P = scipy.linalg.solve_discrete_are(A, B, self.Q, self.R)
            
            # 计算最优反馈增益矩阵
            K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)
            
            return K
        except Exception as e:
            print(f"Riccati求解失败: {e}")
            # 回退到手动调优的增益矩阵
            return np.array([[1.5, 0.8, 0.2], [0.1, 0.2, 2.5]])
    
    def _wrap_angle(self, angle: float) -> float:
        """角度归一化到[-π, π]"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def compute_control(self, current_state: RobotState, 
                       target_point: Point, dt: float) -> Tuple[float, float]:
        
        # 更新时间步长
        self.dt = dt
        
        # 计算状态误差
        error_x = target_point.x - current_state.x
        error_y = target_point.y - current_state.y
        distance_error = np.sqrt(error_x**2 + error_y**2)
        
        # 计算期望角度和角度误差
        target_theta = np.arctan2(error_y, error_x)
        error_theta = self._wrap_angle(target_theta - current_state.theta)
        
        # 状态误差向量
        x_error = np.array([error_x, error_y, error_theta])
        
        # 获取系统矩阵
        A, B = self._compute_system_matrices(current_state, dt)
        
        # 计算或更新LQR增益矩阵
        self.K = self._solve_riccati(A, B)
        
        # 计算LQR控制律
        u = self.K @ x_error
        
        # 提取控制指令
        v_cmd_raw = u[0]
        w_cmd_raw = u[1]
        
        # 速度自适应调整
        # 基于距离的速度增强
        distance_boost = min(2.0, 1.0 + distance_error * 0.5)
        v_cmd_raw *= distance_boost
        
        # 角度对齐检查 - 当角度误差过大时降低线速度
        angle_alignment = np.cos(error_theta)
        if angle_alignment < 0.5:  # 角度误差大于60度
            v_cmd_raw *= max(0.3, angle_alignment)
        
        # 应用速度限制
        v_cmd = np.clip(v_cmd_raw, 0, self.max_linear_velocity)
        w_cmd = np.clip(w_cmd_raw, -self.max_angular_velocity, self.max_angular_velocity)
        
        # 最小速度保证（当距离误差较大时）
        if distance_error > 0.5 and v_cmd < 1.0:
            v_cmd = 1.0
        
        # 记录历史数据
        self.error_history.append(distance_error)
        self.control_history.append((v_cmd, w_cmd))
        
        return v_cmd, w_cmd
    
    def update_weights(self, Q_diag: list, R_diag: list):
        """动态调整Q和R矩阵权重"""
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        # 重置增益矩阵，强制重新计算
        self.K = None
    
    def set_aggressive_tracking(self):
        """设置激进跟踪模式 - 高位置权重，低控制惩罚"""
        self.Q = np.diag([15.0, 15.0, 5.0])  # 增加位置和角度权重
        self.R = np.diag([0.01, 0.1])        # 大幅降低线速度惩罚
        self.K = None
    
    def set_smooth_tracking(self):
        """设置平滑跟踪模式 - 平衡权重"""
        self.Q = np.diag([5.0, 5.0, 2.0])
        self.R = np.diag([0.1, 0.2])
        self.K = None

    
