# 基础数据结构和配置

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass

@dataclass
class Point:
    x: float
    y: float
    
    def distance_to(self, other: 'Point') -> float:
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __lt__(self, other):
        if self.x != other.x:
            return self.x < other.x
        return self.y < other.y
    
    def __eq__(self, other):
        return abs(self.x - other.x) < 1e-6 and abs(self.y - other.y) < 1e-6
    
    def __hash__(self):
        return hash((round(self.x, 6), round(self.y, 6)))

@dataclass
class RobotState:
    x: float
    y: float
    theta: float  # 朝向角
    v: float = 0.0  # 线速度
    w: float = 0.0  # 角速度

class Environment:
    """环境类，定义地图和障碍物"""
    def __init__(self, width: int = 100, height: int = 100):
        self.width = width
        self.height = height
        self.obstacles = []
        self.start = Point(10, 10)
        self.goal = Point(90, 90)
        
    def add_obstacle(self, x: float, y: float, width: float, height: float):
        """添加矩形障碍物"""
        self.obstacles.append((x, y, width, height))
    
    def is_collision(self, point: Point, robot_radius: float = 5.0) -> bool:
        """检查点是否与障碍物碰撞"""
        if point.x < robot_radius or point.x > self.width - robot_radius:
            return True
        if point.y < robot_radius or point.y > self.height - robot_radius:
            return True
            
        for obs_x, obs_y, obs_w, obs_h in self.obstacles:
            if (obs_x - robot_radius <= point.x <= obs_x + obs_w + robot_radius and
                obs_y - robot_radius <= point.y <= obs_y + obs_h + robot_radius):
                return True
        return False
    
    #不膨胀碰撞检查
    def is_collision_true(self, point: Point, robot_radius: float = 5.0) -> bool:
        """检查点是否与障碍物碰撞"""
            
        for obs_x, obs_y, obs_w, obs_h in self.obstacles:
            if (obs_x <= point.x <= obs_x + obs_w  and
                obs_y <= point.y <= obs_y + obs_h ):
                return True
        return False
    

    
    def create_sample_environment(self):
        """创建示例环境"""
        # 添加一些障碍物
        self.add_obstacle(20, 20, 15, 30)
        self.add_obstacle(50, 10, 20, 15)
        self.add_obstacle(40, 40, 25, 20)
        self.add_obstacle(70, 30, 15, 25)
        self.add_obstacle(30, 70, 20, 15)
        self.add_obstacle(60, 60, 15, 20)

class Robot:
    """移动机器人类"""
    def __init__(self, initial_state: RobotState):
        self.state = initial_state
        self.trajectory = [RobotState(initial_state.x, initial_state.y, 
                                    initial_state.theta, 0, 0)]
        
    def update_state(self, v_cmd: float, w_cmd: float, dt: float):
        """更新机器人状态"""
        # 差分驱动运动学模型
        self.state.v = v_cmd
        self.state.w = w_cmd
        self.state.x += v_cmd * np.cos(self.state.theta) * dt
        self.state.y += v_cmd * np.sin(self.state.theta) * dt
        self.state.theta += w_cmd * dt
        
        # 保存轨迹
        self.trajectory.append(RobotState(self.state.x, self.state.y, 
                                        self.state.theta, v_cmd, w_cmd))