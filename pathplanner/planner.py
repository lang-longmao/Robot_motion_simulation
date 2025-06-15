import sys 
sys.path.append("..") 
from base import Environment, Point
from typing import List
from abc import ABC, abstractmethod




class PathPlanner(ABC):
    """路径规划算法抽象基类"""
    def __init__(self, environment: Environment):
        self.env = environment
        self.path = []
        self.visited_nodes = []
        self.planning_time = 0.0
        
    @abstractmethod
    def plan(self, start: Point, goal: Point) -> List[Point]:
        """规划路径"""
        pass
    
    def get_statistics(self) -> dict:
        """获取规划统计信息"""
        path_length = 0
        if len(self.path) > 1:
            for i in range(len(self.path) - 1):
                path_length += self.path[i].distance_to(self.path[i + 1])
        
        return {
            'path_length': path_length,
            'nodes_explored': len(self.visited_nodes),
            'planning_time': self.planning_time,
            'path_points': len(self.path)
        }