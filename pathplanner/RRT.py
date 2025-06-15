from .planner import PathPlanner
import time, random
import sys 
sys.path.append("..") 
from base import Environment, Point
from typing import List


class RRTNode:
    def __init__(self, point: Point, parent=None):
        self.point = point
        self.parent = parent
        self.children = []

class RRTPlanner(PathPlanner):
    """快速扩展随机树算法"""
    def __init__(self, environment: Environment, max_iter: int = 20000, step_size: float = 2.5):
        super().__init__(environment)
        self.max_iter = max_iter
        self.step_size = step_size
        
    def get_random_point(self) -> Point:
        """获取随机采样点"""
        return Point(random.uniform(0, self.env.width), 
                    random.uniform(0, self.env.height))
    
    def get_nearest_node(self, tree: List[RRTNode], point: Point) -> RRTNode:
        """找到树中最近的节点"""
        min_dist = float('inf')
        nearest = None
        for node in tree:
            dist = node.point.distance_to(point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest
    
    def steer(self, from_point: Point, to_point: Point) -> Point:
        """从from_point向to_point方向扩展step_size距离"""
        dist = from_point.distance_to(to_point)
        if dist <= self.step_size:
            return to_point
        
        ratio = self.step_size / dist
        new_x = from_point.x + ratio * (to_point.x - from_point.x)
        new_y = from_point.y + ratio * (to_point.y - from_point.y)
        return Point(new_x, new_y)
    
    def is_path_collision_free(self, p1: Point, p2: Point, num_checks: int = 10) -> bool:
        """检查路径是否无碰撞"""
        for i in range(num_checks + 1):
            t = i / num_checks
            check_point = Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y))
            if self.env.is_collision(check_point):
                return False
        return True
    
    def plan(self, start: Point, goal: Point) -> List[Point]:
        start_time = time.time()
        self.visited_nodes = []
        
        tree = [RRTNode(start)]
        
        for _ in range(self.max_iter):
            # 采样随机点（偏向目标的采样）
            if random.random() < 0.1:  # 10%概率直接采样目标
                rand_point = goal
            else:
                rand_point = self.get_random_point()
            
            # 找最近节点
            nearest_node = self.get_nearest_node(tree, rand_point)
            
            # 扩展
            new_point = self.steer(nearest_node.point, rand_point)
            
            # 碰撞检测
            if (not self.env.is_collision(new_point) and 
                self.is_path_collision_free(nearest_node.point, new_point)):
                
                new_node = RRTNode(new_point, nearest_node)
                nearest_node.children.append(new_node)
                tree.append(new_node)
                self.visited_nodes.append(new_point)
                
                # 检查是否到达目标
                if new_point.distance_to(goal) < self.step_size:
                    # 重构路径
                    path = []
                    current = new_node
                    while current is not None:
                        path.append(current.point)
                        current = current.parent
                    path.reverse()
                    path.append(goal)
                    
                    self.path = path
                    self.planning_time = time.time() - start_time
                    return path
        
        self.planning_time = time.time() - start_time
        return []