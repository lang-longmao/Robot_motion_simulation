from .planner import PathPlanner
import time, heapq, numpy as np
import sys 
sys.path.append("..") 
from base import Environment, Point
from typing import List


class DijkstraPlanner(PathPlanner):
    """Dijkstra最短路径算法"""
    def __init__(self, environment: Environment, grid_size: float = 2.0):
        super().__init__(environment)
        self.grid_size = grid_size
        
    def plan(self, start: Point, goal: Point) -> List[Point]:
        start_time = time.time()
        self.visited_nodes = []
        
        # 创建网格
        grid_start = Point(int(start.x // self.grid_size) * self.grid_size,
                          int(start.y // self.grid_size) * self.grid_size)
        grid_goal = Point(int(goal.x // self.grid_size) * self.grid_size,
                         int(goal.y // self.grid_size) * self.grid_size)
        
        # 优先队列：(距离, 点)
        pq = [(0, grid_start)]
        distances = {grid_start: 0}
        parents = {}
        
        directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current in self.visited_nodes:
                continue
                
            self.visited_nodes.append(current)
            
            if current.distance_to(grid_goal) < self.grid_size:
                # 重构路径
                path = []
                node = current
                while node in parents:
                    path.append(node)
                    node = parents[node]
                path.append(grid_start)
                path.reverse()
                path.append(goal)  # 添加真实目标点
                
                self.path = path
                self.planning_time = time.time() - start_time
                return path
            
            # 探索邻居
            for dx, dy in directions:
                next_x = current.x + dx * self.grid_size
                next_y = current.y + dy * self.grid_size
                next_point = Point(next_x, next_y)
                
                if (next_x < 0 or next_x >= self.env.width or 
                    next_y < 0 or next_y >= self.env.height):
                    continue
                
                if self.env.is_collision(next_point):
                    continue
                
                # 计算移动代价
                move_cost = np.sqrt(dx*dx + dy*dy) * self.grid_size
                new_dist = current_dist + move_cost
                
                if next_point not in distances or new_dist < distances[next_point]:
                    distances[next_point] = new_dist
                    parents[next_point] = current
                    heapq.heappush(pq, (new_dist, next_point))
        
        self.planning_time = time.time() - start_time
        return []