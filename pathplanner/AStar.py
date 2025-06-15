from .planner import PathPlanner
import time, heapq, numpy as np
import sys 
sys.path.append("..") 
from base import Environment, Point
from typing import List

class AStarPlanner(PathPlanner):
    """A*启发式搜索算法"""
    def __init__(self, environment: Environment, grid_size: float = 2.0):
        super().__init__(environment)
        self.grid_size = grid_size
        
    def heuristic(self, p1: Point, p2: Point) -> float:
        """启发式函数：欧几里得距离"""
        return p1.distance_to(p2)
    
    def plan(self, start: Point, goal: Point) -> List[Point]:
        start_time = time.time()
        self.visited_nodes = []
        
        grid_start = Point(int(start.x // self.grid_size) * self.grid_size,
                          int(start.y // self.grid_size) * self.grid_size)
        grid_goal = Point(int(goal.x // self.grid_size) * self.grid_size,
                         int(goal.y // self.grid_size) * self.grid_size)
        
        # 优先队列：(f值, g值, 点)
        pq = [(self.heuristic(grid_start, grid_goal), 0, grid_start)]
        g_costs = {grid_start: 0}
        parents = {}
        closed_set = set()
        
        directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        
        while pq:
            f_cost, g_cost, current = heapq.heappop(pq)
            
            if current in closed_set:
                continue
                
            closed_set.add(current)
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
                path.append(goal)
                
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
                
                if self.env.is_collision(next_point) or next_point in closed_set:
                    continue
                
                move_cost = np.sqrt(dx*dx + dy*dy) * self.grid_size
                tentative_g = g_cost + move_cost
                
                if next_point not in g_costs or tentative_g < g_costs[next_point]:
                    g_costs[next_point] = tentative_g
                    f_cost = tentative_g + self.heuristic(next_point, grid_goal)
                    parents[next_point] = current
                    heapq.heappush(pq, (f_cost, tentative_g, next_point))
        
        self.planning_time = time.time() - start_time
        return []