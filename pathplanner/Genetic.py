# fixing

from .planner import PathPlanner
import time, random
import sys 
sys.path.append("..") 
from base import Environment, Point
from typing import List


class GeneticPlanner(PathPlanner):
    """遗传算法路径规划"""
    def __init__(self, environment: Environment, population_size: int = 50, 
                 generations: int = 100, num_waypoints: int = 10):
        super().__init__(environment)
        self.population_size = population_size
        self.generations = generations
        self.num_waypoints = num_waypoints
        
    def create_individual(self, start: Point, goal: Point) -> List[Point]:
        """创建个体（路径）"""
        waypoints = [start]
        for _ in range(self.num_waypoints):
            x = random.uniform(0, self.env.width)
            y = random.uniform(0, self.env.height)
            waypoints.append(Point(x, y))
        waypoints.append(goal)
        return waypoints
    
    def fitness(self, individual: List[Point]) -> float:
        """适应度函数"""
        # 路径长度
        length = 0
        collision_penalty = 0
        
        for i in range(len(individual) - 1):
            length += individual[i].distance_to(individual[i + 1])
            
            # 检查路径段是否有碰撞
            if not self.is_path_segment_free(individual[i], individual[i + 1]):
                collision_penalty += 1000
        
        # 适应度越高越好，所以返回负值
        return -(length + collision_penalty)
    
    def is_path_segment_free(self, p1: Point, p2: Point) -> bool:
        """检查路径段是否无碰撞"""
        num_checks = int(p1.distance_to(p2) / 2) + 1
        for i in range(num_checks + 1):
            t = i / num_checks if num_checks > 0 else 0
            check_point = Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y))
            if self.env.is_collision(check_point):
                return False
        return True
    
    def crossover(self, parent1: List[Point], parent2: List[Point]) -> List[Point]:
        """交叉操作"""
        child = [parent1[0]]  # 保持起点
        
        for i in range(1, len(parent1) - 1):
            if random.random() < 0.5:
                child.append(parent1[i])
            else:
                child.append(parent2[i])
        
        child.append(parent1[-1])  # 保持终点
        return child
    
    def mutate(self, individual: List[Point], mutation_rate: float = 0.1) -> List[Point]:
        """变异操作"""
        mutated = individual.copy()
        
        for i in range(1, len(mutated) - 1):  # 不变异起点和终点
            if random.random() < mutation_rate:
                # 在当前点附近随机扰动
                noise_x = random.uniform(-10, 10)
                noise_y = random.uniform(-10, 10)
                new_x = max(0, min(self.env.width, mutated[i].x + noise_x))
                new_y = max(0, min(self.env.height, mutated[i].y + noise_y))
                mutated[i] = Point(new_x, new_y)
        
        return mutated
    
    def plan(self, start: Point, goal: Point) -> List[Point]:
        start_time = time.time()
        self.visited_nodes = []
        
        # 初始化种群
        population = []
        for _ in range(self.population_size):
            individual = self.create_individual(start, goal)
            population.append(individual)
        
        best_individual = None
        best_fitness = float('-inf')
        
        for generation in range(self.generations):
            # 评估适应度
            fitness_scores = [(self.fitness(ind), ind) for ind in population]
            fitness_scores.sort(reverse=True)  # 按适应度降序排列
            
            # 更新最佳个体
            if fitness_scores[0][0] > best_fitness:
                best_fitness = fitness_scores[0][0]
                best_individual = fitness_scores[0][1].copy()
            
            # 选择
            elite_size = self.population_size // 4
            new_population = [ind for _, ind in fitness_scores[:elite_size]]
            
            # 生成新个体
            while len(new_population) < self.population_size:
                # 轮盘赌选择
                parent1 = self.tournament_selection(fitness_scores)
                parent2 = self.tournament_selection(fitness_scores)
                
                child = self.crossover(parent1, parent2)
                child = self.mutate(child)
                new_population.append(child)
            
            population = new_population
        
        self.path = best_individual if best_individual else []
        self.planning_time = time.time() - start_time
        return self.path
    
    def tournament_selection(self, fitness_scores: List, tournament_size: int = 3):
        """锦标赛选择"""
        tournament = random.sample(fitness_scores, min(tournament_size, len(fitness_scores)))
        return max(tournament, key=lambda x: x[0])[1]