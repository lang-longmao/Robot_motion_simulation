from base import Environment, Point, RobotState, Robot
from pathplanner.Dijkstra import DijkstraPlanner
from pathplanner.AStar import AStarPlanner
from pathplanner.RRT import RRTPlanner
from pathplanner.Genetic import GeneticPlanner
from controller.PID import PIDController
from controller.LQR import LQRController
from controller.MPC import MPCController
from smoother import PathSmoother
import time

class PathPlanningSimulator:
    """核心路径规划仿真器"""
    def __init__(self, environment: Environment = None):
        self.env = environment if environment else Environment()
        self.robot = None
        self.planner = None
        self.controller = None
        self.path = []
        self.smoothed_path = []
        self.path_smoother = PathSmoother()
        
    def setup_planner(self, planner_type: str):
        """设置路径规划器"""
        planners = {
            'dijkstra': DijkstraPlanner,
            'astar': AStarPlanner,
            'rrt': RRTPlanner,
            'genetic': GeneticPlanner
        }
        
        if planner_type not in planners:
            raise ValueError(f"Unknown planner type: {planner_type}")
        
        self.planner = planners[planner_type](self.env)
    
    def setup_controller(self, controller_type: str):
        """设置跟踪控制器"""
        controllers = {
            'pid': PIDController,
            'lqr': LQRController,
            'mpc': MPCController
        }
        
        if controller_type not in controllers:
            raise ValueError(f"Unknown controller type: {controller_type}")
            
        self.controller = controllers[controller_type]()
    
    def plan_path(self, smoothing_method='gradient_descent'):
        """执行路径规划"""
        if self.planner is None:
            raise ValueError("Planner not initialized")
        
        print(f"开始使用 {type(self.planner).__name__} 进行路径规划...")
        self.path = self.planner.plan(self.env.start, self.env.goal)
        
        if self.path:
            stats = self.planner.get_statistics()
            print(f"规划成功! 路径长度: {stats['path_length']:.2f}")
            
            # 应用路径平滑
            self.smoothed_path = self._apply_path_smoothing(smoothing_method)
            if self.smoothed_path:
                self.path = self.smoothed_path
        else:
            print("规划失败!")
    
    def _apply_path_smoothing(self, method='gradient_descent'):
        """应用路径平滑算法"""
        if not self.path:
            return []
        
        start_time = time.time()
        
        try:
            smoothing_methods = {
                'gradient_descent': lambda: self.path_smoother.smooth_path_gradient_descent(
                    self.path, self.env, iterations=100, alpha=0.1, beta=0.3),
                'spline': lambda: self.path_smoother.smooth_path_spline(
                    self.path, num_points=len(self.path)*2, smoothing_factor=0),
                'bezier': lambda: self.path_smoother.smooth_path_bezier(self.path),
                'douglas_peucker': lambda: self.path_smoother.douglas_peucker_simplify(
                    self.path, epsilon=2.0),
                'combined': lambda: self._combined_smoothing()
            }
            
            if method not in smoothing_methods:
                raise ValueError(f"Unknown smoothing method: {method}")
            
            smoothed = smoothing_methods[method]()
            
            if self._validate_smoothed_path(smoothed):
                print(f"平滑处理完成，耗时: {time.time() - start_time:.3f}秒")
                return smoothed
            else:
                print("平滑路径验证失败，使用原始路径")
                return self.path.copy()
                
        except Exception as e:
            print(f"路径平滑出错: {e}")
            return self.path.copy()
    
    def _combined_smoothing(self):
        """组合平滑方法"""
        simplified = self.path_smoother.douglas_peucker_simplify(self.path, epsilon=2.0)
        return self.path_smoother.smooth_path_gradient_descent(
            simplified, self.env, iterations=50, alpha=0.1, beta=0.3)
    
    def _validate_smoothed_path(self, smoothed_path):
        """验证平滑路径的有效性"""
        if not smoothed_path:
            return False
        
        collision_count = sum(1 for point in smoothed_path if self.env.is_collision_true(point))
        collision_ratio = collision_count / len(smoothed_path)
        
        if collision_ratio > 0.05:
            print(f"警告: 平滑路径有{collision_ratio*100:.1f}%的点发生碰撞")
            return False
        
        return True
    
    def simulate_tracking(self, dt: float = 0.1, max_time: float = 50.0):
        """仿真轨迹跟踪"""
        if not self.path or self.controller is None:
            raise ValueError("Path or controller not initialized")
        
        initial_state = RobotState(self.env.start.x, self.env.start.y, 0)
        self.robot = Robot(initial_state)
        
        print(f"开始使用 {type(self.controller).__name__} 进行轨迹跟踪...")
        
        current_target_idx = 1
        time_elapsed = 0
        
        while (current_target_idx < len(self.path) and time_elapsed < max_time):
            target_point = self.path[current_target_idx]
            
            v_cmd, w_cmd = self.controller.compute_control(
                self.robot.state, target_point, dt)
            
            self.robot.update_state(v_cmd, w_cmd, dt)
            
            distance_to_target = Point(self.robot.state.x, self.robot.state.y).distance_to(target_point)
            if distance_to_target < 3.0:
                current_target_idx += 1
                print(f"到达路径点 {current_target_idx-1}/{len(self.path)-1}")
            
            time_elapsed += dt
        
        stats = self.controller.get_statistics()
        print(f"跟踪完成! 平均误差: {stats['avg_error']:.2f}")
        return stats