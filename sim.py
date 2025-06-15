# from base import Environment, Point, RobotState, Robot
# import matplotlib.pyplot as plt
# from matplotlib.patches import Rectangle
# from matplotlib.animation import FuncAnimation
# from pathplanner.Dijkstra import DijkstraPlanner
# from pathplanner.AStar import AStarPlanner
# from pathplanner.RRT import RRTPlanner
# from pathplanner.Genetic import GeneticPlanner
# from controller.PID import PIDController
# from controller.LQR import LQRController
# from controller.MPC import MPCController
# import numpy as np
# from smoother import PathSmoother
# import time

# plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
# plt.rcParams['axes.unicode_minus'] = False

# class DynamicPathPlanningSimulator:
#     """动态路径规划仿真器"""
#     def __init__(self):
#         self.env = Environment()
#         self.env.create_sample_environment()
#         self.robot = None
#         self.planner = None
#         self.controller = None
#         self.path = []
#         self.smoothed_path = []
#         self.path_smoother = PathSmoother()
        
#         # 动画相关参数
#         self.animation = None
#         self.is_animating = False
#         self.current_target_idx = 1
#         self.time_elapsed = 0
#         self.dt = 0.05
#         self.max_time = 50.0
        
#         # 存储动画数据
#         self.robot_positions = []
#         self.robot_orientations = []
#         self.animation_times = []
#         self.real_time_errors = []
#         self.real_time_controls = []
        
#         # 创建图形界面
#         self.setup_figure()
        
#     def setup_figure(self):
#         """设置图形界面"""
#         self.fig = plt.figure(figsize=(18, 10))
        
#         # 创建子图布局
#         gs = self.fig.add_gridspec(2, 3, height_ratios=[2, 1], width_ratios=[2, 1, 1])
        
#         # 主仿真窗口
#         self.ax_main = self.fig.add_subplot(gs[0, :2])
#         self.ax_main.set_title('实时机器人运动仿真', fontsize=16, fontweight='bold')
        
#         # 控制输入实时图
#         self.ax_control = self.fig.add_subplot(gs[0, 2])
#         self.ax_control.set_title('实时控制输入', fontsize=12, fontweight='bold')
        
#         # 误差分析图
#         self.ax_error = self.fig.add_subplot(gs[1, 0])
#         self.ax_error.set_title('跟踪误差', fontsize=12, fontweight='bold')
        
#         # 速度曲线图
#         self.ax_velocity = self.fig.add_subplot(gs[1, 1])
#         self.ax_velocity.set_title('速度曲线', fontsize=12, fontweight='bold')
        
#         # 状态信息显示
#         self.ax_info = self.fig.add_subplot(gs[1, 2])
#         self.ax_info.set_title('状态信息', fontsize=12, fontweight='bold')
#         self.ax_info.axis('off')
        
#         plt.tight_layout()
        
#     def setup_planner(self, planner_type: str):
#         """设置路径规划器"""
#         if planner_type == 'dijkstra':
#             self.planner = DijkstraPlanner(self.env)
#         elif planner_type == 'astar':
#             self.planner = AStarPlanner(self.env)
#         elif planner_type == 'rrt':
#             self.planner = RRTPlanner(self.env)
#         elif planner_type == 'genetic':
#             self.planner = GeneticPlanner(self.env)
#         else:
#             raise ValueError(f"Unknown planner type: {planner_type}")
    
#     def setup_controller(self, controller_type: str):
#         """设置跟踪控制器"""
#         if controller_type == 'pid':
#             self.controller = PIDController()
#         elif controller_type == 'lqr':
#             self.controller = LQRController()
#         elif controller_type == 'mpc':
#             self.controller = MPCController()
#         else:
#             raise ValueError(f"Unknown controller type: {controller_type}")
    
#     def plan_path(self, smoothing_method='gradient_descent'):
#         """执行路径规划"""
#         if self.planner is None:
#             raise ValueError("Planner not initialized")
        
#         print(f"开始使用 {type(self.planner).__name__} 进行路径规划...")
#         self.path = self.planner.plan(self.env.start, self.env.goal)
        
#         if self.path:
#             stats = self.planner.get_statistics()
#             print(f"规划成功!")
#             print(f"  路径长度: {stats['path_length']:.2f}")
#             print(f"  探索节点数: {stats['nodes_explored']}")
#             print(f"  规划时间: {stats['planning_time']:.3f}秒")
#             print(f"  路径点数: {stats['path_points']}")
#         else:
#             print("规划失败!")

#         print(f"\n开始路径平滑处理 ({smoothing_method})...")
#         self.smoothed_path = self.apply_path_smoothing(smoothing_method)
                
#         if self.smoothed_path:
#             # 计算平滑后的路径统计
#             smoothed_length = self.calculate_path_length(self.smoothed_path)
#             print(f"平滑处理完成!")
#             print(f"  平滑后路径长度: {smoothed_length:.2f}")
#             print(f"  平滑后路径点数: {len(self.smoothed_path)}")
#             print(f"  长度变化: {((smoothed_length - stats['path_length']) / stats['path_length'] * 100):+.1f}%")
#             print(f"  点数变化: {((len(self.smoothed_path) - stats['path_points']) / stats['path_points'] * 100):+.1f}%")
#             self.path = self.smoothed_path
#         else:
#             print("路径平滑失败，使用原始路径")
#             self.smoothed_path = self.path.copy()

    
#     def apply_path_smoothing(self, method='gradient_descent'):
#         """应用路径平滑算法"""
#         if not self.path:
#             return []
        
#         start_time = time.time()
        
#         try:
#             if method == 'gradient_descent':
#                 smoothed = self.path_smoother.smooth_path_gradient_descent(
#                     self.path, self.env, iterations=100, alpha=0.1, beta=0.3)
#             elif method == 'spline':
#                 smoothed = self.path_smoother.smooth_path_spline(
#                     self.path, num_points=len(self.path)*2, smoothing_factor=0)
#             elif method == 'bezier':
#                 smoothed = self.path_smoother.smooth_path_bezier(self.path)
#             elif method == 'douglas_peucker':
#                 smoothed = self.path_smoother.douglas_peucker_simplify(self.path, epsilon=2.0)
#             elif method == 'combined':
#                 # 组合方法：先简化，再平滑
#                 simplified = self.path_smoother.douglas_peucker_simplify(self.path, epsilon=2.0)
#                 smoothed = self.path_smoother.smooth_path_gradient_descent(
#                     simplified, self.env, iterations=50, alpha=0.1, beta=0.3)
#             else:
#                 raise ValueError(f"Unknown smoothing method: {method}")
            
#             # 验证平滑路径的有效性
#             if self.validate_smoothed_path(smoothed):
#                 smoothing_time = time.time() - start_time
#                 print(f"  平滑处理时间: {smoothing_time:.3f}秒")
#                 return smoothed
#             else:
#                 print("  平滑路径验证失败，使用原始路径")
#                 return self.path.copy()
                
#         except Exception as e:
#             print(f"  路径平滑出错: {e}")
#             return self.path.copy()
    
#     def validate_smoothed_path(self, smoothed_path):
#         """验证平滑路径的有效性"""
#         if not smoothed_path:
#             return False
        
#         # 检查路径是否仍然无碰撞
#         collision_count = 0
#         for point in smoothed_path:
#             if self.env.is_collision_true(point):
#                 collision_count += 1
        
#         # 允许少量碰撞点（可能是数值误差）
#         collision_ratio = collision_count / len(smoothed_path)
#         if collision_ratio > 0.05:  # 超过5%的点碰撞
#             print(f"  警告: 平滑路径有{collision_ratio*100:.1f}%的点发生碰撞")
#             return False
        
#         return True
    
#     def calculate_path_length(self, path):
#         """计算路径长度"""
#         if len(path) < 2:
#             return 0.0
        
#         total_length = 0.0
#         for i in range(1, len(path)):
#             total_length += path[i-1].distance_to(path[i])
        
#         return total_length


#     def init_animation(self):
#         """初始化动画"""
#         # 初始化机器人
#         initial_state = RobotState(self.env.start.x, self.env.start.y, 0)
#         self.robot = Robot(initial_state)
        
#         # 重置动画参数
#         self.current_target_idx = 1
#         self.time_elapsed = 0
#         self.robot_positions = [(self.robot.state.x, self.robot.state.y)]
#         self.robot_orientations = [self.robot.state.theta]
#         self.animation_times = [0]
#         self.real_time_errors = [0]
#         self.real_time_controls = [(0, 0)]
        
#         # 设置主图
#         self.ax_main.clear()
#         self.ax_main.set_xlim(0, self.env.width)
#         self.ax_main.set_ylim(0, self.env.height)
#         self.ax_main.set_aspect('equal')
#         self.ax_main.grid(True, alpha=0.3)
        
#         # 绘制静态元素
#         self.draw_static_elements()
        
#         # 初始化动态元素
#         self.robot_point, = self.ax_main.plot([], [], 'ko', markersize=10, zorder=5)
#         self.robot_arrow = self.ax_main.annotate('', xy=(0, 0), xytext=(0, 0),
#                                                arrowprops=dict(arrowstyle='->', 
#                                                              color='black', lw=2),
#                                                zorder=5)
#         self.trajectory_line, = self.ax_main.plot([], [], 'm-', linewidth=2, 
#                                                 alpha=0.8, label='实际轨迹', zorder=4)
#         self.current_target, = self.ax_main.plot([], [], 'yo', markersize=8, 
#                                                label='当前目标', zorder=4)
        
#         # 初始化控制图表
#         self.control_line_v, = self.ax_control.plot([], [], 'b-', label='线速度 v')
#         self.control_line_w, = self.ax_control.plot([], [], 'r-', label='角速度 ω')
#         self.ax_control.legend(fontsize=8)
#         self.ax_control.grid(True, alpha=0.3)
        
#         # 初始化误差图表
#         self.error_line, = self.ax_error.plot([], [], 'g-', label='跟踪误差')
#         self.ax_error.legend(fontsize=8)
#         self.ax_error.grid(True, alpha=0.3)
        
#         # 初始化速度图表
#         self.velocity_line, = self.ax_velocity.plot([], [], 'purple', label='速度大小')
#         self.ax_velocity.legend(fontsize=8)
#         self.ax_velocity.grid(True, alpha=0.3)
        
#         self.ax_main.legend(loc='upper left', fontsize=10)
        
#     def draw_static_elements(self):
#         """绘制静态环境元素"""
#         # 绘制障碍物
#         for obs_x, obs_y, obs_w, obs_h in self.env.obstacles:
#             rect = Rectangle((obs_x, obs_y), obs_w, obs_h, 
#                            facecolor='red', alpha=0.7, edgecolor='black')
#             self.ax_main.add_patch(rect)
        
#         # 绘制起点和终点
#         self.ax_main.plot(self.env.start.x, self.env.start.y, 'go', markersize=12, 
#                          label='起点', markeredgecolor='black', markeredgewidth=2)
#         self.ax_main.plot(self.env.goal.x, self.env.goal.y, 'rs', markersize=12, 
#                          label='终点', markeredgecolor='black', markeredgewidth=2)
        
#         # 绘制探索的节点（如果有）
#         if hasattr(self.planner, 'visited_nodes') and self.planner.visited_nodes:
#             visited_x = [p.x for p in self.planner.visited_nodes]
#             visited_y = [p.y for p in self.planner.visited_nodes]
#             self.ax_main.plot(visited_x, visited_y, 'c.', markersize=2, alpha=0.3, 
#                              label='探索节点')
        
#         # 绘制规划路径
#         if self.path:
#             path_x = [p.x for p in self.path]
#             path_y = [p.y for p in self.path]
#             self.ax_main.plot(path_x, path_y, 'b-', linewidth=3, alpha=0.6, 
#                              label='规划路径')
#             self.ax_main.plot(path_x, path_y, 'bo', markersize=4, alpha=0.4)
    
#     def animate(self, frame):
#         """动画更新函数"""
#         if not self.is_animating or self.current_target_idx >= len(self.path):
#             return self.get_animation_artists()
        
#         # 获取当前目标点
#         target_point = self.path[self.current_target_idx]
        
#         # 计算控制输入
#         v_cmd, w_cmd = self.controller.compute_control(
#             self.robot.state, target_point, self.dt)
        
#         # 更新机器人状态
#         self.robot.update_state(v_cmd, w_cmd, self.dt)
        
#         # 记录数据
#         self.robot_positions.append((self.robot.state.x, self.robot.state.y))
#         self.robot_orientations.append(self.robot.state.theta)
#         self.animation_times.append(self.time_elapsed)
#         self.real_time_controls.append((v_cmd, w_cmd))
        
#         # 计算跟踪误差
#         current_pos = Point(self.robot.state.x, self.robot.state.y)
#         error = current_pos.distance_to(target_point)
#         self.real_time_errors.append(error)
        
#         # 检查是否到达目标点
#         if error < 3.0:
#             self.current_target_idx += 1
#             if self.current_target_idx < len(self.path):
#                 print(f"到达路径点 {self.current_target_idx-1}/{len(self.path)-1}")
        
#         self.time_elapsed += self.dt
        
#         # 更新可视化
#         self.update_visualization()
        
#         # 检查结束条件
#         if (self.current_target_idx >= len(self.path) or 
#             self.time_elapsed >= self.max_time):
#             self.is_animating = False
#             self.on_animation_complete()
        
#         return self.get_animation_artists()
    
#     def update_visualization(self):
#         """更新可视化元素"""
#         # 更新机器人位置
#         self.robot_point.set_data([self.robot.state.x], [self.robot.state.y])
        
#         # 更新机器人朝向箭头
#         arrow_length = 8
#         dx = arrow_length * np.cos(self.robot.state.theta)
#         dy = arrow_length * np.sin(self.robot.state.theta)
#         self.robot_arrow.set_position((self.robot.state.x, self.robot.state.y))
#         self.robot_arrow.xy = (self.robot.state.x + dx, self.robot.state.y + dy)
        
#         # 更新轨迹线
#         if len(self.robot_positions) > 1:
#             traj_x, traj_y = zip(*self.robot_positions)
#             self.trajectory_line.set_data(traj_x, traj_y)
        
#         # 更新当前目标点
#         if self.current_target_idx < len(self.path):
#             target = self.path[self.current_target_idx]
#             self.current_target.set_data([target.x], [target.y])
        
#         # 更新控制输入图表
#         if len(self.animation_times) > 1:
#             controls = np.array(self.real_time_controls)
#             self.control_line_v.set_data(self.animation_times, controls[:, 0])
#             self.control_line_w.set_data(self.animation_times, controls[:, 1])
            
#             # 自动调整控制图表范围
#             self.ax_control.set_xlim(0, max(self.animation_times))
#             v_range = [min(controls[:, 0]), max(controls[:, 0])]
#             w_range = [min(controls[:, 1]), max(controls[:, 1])]
#             self.ax_control.set_ylim(min(v_range[0], w_range[0]) - 0.1, 
#                                    max(v_range[1], w_range[1]) + 0.1)
        
#         # 更新误差图表
#         if len(self.animation_times) > 1:
#             self.error_line.set_data(self.animation_times, self.real_time_errors)
#             self.ax_error.set_xlim(0, max(self.animation_times))
#             self.ax_error.set_ylim(0, max(self.real_time_errors) + 0.5)
        
#         # 更新速度图表
#         if len(self.animation_times) > 1:
#             velocities = [np.sqrt(v**2 + (w*2)**2) for v, w in self.real_time_controls]
#             self.velocity_line.set_data(self.animation_times, velocities)
#             self.ax_velocity.set_xlim(0, max(self.animation_times))
#             self.ax_velocity.set_ylim(0, max(velocities) + 0.5)
        
#         # 更新状态信息
#         self.ax_info.clear()
#         self.ax_info.axis('off')
#         info_text = f"""
# 仿真状态信息

# 时间: {self.time_elapsed:.1f}s
# 当前位置: ({self.robot.state.x:.1f}, {self.robot.state.y:.1f})
# 朝向角度: {np.degrees(self.robot.state.theta):.1f}°
# 目标点: {self.current_target_idx}/{len(self.path)-1}
# 跟踪误差: {self.real_time_errors[-1]:.2f}m

# 控制输入:
# 线速度: {self.real_time_controls[-1][0]:.2f} m/s
# 角速度: {self.real_time_controls[-1][1]:.2f} rad/s

# 路径规划器: {type(self.planner).__name__}
# 控制器: {type(self.controller).__name__}
#         """
        
#         self.ax_info.text(0.05, 0.95, info_text.strip(), transform=self.ax_info.transAxes,
#                          fontsize=9, verticalalignment='top')
    
#     def get_animation_artists(self):
#         """获取动画艺术家对象"""
#         return [self.robot_point, self.robot_arrow, self.trajectory_line, 
#                 self.current_target, self.control_line_v, self.control_line_w,
#                 self.error_line, self.velocity_line]
    
#     def on_animation_complete(self):
#         """动画完成时的回调"""
#         if self.controller and hasattr(self.controller, 'get_statistics'):
#             control_stats = self.controller.get_statistics()
#             print(f"\n跟踪完成!")
#             print(f"  平均跟踪误差: {control_stats['avg_error']:.2f}m")
#             print(f"  最大跟踪误差: {control_stats['max_error']:.2f}m")
#             print(f"  最终跟踪误差: {control_stats['final_error']:.2f}m")
#             print(f"  仿真时间: {self.time_elapsed:.1f}秒")
    
#     def start_dynamic_simulation(self, planner_type: str = 'astar', 
#                                 controller_type: str = 'pid', 
#                                 animation_speed: float = 50):
#         """启动动态仿真"""
#         print(f"开始动态仿真: {planner_type.upper()} + {controller_type.upper()}")
        
#         # 设置规划器和控制器
#         self.setup_planner(planner_type)
#         self.setup_controller(controller_type)
        
#         # 执行路径规划
#         self.plan_path()
        
#         if not self.path:
#             print("路径规划失败，无法开始仿真")
#             return
        
#         # 初始化动画
#         self.init_animation()
#         self.is_animating = True
        
#         # 创建并启动动画
#         self.animation = FuncAnimation(
#             self.fig, self.animate, 
#             interval=animation_speed,  # 毫秒
#             blit=False,
#             repeat=False,
#             cache_frame_data=False
#         )
        
#         plt.show()
    
    
#     def run_comparison_study(self):
#         """运行对比研究"""
#         print("=" * 80)
#         print("开始路径规划算法对比研究")
#         print("=" * 80)
        
#         planners = ['dijkstra', 'astar', 'rrt']
#         controllers = ['pid', 'lqr', 'mpc']
        
#         results = {}
        
#         # 测试不同规划算法
#         for planner_name in planners:
#             print(f"\n测试规划算法: {planner_name.upper()}")
#             print("-" * 40)
            
#             self.setup_planner(planner_name)
#             self.plan_path()
            
#             if self.path:
#                 planner_stats = self.planner.get_statistics()
#                 results[planner_name] = {'planning': planner_stats, 'tracking': {}}
                
#                 # 测试不同控制器
#                 for controller_name in controllers:
#                     print(f"\n  测试控制器: {controller_name.upper()}")
#                     try:
#                         self.setup_controller(controller_name)
#                         self.simulate_tracking()
                        
#                         control_stats = self.controller.get_statistics()
#                         results[planner_name]['tracking'][controller_name] = control_stats
                        
#                     except Exception as e:
#                         print(f"    控制器测试出错: {e}")
#                         results[planner_name]['tracking'][controller_name] = None
#             else:
#                 results[planner_name] = {'planning': None, 'tracking': {}}
        
#         # 输出对比结果
#         self.print_comparison_results(results)
#         return results

#     # 静态仿真
#     def simulate_tracking(self, dt: float = 0.1, max_time: float = 50.0):
#         """仿真轨迹跟踪"""
#         if not self.path or self.controller is None:
#             raise ValueError("Path or controller not initialized")
        
#         # 初始化机器人
#         initial_state = RobotState(self.env.start.x, self.env.start.y, 0)
#         self.robot = Robot(initial_state)
        
#         print(f"开始使用 {type(self.controller).__name__} 进行轨迹跟踪...")
        
#         current_target_idx = 1  # 跳过起点
#         time_elapsed = 0
        
#         while (current_target_idx < len(self.path) and 
#                time_elapsed < max_time):
            
#             target_point = self.path[current_target_idx]
            
#             # 计算控制输入
#             v_cmd, w_cmd = self.controller.compute_control(
#                 self.robot.state, target_point, dt)
            
#             # 更新机器人状态
#             self.robot.update_state(v_cmd, w_cmd, dt)
            
#             # 检查是否到达目标点
#             distance_to_target = Point(self.robot.state.x, self.robot.state.y).distance_to(target_point)
#             if distance_to_target < 3.0:  # 到达阈值
#                 current_target_idx += 1
#                 print(f"到达路径点 {current_target_idx-1}/{len(self.path)-1}")
            
#             time_elapsed += dt
        
#         # 输出跟踪统计
#         control_stats = self.controller.get_statistics()
#         print(f"跟踪完成!")
#         print(f"  平均跟踪误差: {control_stats['avg_error']:.2f}")
#         print(f"  最大跟踪误差: {control_stats['max_error']:.2f}")
#         print(f"  最终跟踪误差: {control_stats['final_error']:.2f}")
#         print(f"  仿真时间: {time_elapsed:.1f}秒")


#     def print_comparison_results(self, results: dict):
#         """打印对比结果"""
#         print("\n" + "=" * 80)
#         print("算法性能对比结果")
#         print("=" * 80)
        
#         # 规划算法对比
#         print("\n1. 路径规划算法对比:")
#         print("-" * 60)
#         print(f"{'算法':<12} {'路径长度':<12} {'探索节点':<12} {'规划时间':<12} {'成功率'}")
#         print("-" * 60)
        
#         for planner, data in results.items():
#             if data['planning']:
#                 stats = data['planning']
#                 print(f"{planner:<12} {stats['path_length']:<12.2f} "
#                       f"{stats['nodes_explored']:<12} {stats['planning_time']:<12.3f} {'成功'}")
#             else:
#                 print(f"{planner:<12} {'N/A':<12} {'N/A':<12} {'N/A':<12} {'失败'}")
        
#         # 控制算法对比
#         print("\n2. 跟踪控制算法对比:")
#         print("-" * 70)
#         for planner, data in results.items():
#             if data['planning'] and data['tracking']:
#                 print(f"\n{planner.upper()} 规划器的跟踪性能:")
#                 print(f"{'控制器':<12} {'平均误差':<12} {'最大误差':<12} {'最终误差':<12}")
#                 print("-" * 50)
                
#                 for controller, stats in data['tracking'].items():
#                     if stats:
#                         print(f"{controller:<12} {stats['avg_error']:<12.2f} "
#                               f"{stats['max_error']:<12.2f} {stats['final_error']:<12.2f}")
#                     else:
#                         print(f"{controller:<12} {'N/A':<12} {'N/A':<12} {'N/A':<12}")
from simulator.core import PathPlanningSimulator
from simulator.animator import SimulationAnimator
from simulator.visualizer import SimulationVisualizer
from simulator.comparison import ComparisonStudy
import matplotlib.pyplot as plt

class DynamicPathPlanningSimulator:
    """主仿真器接口"""
    def __init__(self, env):
        self.core = PathPlanningSimulator(env)
        self.visualizer = None
        self.animator = None
        
    def start_dynamic_simulation(self, planner_type: str = 'astar', 
                                controller_type: str = 'pid', 
                                animation_speed: float = 50):
        """启动动态仿真"""
        print(f"开始动态仿真: {planner_type.upper()} + {controller_type.upper()}")
        
        # 设置算法
        self.core.setup_planner(planner_type)
        self.core.setup_controller(controller_type)
        
        # 执行路径规划
        self.core.plan_path()
        
        if not self.core.path:
            print("路径规划失败，无法开始仿真")
            return
        
        # 创建动画器并开始仿真
        self.animator = SimulationAnimator(self.core)
        animation = self.animator.start_animation(animation_speed)
        
        plt.show()
    
    def show_static_result(self, planner_type: str = 'astar'):
        """显示静态规划结果"""
        self.core.setup_planner(planner_type)
        self.core.plan_path()
        
        if self.core.path:
            self.visualizer = SimulationVisualizer(self.core)
            self.visualizer.show_static_result()
    
    def run_comparison_study(self):
        """运行对比研究"""
        comparison = ComparisonStudy(self.core)
        return comparison.run_full_comparison()