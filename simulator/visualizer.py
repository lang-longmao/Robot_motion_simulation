import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class SimulationVisualizer:
    """仿真可视化器"""
    def __init__(self, simulator):
        self.simulator = simulator
        self.fig = None
        self.setup_figure()
        
    def setup_figure(self):
        """设置图形界面"""
        self.fig = plt.figure(figsize=(18, 10))
        
        gs = self.fig.add_gridspec(2, 3, height_ratios=[2, 1], width_ratios=[2, 1, 1])
        
        self.ax_main = self.fig.add_subplot(gs[0, :2])
        self.ax_main.set_title('实时机器人运动仿真', fontsize=16, fontweight='bold')
        
        self.ax_control = self.fig.add_subplot(gs[0, 2])
        self.ax_control.set_title('实时控制输入', fontsize=12, fontweight='bold')
        
        self.ax_error = self.fig.add_subplot(gs[1, 0])
        self.ax_error.set_title('跟踪误差', fontsize=12, fontweight='bold')
        
        self.ax_velocity = self.fig.add_subplot(gs[1, 1])
        self.ax_velocity.set_title('速度曲线', fontsize=12, fontweight='bold')
        
        self.ax_info = self.fig.add_subplot(gs[1, 2])
        self.ax_info.set_title('状态信息', fontsize=12, fontweight='bold')
        self.ax_info.axis('off')
        
        plt.tight_layout()
        
    def draw_static_elements(self):
        """绘制静态环境元素"""
        env = self.simulator.env
        path = self.simulator.path
        planner = self.simulator.planner
        
        # 绘制障碍物
        for obs_x, obs_y, obs_w, obs_h in env.obstacles:
            rect = Rectangle((obs_x, obs_y), obs_w, obs_h, 
                           facecolor='red', alpha=0.7, edgecolor='black')
            self.ax_main.add_patch(rect)
        
        # 绘制起点和终点
        self.ax_main.plot(env.start.x, env.start.y, 'go', markersize=12, 
                         label='起点', markeredgecolor='black', markeredgewidth=2)
        self.ax_main.plot(env.goal.x, env.goal.y, 'rs', markersize=12, 
                         label='终点', markeredgecolor='black', markeredgewidth=2)
        
        # 绘制探索节点
        if hasattr(planner, 'visited_nodes') and planner.visited_nodes:
            visited_x = [p.x for p in planner.visited_nodes]
            visited_y = [p.y for p in planner.visited_nodes]
            self.ax_main.plot(visited_x, visited_y, 'c.', markersize=2, alpha=0.3, 
                             label='探索节点')
        
        # 绘制规划路径
        if path:
            path_x = [p.x for p in path]
            path_y = [p.y for p in path]
            self.ax_main.plot(path_x, path_y, 'b-', linewidth=3, alpha=0.6, 
                             label='规划路径')
            self.ax_main.plot(path_x, path_y, 'bo', markersize=4, alpha=0.4)
    
    def show_static_result(self):
        """显示静态规划结果"""
        self.ax_main.clear()
        self.ax_main.set_xlim(0, self.simulator.env.width)
        self.ax_main.set_ylim(0, self.simulator.env.height)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        
        self.draw_static_elements()
        self.ax_main.legend()
        plt.show()