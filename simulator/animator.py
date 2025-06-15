from .visualizer import SimulationVisualizer
from base import RobotState, Robot, Point
import numpy as np

class SimulationAnimator(SimulationVisualizer):
    """仿真动画控制器"""
    def __init__(self, simulator):
        super().__init__(simulator)
        self.animation = None
        self.is_animating = False
        self.current_target_idx = 1
        self.time_elapsed = 0
        self.dt = 0.05
        self.max_time = 50.0
        
        # 动画数据存储
        self.robot_positions = []
        self.robot_orientations = []
        self.animation_times = []
        self.real_time_errors = []
        self.real_time_controls = []
        
    def init_animation(self):
        """初始化动画"""
        # 初始化机器人
        initial_state = RobotState(self.simulator.env.start.x, self.simulator.env.start.y, 0)
        self.simulator.robot = Robot(initial_state)
        
        # 重置参数
        self.current_target_idx = 1
        self.time_elapsed = 0
        self.robot_positions = [(self.simulator.robot.state.x, self.simulator.robot.state.y)]
        self.robot_orientations = [self.simulator.robot.state.theta]
        self.animation_times = [0]
        self.real_time_errors = [0]
        self.real_time_controls = [(0, 0)]
        
        # 设置图形
        self.ax_main.clear()
        self.ax_main.set_xlim(0, self.simulator.env.width)
        self.ax_main.set_ylim(0, self.simulator.env.height)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        
        self.draw_static_elements()
        self._init_dynamic_elements()
        
    def _init_dynamic_elements(self):
        """初始化动态元素"""
        self.robot_point, = self.ax_main.plot([], [], 'ko', markersize=10, zorder=5)
        self.robot_arrow = self.ax_main.annotate('', xy=(0, 0), xytext=(0, 0),
                                               arrowprops=dict(arrowstyle='->', 
                                                             color='black', lw=2),
                                               zorder=5)
        self.trajectory_line, = self.ax_main.plot([], [], 'm-', linewidth=2, 
                                                alpha=0.8, label='实际轨迹', zorder=4)
        self.current_target, = self.ax_main.plot([], [], 'yo', markersize=8, 
                                               label='当前目标', zorder=4)
        
        # 初始化图表
        self.control_line_v, = self.ax_control.plot([], [], 'b-', label='线速度 v')
        self.control_line_w, = self.ax_control.plot([], [], 'r-', label='角速度 ω')
        self.ax_control.legend(fontsize=8)
        self.ax_control.grid(True, alpha=0.3)
        
        self.error_line, = self.ax_error.plot([], [], 'g-', label='跟踪误差')
        self.ax_error.legend(fontsize=8)
        self.ax_error.grid(True, alpha=0.3)
        
        self.velocity_line, = self.ax_velocity.plot([], [], 'purple', label='速度大小')
        self.ax_velocity.legend(fontsize=8)
        self.ax_velocity.grid(True, alpha=0.3)
        
        self.ax_main.legend(loc='upper left', fontsize=10)
    
    def animate(self, frame):
        """动画更新函数"""
        if not self.is_animating or self.current_target_idx >= len(self.simulator.path):
            return self._get_animation_artists()
        
        # 获取当前目标点并计算控制
        target_point = self.simulator.path[self.current_target_idx]
        v_cmd, w_cmd = self.simulator.controller.compute_control(
            self.simulator.robot.state, target_point, self.dt)
        
        # 更新机器人状态
        self.simulator.robot.update_state(v_cmd, w_cmd, self.dt)
        
        # 记录数据
        self._record_frame_data(v_cmd, w_cmd, target_point)
        
        # 检查目标点到达
        current_pos = Point(self.simulator.robot.state.x, self.simulator.robot.state.y)
        error = current_pos.distance_to(target_point)
        
        if error < 3.0:
            self.current_target_idx += 1
            if self.current_target_idx < len(self.simulator.path):
                print(f"到达路径点 {self.current_target_idx-1}/{len(self.simulator.path)-1}")
        
        self.time_elapsed += self.dt
        
        # 更新可视化
        self._update_visualization()
        
        # 检查结束条件
        if (self.current_target_idx >= len(self.simulator.path) or 
            self.time_elapsed >= self.max_time):
            self.is_animating = False
            self._on_animation_complete()
        
        return self._get_animation_artists()
    
    def _record_frame_data(self, v_cmd, w_cmd, target_point):
        """记录帧数据"""
        self.robot_positions.append((self.simulator.robot.state.x, self.simulator.robot.state.y))
        self.robot_orientations.append(self.simulator.robot.state.theta)
        self.animation_times.append(self.time_elapsed)
        self.real_time_controls.append((v_cmd, w_cmd))
        
        current_pos = Point(self.simulator.robot.state.x, self.simulator.robot.state.y)
        error = current_pos.distance_to(target_point)
        self.real_time_errors.append(error)
    
    def _update_visualization(self):
        """更新可视化元素"""
        robot = self.simulator.robot
        
        # 更新机器人
        self.robot_point.set_data([robot.state.x], [robot.state.y])
        
        arrow_length = 8
        dx = arrow_length * np.cos(robot.state.theta)
        dy = arrow_length * np.sin(robot.state.theta)
        self.robot_arrow.set_position((robot.state.x, robot.state.y))
        self.robot_arrow.xy = (robot.state.x + dx, robot.state.y + dy)
        
        # 更新轨迹
        if len(self.robot_positions) > 1:
            traj_x, traj_y = zip(*self.robot_positions)
            self.trajectory_line.set_data(traj_x, traj_y)
        
        # 更新目标点
        if self.current_target_idx < len(self.simulator.path):
            target = self.simulator.path[self.current_target_idx]
            self.current_target.set_data([target.x], [target.y])
        
        # 更新图表
        self._update_charts()
        self._update_info_panel()
    
    def _update_charts(self):
        """更新图表"""
        if len(self.animation_times) > 1:
            # 控制输入图表
            controls = np.array(self.real_time_controls)
            self.control_line_v.set_data(self.animation_times, controls[:, 0])
            self.control_line_w.set_data(self.animation_times, controls[:, 1])
            
            self.ax_control.set_xlim(0, max(self.animation_times))
            v_range = [min(controls[:, 0]), max(controls[:, 0])]
            w_range = [min(controls[:, 1]), max(controls[:, 1])]
            self.ax_control.set_ylim(min(v_range[0], w_range[0]) - 0.1, 
                                   max(v_range[1], w_range[1]) + 0.1)
            
            # 误差图表
            self.error_line.set_data(self.animation_times, self.real_time_errors)
            self.ax_error.set_xlim(0, max(self.animation_times))
            self.ax_error.set_ylim(0, max(self.real_time_errors) + 0.5)
            
            # 速度图表
            velocities = [np.sqrt(v**2 + (w*2)**2) for v, w in self.real_time_controls]
            self.velocity_line.set_data(self.animation_times, velocities)
            self.ax_velocity.set_xlim(0, max(self.animation_times))
            self.ax_velocity.set_ylim(0, max(velocities) + 0.5)
    
    def _update_info_panel(self):
        """更新信息面板"""
        self.ax_info.clear()
        self.ax_info.axis('off')
        
        info_text = f"""仿真状态信息

时间: {self.time_elapsed:.1f}s
位置: ({self.simulator.robot.state.x:.1f}, {self.simulator.robot.state.y:.1f})
角度: {np.degrees(self.simulator.robot.state.theta):.1f}°
目标: {self.current_target_idx}/{len(self.simulator.path)-1}
误差: {self.real_time_errors[-1]:.2f}m

控制输入:
v: {self.real_time_controls[-1][0]:.2f} m/s
ω: {self.real_time_controls[-1][1]:.2f} rad/s

算法: {type(self.simulator.planner).__name__}
控制: {type(self.simulator.controller).__name__}"""
        
        self.ax_info.text(0.05, 0.95, info_text.strip(), transform=self.ax_info.transAxes,
                         fontsize=9, verticalalignment='top')
    
    def _get_animation_artists(self):
        """获取动画对象"""
        return [self.robot_point, self.robot_arrow, self.trajectory_line, 
                self.current_target, self.control_line_v, self.control_line_w,
                self.error_line, self.velocity_line]
    
    def _on_animation_complete(self):
        """动画完成回调"""
        if hasattr(self.simulator.controller, 'get_statistics'):
            stats = self.simulator.controller.get_statistics()
            print(f"仿真完成! 平均误差: {stats['avg_error']:.2f}m")
    
    def start_animation(self, animation_speed: float = 50):
        """启动动画"""
        self.init_animation()
        self.is_animating = True
        
        from matplotlib.animation import FuncAnimation
        self.animation = FuncAnimation(
            self.fig, self.animate, 
            interval=animation_speed,
            blit=False,
            repeat=False,
            cache_frame_data=False
        )
        
        return self.animation