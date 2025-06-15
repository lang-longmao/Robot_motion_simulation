# class ComparisonStudy:
#     """算法对比研究"""
#     def __init__(self, simulator):
#         self.simulator = simulator
        
#     def run_full_comparison(self):
#         """运行完整对比研究"""
#         print("=" * 80)
#         print("开始路径规划算法对比研究")
#         print("=" * 80)
        
#         planners = ['dijkstra', 'astar', 'rrt']
#         controllers = ['pid', 'lqr', 'mpc']
        
#         results = {}
        
#         for planner_name in planners:
#             print(f"\n测试规划算法: {planner_name.upper()}")
#             print("-" * 40)
            
#             self.simulator.setup_planner(planner_name)
#             self.simulator.plan_path()
            
#             if self.simulator.path:
#                 planner_stats = self.simulator.planner.get_statistics()
#                 results[planner_name] = {'planning': planner_stats, 'tracking': {}}
                
#                 for controller_name in controllers:
#                     print(f"  测试控制器: {controller_name.upper()}")
#                     try:
#                         self.simulator.setup_controller(controller_name)
#                         control_stats = self.simulator.simulate_tracking()
#                         results[planner_name]['tracking'][controller_name] = control_stats
                        
#                     except Exception as e:
#                         print(f"    控制器测试出错: {e}")
#                         results[planner_name]['tracking'][controller_name] = None
#             else:
#                 results[planner_name] = {'planning': None, 'tracking': {}}
        
#         self._print_comparison_results(results)
#         return results
    
#     def _print_comparison_results(self, results: dict):
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
import numpy as np
import matplotlib.pyplot as plt
import time
from typing import Dict, List, Any
import json
from datetime import datetime

class ComparisonStudy:
    """算法对比研究"""
    def __init__(self, simulator):
        self.simulator = simulator
        self.detailed_results = {}
        self.comparison_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
    def run_full_comparison(self, num_runs: int = 3, save_results: bool = True):
        """运行完整对比研究"""
        print("=" * 80)
        print("开始算法性能综合对比研究")
        print("=" * 80)
        
        planners = ['dijkstra', 'astar', 'rrt']
        controllers = ['pid', 'lqr', 'mpc']
        
        results = {}
        
        for planner_name in planners:
            print(f"\n测试规划算法: {planner_name.upper()}")
            print("-" * 40)
            
            planner_results = []
            
            # 多次运行以获得统计平均值
            for run in range(num_runs):
                print(f"  运行 {run+1}/{num_runs}...")
                
                self.simulator.setup_planner(planner_name)
                
                # 记录规划开始时间
                planning_start = time.time()
                self.simulator.plan_path()
                planning_end = time.time()
                
                if self.simulator.path:
                    planner_stats = self.simulator.planner.get_statistics()
                    
                    # 规划指标
                    enhanced_planning_stats = self._calculate_enhanced_planning_metrics(
                        planner_stats, planning_end - planning_start)
                    
                    run_result = {
                        'planning': enhanced_planning_stats,
                        'tracking': {}
                    }
                    
                    # 测试每个控制器
                    for controller_name in controllers:
                        print(f"    测试控制器: {controller_name.upper()}")
                        try:
                            self.simulator.setup_controller(controller_name)
                            
                            # 记录跟踪开始时间
                            tracking_start = time.time()
                            control_stats = self.simulator.simulate_tracking()
                            tracking_end = time.time()
                            
                            # 增强的跟踪指标
                            enhanced_control_stats = self._calculate_enhanced_tracking_metrics(
                                control_stats, tracking_end - tracking_start)
                            
                            run_result['tracking'][controller_name] = enhanced_control_stats
                            
                        except Exception as e:
                            print(f"      控制器测试出错: {e}")
                            run_result['tracking'][controller_name] = None
                    
                    planner_results.append(run_result)
                else:
                    print(f"    运行 {run+1} 规划失败")
            
            # 计算多次运行的统计结果
            if planner_results:
                results[planner_name] = self._aggregate_multiple_runs(planner_results)
            else:
                results[planner_name] = {'planning': None, 'tracking': {}}
        
        self.detailed_results = results
        
        # 打印详细对比结果
        self._print_enhanced_comparison_results(results)
        
        # 生成可视化图表
        self._generate_comparison_charts(results)
        
        # 保存结果
        if save_results:
            self._save_results_to_file(results)
        
        return results
    
    def _calculate_enhanced_planning_metrics(self, basic_stats: Dict, actual_time: float) -> Dict:
        """计算规划指标"""
        enhanced = basic_stats.copy()
        
        # 路径质量指标
        path = self.simulator.path
        if path and len(path) > 1:
            # 路径平滑度 (角度变化的标准差)
            angles = []
            for i in range(1, len(path) - 1):
                v1 = np.array([path[i].x - path[i-1].x, path[i].y - path[i-1].y])
                v2 = np.array([path[i+1].x - path[i].x, path[i+1].y - path[i].y])
                
                # 计算角度变化
                if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                    cos_angle = np.clip(cos_angle, -1, 1)
                    angle = np.arccos(cos_angle)
                    angles.append(angle)
            
            enhanced['path_smoothness'] = np.std(angles) if angles else 0
            enhanced['max_turn_angle'] = np.max(angles) if angles else 0
            enhanced['avg_turn_angle'] = np.mean(angles) if angles else 0
            
            # 路径效率 (直线距离比)
            start_point = path[0]
            end_point = path[-1]
            straight_line_distance = start_point.distance_to(end_point)
            enhanced['path_efficiency'] = straight_line_distance / enhanced['path_length'] if enhanced['path_length'] > 0 else 0
            
            # 计算路径复杂度 (转折点数量)
            turn_points = 0
            threshold = np.pi / 6  # 30度阈值
            for angle in angles:
                if angle > threshold:
                    turn_points += 1
            enhanced['turn_points'] = turn_points
            
        # 算法效率指标
        enhanced['actual_planning_time'] = actual_time
        enhanced['nodes_per_second'] = enhanced['nodes_explored'] / actual_time if actual_time > 0 else 0
        
        # 内存效率 (假设每个节点占用固定内存)
        enhanced['estimated_memory_mb'] = enhanced['nodes_explored'] * 0.1  # 假设每节点100KB
        
        return enhanced
    
    def _calculate_enhanced_tracking_metrics(self, basic_stats: Dict, actual_time: float) -> Dict:
        """计算增强的跟踪指标"""
        enhanced = basic_stats.copy()
        
        # 从控制器获取详细数据
        if hasattr(self.simulator.controller, 'error_history'):
            errors = self.simulator.controller.error_history
            controls = self.simulator.controller.control_history
            
            # 误差统计指标
            enhanced['error_std'] = np.std(errors)
            enhanced['error_variance'] = np.var(errors)
            enhanced['rmse'] = np.sqrt(np.mean(np.square(errors)))
            
            # 控制性能指标
            if controls:
                v_commands = [c[0] for c in controls]
                w_commands = [c[1] for c in controls]
                
                enhanced['avg_linear_velocity'] = np.mean(np.abs(v_commands))
                enhanced['avg_angular_velocity'] = np.mean(np.abs(w_commands))
                enhanced['max_linear_velocity'] = np.max(np.abs(v_commands))
                enhanced['max_angular_velocity'] = np.max(np.abs(w_commands))
                
                # 控制平滑度 (命令变化率)
                v_changes = np.diff(v_commands)
                w_changes = np.diff(w_commands)
                enhanced['control_smoothness_v'] = np.std(v_changes)
                enhanced['control_smoothness_w'] = np.std(w_changes)
                
                # 能耗估计 (基于速度命令)
                energy_consumption = sum(abs(v) + abs(w) * 0.5 for v, w in controls)
                enhanced['estimated_energy'] = energy_consumption
        
        # 时间效率指标
        enhanced['actual_tracking_time'] = actual_time
        enhanced['real_time_factor'] = enhanced.get('simulation_time', actual_time) / actual_time
        
        # 稳定性指标
        if 'convergence_time' not in enhanced:
            enhanced['convergence_time'] = actual_time * 0.8  # 估算值
        
        enhanced['overshoot_percentage'] = max(0, (enhanced['max_error'] - enhanced['final_error']) / enhanced['final_error'] * 100) if enhanced.get('final_error', 0) > 0 else 0
        
        return enhanced
    
    def _aggregate_multiple_runs(self, runs: List[Dict]) -> Dict:
        """聚合多次运行的结果"""
        if not runs:
            return {'planning': None, 'tracking': {}}
        
        # 聚合规划结果
        planning_metrics = {}
        for metric in runs[0]['planning'].keys():
            values = [run['planning'][metric] for run in runs if run['planning'][metric] is not None]
            if values:
                planning_metrics[metric] = {
                    'mean': np.mean(values),
                    'std': np.std(values),
                    'min': np.min(values),
                    'max': np.max(values),
                    'values': values
                }
        
        # 聚合跟踪结果
        tracking_results = {}
        controller_names = runs[0]['tracking'].keys()
        
        for controller in controller_names:
            controller_results = [run['tracking'][controller] for run in runs 
                                if run['tracking'][controller] is not None]
            
            if controller_results:
                controller_metrics = {}
                for metric in controller_results[0].keys():
                    values = [result[metric] for result in controller_results 
                            if result[metric] is not None]
                    if values:
                        controller_metrics[metric] = {
                            'mean': np.mean(values),
                            'std': np.std(values),
                            'min': np.min(values),
                            'max': np.max(values),
                            'values': values
                        }
                tracking_results[controller] = controller_metrics
            else:
                tracking_results[controller] = None
        
        return {
            'planning': planning_metrics,
            'tracking': tracking_results,
            'num_successful_runs': len(runs)
        }
    
    def _print_enhanced_comparison_results(self, results: Dict):
        """打印增强的对比结果"""
        print("\n" + "=" * 100)
        print("算法性能综合对比结果")
        print("=" * 100)
        
        # 1. 规划算法详细对比
        print("\n1. 路径规划算法详细对比:")
        print("-" * 100)
        header = f"{'算法':<12} {'路径长度':<12} {'路径效率':<12} {'平滑度':<12} {'规划时间':<12} {'探索效率':<12}"
        print(header)
        print("-" * 100)
        
        for planner, data in results.items():
            if data['planning']:
                p = data['planning']
                efficiency = p.get('path_efficiency', {}).get('mean', 0)
                smoothness = p.get('path_smoothness', {}).get('mean', 0)
                planning_time = p.get('actual_planning_time', {}).get('mean', 0)
                nodes_per_sec = p.get('nodes_per_second', {}).get('mean', 0)
                path_length = p.get('path_length', {}).get('mean', 0)
                
                print(f"{planner:<12} {path_length:<12.2f} {efficiency:<12.3f} "
                      f"{smoothness:<12.3f} {planning_time:<12.3f} {nodes_per_sec:<12.1f}")
            else:
                print(f"{planner:<12} {'失败':<12} {'N/A':<12} {'N/A':<12} {'N/A':<12} {'N/A':<12}")
        
        # 2. 跟踪控制详细对比
        print("\n2. 跟踪控制算法详细对比:")
        print("-" * 120)
        
        for planner, data in results.items():
            if data['planning'] and data['tracking']:
                print(f"\n{planner.upper()} 规划器的跟踪性能:")
                header = f"{'控制器':<12} {'平均误差':<12} {'RMSE':<12} {'最大误差':<12} {'收敛时间':<12} {'能耗估计':<12} {'控制平滑':<12}"
                print(header)
                print("-" * 120)
                
                for controller, stats in data['tracking'].items():
                    if stats:
                        avg_error = stats.get('avg_error', {}).get('mean', 0)
                        rmse = stats.get('rmse', {}).get('mean', 0)
                        max_error = stats.get('max_error', {}).get('mean', 0)
                        conv_time = stats.get('convergence_time', {}).get('mean', 0)
                        energy = stats.get('estimated_energy', {}).get('mean', 0)
                        smoothness = stats.get('control_smoothness_v', {}).get('mean', 0)
                        
                        print(f"{controller:<12} {avg_error:<12.2f} {rmse:<12.2f} "
                              f"{max_error:<12.2f} {conv_time:<12.2f} {energy:<12.1f} {smoothness:<12.3f}")
                    else:
                        print(f"{controller:<12} {'N/A':<12} {'N/A':<12} {'N/A':<12} {'N/A':<12} {'N/A':<12} {'N/A':<12}")
        
        # 3. 综合性能排名
        print("\n3. 综合性能排名:")
        print("-" * 60)
        self._print_performance_ranking(results)
        
        # 4. 算法适用场景建议
        print("\n4. 算法适用场景建议:")
        print("-" * 60)
        self._print_usage_recommendations(results)
    
    def _print_performance_ranking(self, results: Dict):
        """打印性能排名"""
        scores = {}
        
        for planner, data in results.items():
            if data['planning']:
                # 规划性能评分 (0-100)
                planning_score = self._calculate_planning_score(data['planning'])
                
                # 跟踪性能评分
                tracking_scores = []
                for controller, stats in data['tracking'].items():
                    if stats:
                        tracking_score = self._calculate_tracking_score(stats)
                        tracking_scores.append(tracking_score)
                
                avg_tracking_score = np.mean(tracking_scores) if tracking_scores else 0
                overall_score = (planning_score * 0.4 + avg_tracking_score * 0.6)
                
                scores[planner] = {
                    'planning': planning_score,
                    'tracking': avg_tracking_score,
                    'overall': overall_score
                }
        
        # 排序并显示
        sorted_planners = sorted(scores.items(), key=lambda x: x[1]['overall'], reverse=True)
        
        print(f"{'排名':<6} {'算法':<12} {'规划评分':<12} {'跟踪评分':<12} {'综合评分':<12}")
        print("-" * 60)
        
        for i, (planner, score) in enumerate(sorted_planners, 1):
            print(f"{i:<6} {planner:<12} {score['planning']:<12.1f} "
                  f"{score['tracking']:<12.1f} {score['overall']:<12.1f}")
    
    def _calculate_planning_score(self, planning_stats: Dict) -> float:
        """计算规划算法评分"""
        # 基于多个指标的综合评分
        efficiency = planning_stats.get('path_efficiency', {}).get('mean', 0)
        time_score = min(100, 10 / max(0.1, planning_stats.get('actual_planning_time', {}).get('mean', 1)))
        smoothness_score = max(0, 100 - planning_stats.get('path_smoothness', {}).get('mean', 0) * 50)
        
        return (efficiency * 40 + time_score * 30 + smoothness_score * 30)
    
    def _calculate_tracking_score(self, tracking_stats: Dict) -> float:
        """计算跟踪算法评分"""
        # 基于误差和稳定性的评分
        error_score = max(0, 100 - tracking_stats.get('avg_error', {}).get('mean', 0) * 10)
        stability_score = max(0, 100 - tracking_stats.get('error_std', {}).get('mean', 0) * 20)
        efficiency_score = min(100, 100 / max(1, tracking_stats.get('estimated_energy', {}).get('mean', 1)))
        
        return (error_score * 50 + stability_score * 30 + efficiency_score * 20)
    
    def _print_usage_recommendations(self, results: Dict):
        """打印使用建议"""
        recommendations = {
            'dijkstra': "适用于静态环境，需要最优解的场景",
            'astar': "适用于大多数场景，平衡了速度和最优性",
            'rrt': "适用于复杂环境和高维空间规划",
            'pid': "适用于简单跟踪任务，实现简单",
            'lqr': "适用于需要最优控制的线性系统",
            'mpc': "适用于有约束的复杂控制任务"
        }
        
        # 基于性能结果给出具体建议
        for planner, rec in recommendations.items():
            if planner in results:
                print(f"{planner.upper():<12}: {rec}")
    
    def _generate_comparison_charts(self, results: Dict):
        """生成对比图表"""
        try:
            fig, axes = plt.subplots(2, 3, figsize=(18, 12))
            fig.suptitle('算法性能对比图表', fontsize=16, fontweight='bold')
            
            # 1. 规划时间对比
            self._plot_planning_time_comparison(axes[0, 0], results)
            
            # 2. 路径质量对比
            self._plot_path_quality_comparison(axes[0, 1], results)
            
            # 3. 跟踪误差对比
            self._plot_tracking_error_comparison(axes[0, 2], results)
            
            # 4. 算法效率对比
            self._plot_algorithm_efficiency_comparison(axes[1, 0], results)
            
            # 5. 能耗估计对比图 (替换原来的雷达图)
            self._plot_performance_radar(axes[1, 1], results)
            
            # 6. 成功率和稳定性
            self._plot_success_rate_stability(axes[1, 2], results)
            
            plt.tight_layout()
            
            # 保存图表
            chart_filename = f"comparison_charts_{self.comparison_timestamp}.png"
            plt.savefig(chart_filename, dpi=300, bbox_inches='tight')
            print(f"\n对比图表已保存为: {chart_filename}")
            
            plt.show()
            
        except Exception as e:
            print(f"生成图表时出错: {e}")
    
    def _plot_planning_time_comparison(self, ax, results):
        """绘制规划时间对比"""
        planners = []
        times = []
        errors = []
        
        for planner, data in results.items():
            if data['planning']:
                planners.append(planner.upper())
                time_stats = data['planning'].get('actual_planning_time', {})
                times.append(time_stats.get('mean', 0))
                errors.append(time_stats.get('std', 0))
        
        bars = ax.bar(planners, times, yerr=errors, capsize=5, alpha=0.7)
        ax.set_title('规划时间对比')
        ax.set_ylabel('时间 (秒)')
        ax.grid(True, alpha=0.3)
        
        # 添加数值标签
        for bar, time in zip(bars, times):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                   f'{time:.3f}s', ha='center', va='bottom')
    
    def _plot_path_quality_comparison(self, ax, results):
        """绘制路径质量对比"""
        planners = []
        efficiencies = []
        lengths = []
        
        for planner, data in results.items():
            if data['planning']:
                planners.append(planner.upper())
                eff_stats = data['planning'].get('path_efficiency', {})
                len_stats = data['planning'].get('path_length', {})
                efficiencies.append(eff_stats.get('mean', 0))
                lengths.append(len_stats.get('mean', 0))
        
        ax2 = ax.twinx()
        
        bars1 = ax.bar([p + ' (效率)' for p in planners], efficiencies, 
                      alpha=0.7, color='blue', label='路径效率')
        bars2 = ax2.bar([p + ' (长度)' for p in planners], lengths, 
                       alpha=0.7, color='red', label='路径长度')
        
        ax.set_title('路径质量对比')
        ax.set_ylabel('路径效率', color='blue')
        ax2.set_ylabel('路径长度', color='red')
        ax.tick_params(axis='x', rotation=45)
    
    def _plot_tracking_error_comparison(self, ax, results):
        """绘制跟踪误差对比"""
        controllers = ['pid', 'lqr', 'mpc']
        planners = list(results.keys())
        
        x = np.arange(len(controllers))
        width = 0.25
        
        for i, planner in enumerate(planners):
            if results[planner]['planning']:
                errors = []
                for controller in controllers:
                    tracking_data = results[planner]['tracking'].get(controller)
                    if tracking_data:
                        error = tracking_data.get('avg_error', {}).get('mean', 0)
                        errors.append(error)
                    else:
                        errors.append(0)
                
                ax.bar(x + i * width, errors, width, label=planner.upper(), alpha=0.7)
        
        ax.set_title('跟踪误差对比')
        ax.set_ylabel('平均误差 (m)')
        ax.set_xlabel('控制器')
        ax.set_xticks(x + width)
        ax.set_xticklabels([c.upper() for c in controllers])
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def _plot_algorithm_efficiency_comparison(self, ax, results):
        """绘制算法效率对比"""
        planners = []
        nodes_per_sec = []
        memory_usage = []
        
        for planner, data in results.items():
            if data['planning']:
                planners.append(planner.upper())
                nps_stats = data['planning'].get('nodes_per_second', {})
                mem_stats = data['planning'].get('estimated_memory_mb', {})
                nodes_per_sec.append(nps_stats.get('mean', 0))
                memory_usage.append(mem_stats.get('mean', 0))
        
        ax2 = ax.twinx()
        
        line1 = ax.plot(planners, nodes_per_sec, 'bo-', label='探索效率', linewidth=2)
        line2 = ax2.plot(planners, memory_usage, 'rs-', label='内存使用', linewidth=2)
        
        ax.set_title('算法效率对比')
        ax.set_ylabel('节点/秒', color='blue')
        ax2.set_ylabel('内存使用 (MB)', color='red')
        ax.grid(True, alpha=0.3)
        
        # 合并图例
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, loc='upper left')
    
    def _plot_performance_radar(self, ax, results):
        """绘制能耗估计对比图"""
        # 收集能耗数据
        planners = []
        controllers = ['pid', 'lqr', 'mpc']
        energy_data = {controller: [] for controller in controllers}
        
        for planner, data in results.items():
            if data['planning'] and data['tracking']:
                planners.append(planner.upper())
                
                for controller in controllers:
                    tracking_data = data['tracking'].get(controller)
                    if tracking_data and 'estimated_energy' in tracking_data:
                        energy = tracking_data['estimated_energy'].get('mean', 0)
                        energy_data[controller].append(energy)
                    else:
                        energy_data[controller].append(0)
        
        if not planners:
            ax.text(0.5, 0.5, '无可用的能耗数据', ha='center', va='center', 
                transform=ax.transAxes, fontsize=12)
            ax.set_title('能耗估计对比')
            return
        
        # 绘制分组柱状图
        x = np.arange(len(planners))
        width = 0.25
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # 蓝色、橙色、绿色
        
        for i, (controller, energies) in enumerate(energy_data.items()):
            bars = ax.bar(x + i * width, energies, width, 
                        label=controller.upper(), alpha=0.8, color=colors[i])
            
            # 添加数值标签
            for bar, energy in zip(bars, energies):
                if energy > 0:
                    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(energies) * 0.01,
                        f'{energy:.1f}', ha='center', va='bottom', fontsize=8)
        
        ax.set_title('不同算法组合的能耗估计对比')
        ax.set_xlabel('规划算法')
        ax.set_ylabel('估计能耗 (相对单位)')
        ax.set_xticks(x + width)
        ax.set_xticklabels(planners)
        ax.legend(title='控制器', fontsize=8)
        ax.grid(True, alpha=0.3, axis='y')
        
        # 设置y轴范围，确保所有柱子都可见
        if any(energy_data.values()):
            max_energy = max(max(energies) for energies in energy_data.values() if energies)
            ax.set_ylim(0, max_energy * 1.15)
        
        # 添加平均能耗线
        if planners:
            avg_energies = []
            for i, planner in enumerate(planners):
                planner_avg = np.mean([energy_data[controller][i] for controller in controllers 
                                    if energy_data[controller][i] > 0])
                if not np.isnan(planner_avg):
                    avg_energies.append(planner_avg)
                else:
                    avg_energies.append(0)
            
            if avg_energies:
                ax2 = ax.twinx()
                line = ax2.plot(x + width, avg_energies, 'ro-', linewidth=2, 
                            markersize=6, label='平均能耗', alpha=0.8)
                ax2.set_ylabel('平均能耗', color='red')
                ax2.tick_params(axis='y', labelcolor='red')
                
                # 添加平均能耗的数值标签
                for i, avg_energy in enumerate(avg_energies):
                    if avg_energy > 0:
                        ax2.text(x[i] + width, avg_energy + max(avg_energies) * 0.02,
                            f'{avg_energy:.1f}', ha='center', va='bottom', 
                            fontsize=8, color='red', weight='bold')
    
    def _plot_success_rate_stability(self, ax, results):
        """绘制成功率和稳定性"""
        planners = []
        success_rates = []
        stabilities = []
        
        for planner, data in results.items():
            if data['planning']:
                planners.append(planner.upper())
                success_rates.append(100)  # 假设规划成功
                
                # 计算稳定性 (基于标准差)
                avg_std = 0
                count = 0
                for controller, tracking in data['tracking'].items():
                    if tracking and 'error_std' in tracking:
                        avg_std += tracking['error_std'].get('mean', 0)
                        count += 1
                
                stability = 100 - (avg_std / count * 10) if count > 0 else 50
                stabilities.append(max(0, stability))
        
        x = np.arange(len(planners))
        width = 0.35
        
        bars1 = ax.bar(x - width/2, success_rates, width, label='成功率 (%)', alpha=0.7)
        bars2 = ax.bar(x + width/2, stabilities, width, label='稳定性评分', alpha=0.7)
        
        ax.set_title('成功率与稳定性')
        ax.set_ylabel('评分')
        ax.set_xlabel('算法')
        ax.set_xticks(x)
        ax.set_xticklabels(planners)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 105)
    
    def _save_results_to_file(self, results: Dict):
        """保存结果到文件"""
        try:
            # 保存为JSON文件
            json_filename = f"comparison_results_{self.comparison_timestamp}.json"
            
            # 转换numpy数组为列表以便JSON序列化
            serializable_results = self._make_json_serializable(results)
            
            with open(json_filename, 'w', encoding='utf-8') as f:
                json.dump({
                    'timestamp': self.comparison_timestamp,
                    'results': serializable_results,
                    'metadata': {
                        'comparison_version': '2.0',
                        'metrics_included': [
                            'path_length', 'path_efficiency', 'path_smoothness',
                            'planning_time', 'nodes_explored', 'tracking_error',
                            'control_smoothness', 'energy_consumption'
                        ]
                    }
                }, f, indent=2, ensure_ascii=False)
            
            print(f"详细结果已保存为: {json_filename}")
            
        except Exception as e:
            print(f"保存结果时出错: {e}")
    
    def _make_json_serializable(self, obj):
        """使对象可JSON序列化"""
        if isinstance(obj, dict):
            return {k: self._make_json_serializable(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self._make_json_serializable(item) for item in obj]
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.integer, np.floating)):
            return float(obj)
        else:
            return obj
    
    def load_and_compare_results(self, filename: str):
        """加载并对比历史结果"""
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                historical_data = json.load(f)
            
            print(f"成功加载历史对比数据: {historical_data['timestamp']}")
            return historical_data['results']
            
        except Exception as e:
            print(f"加载历史数据时出错: {e}")
            return None