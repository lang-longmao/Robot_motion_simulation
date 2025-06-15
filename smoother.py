from base import Point, Environment
import numpy as np
from scipy.interpolate import splprep, splev

class PathSmoother:
    """路径平滑处理器"""
    
    @staticmethod
    def smooth_path_gradient_descent(path, environment: Environment, iterations=1000, alpha=0.9, 
                                   beta=0.3, tolerance=0.1):
        """基于梯度下降的路径平滑算法"""
        if len(path) <= 2:
            return path
        
        # 转换为numpy数组
        smoothed_path = np.array([[p.x, p.y] for p in path])
        original_path = smoothed_path.copy()
        
        for _ in range(iterations):
            new_path = smoothed_path.copy()
            
            # 平滑项：使路径更加平滑
            for i in range(1, len(smoothed_path) - 1):
                # 计算平滑梯度
                smooth_gradient = (2 * smoothed_path[i] - 
                                 smoothed_path[i-1] - smoothed_path[i+1])
                
                # 数据项：保持与原路径的相似性
                data_gradient = smoothed_path[i] - original_path[i]
                
                # 更新路径点
                new_path[i] = (smoothed_path[i] - 
                              alpha * smooth_gradient - 
                              beta * data_gradient)
                
                # 检查碰撞约束
                if not environment.is_collision_true(Point(new_path[i][0], new_path[i][1])):
                    smoothed_path[i] = new_path[i]
            
            # 检查收敛性
            if np.linalg.norm(new_path - smoothed_path) < tolerance:
                break
        
        # 转换回Point格式
        print("平滑后的路径:", smoothed_path)
        return [Point(p[0], p[1]) for p in smoothed_path]
    
    @staticmethod
    def smooth_path_spline(path, num_points=None, smoothing_factor=0):
        """基于样条曲线的路径平滑"""
        if len(path) <= 2:
            return path
        
        # 提取坐标
        x_coords = [p.x for p in path]
        y_coords = [p.y for p in path]
        
        # 计算累积距离作为参数
        distances = [0]
        for i in range(1, len(path)):
            dist = np.sqrt((x_coords[i] - x_coords[i-1])**2 + 
                          (y_coords[i] - y_coords[i-1])**2)
            distances.append(distances[-1] + dist)
        
        # B样条插值
        try:
            tck, u = splprep([x_coords, y_coords], u=distances, s=smoothing_factor, k=3)
            
            # 生成新的点
            if num_points is None:
                num_points = len(path) * 2
            
            u_new = np.linspace(0, distances[-1], num_points)
            smooth_coords = splev(u_new, tck)
            
            return [Point(x, y) for x, y in zip(smooth_coords[0], smooth_coords[1])]
        
        except Exception as e:
            print(f"样条插值失败: {e}")
            return path
    
    @staticmethod
    def smooth_path_bezier(path):
        """基于贝塞尔曲线的路径平滑"""
        if len(path) <= 2:
            return path
        
        smoothed_points = []
        smoothed_points.append(path[0])  # 起点
        
        for i in range(1, len(path) - 1):
            # 为每个路径段创建贝塞尔曲线
            p0 = path[i-1]
            p1 = path[i]
            p2 = path[i+1]
            
            # 计算控制点
            control_factor = 0.3
            cp1_x = p1.x + control_factor * (p0.x - p2.x)
            cp1_y = p1.y + control_factor * (p0.y - p2.y)
            cp2_x = p1.x - control_factor * (p0.x - p2.x)
            cp2_y = p1.y - control_factor * (p0.y - p2.y)
            
            # 生成贝塞尔曲线点
            for t in np.linspace(0, 1, 10):
                x = ((1-t)**3 * p0.x + 3*(1-t)**2*t * cp1_x + 
                     3*(1-t)*t**2 * cp2_x + t**3 * p1.x)
                y = ((1-t)**3 * p0.y + 3*(1-t)**2*t * cp1_y + 
                     3*(1-t)*t**2 * cp2_y + t**3 * p1.y)
                smoothed_points.append(Point(x, y))
        
        smoothed_points.append(path[-1])  # 终点
        return smoothed_points
    
    @staticmethod
    def douglas_peucker_simplify(path, epsilon=2.0):
        """Douglas-Peucker算法简化路径"""
        if len(path) <= 2:
            return path
        
        def perpendicular_distance(point, line_start, line_end):
            """计算点到直线的垂直距离"""
            if line_start.x == line_end.x and line_start.y == line_end.y:
                return np.sqrt((point.x - line_start.x)**2 + (point.y - line_start.y)**2)
            
            A = line_end.y - line_start.y
            B = line_start.x - line_end.x
            C = line_end.x * line_start.y - line_start.x * line_end.y
            
            return abs(A * point.x + B * point.y + C) / np.sqrt(A**2 + B**2)
        
        def douglas_peucker_recursive(points, epsilon):
            if len(points) <= 2:
                return points
            
            # 找到距离直线最远的点
            max_distance = 0
            max_index = 0
            
            for i in range(1, len(points) - 1):
                distance = perpendicular_distance(points[i], points[0], points[-1])
                if distance > max_distance:
                    max_distance = distance
                    max_index = i
            
            # 如果最大距离大于阈值，递归处理
            if max_distance > epsilon:
                left_points = douglas_peucker_recursive(points[:max_index+1], epsilon)
                right_points = douglas_peucker_recursive(points[max_index:], epsilon)
                
                # 合并结果（避免重复点）
                return left_points[:-1] + right_points
            else:
                return [points[0], points[-1]]
        
        return douglas_peucker_recursive(path, epsilon)