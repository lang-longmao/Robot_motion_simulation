from base import Environment, Point
from sim import DynamicPathPlanningSimulator
import yaml

def create_custom_environment() -> Environment:
    """创建自定义环境"""
    cfg = None
    with open('config.yml', 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    env = Environment(cfg['map']['height'], cfg['map']['width'])

    # 设置起点和终点
    env.start = Point(*cfg['map']['start'])
    env.goal = Point(*cfg['map']['goal'])
    
    # 添加复杂障碍物布局 
    obstacles = cfg['map']['obstacles']
    
    for obs in obstacles:
        env.add_obstacle(*obs)
    
    return env

def demo_single_algorithm():
    """演示单个算法"""
    print("=" * 60)
    print("移动机器人路径规划系统 - 单算法演示")
    print("=" * 60)
    
    # 创建仿真器
    simulator = DynamicPathPlanningSimulator(create_custom_environment())
    
    # 用户选择
    print("\n请选择路径规划算法:")
    print("1. Dijkstra算法")
    print("2. A*算法") 
    print("3. RRT算法")
    
    choice = input("请输入选择 (1-3): ").strip()
    planner_map = {'1': 'dijkstra', '2': 'astar', '3': 'rrt'}
    
    if choice not in planner_map:
        print("无效选择，使用默认A*算法")
        choice = '2'
    
    print("\n请选择跟踪控制算法:")
    print("1. PID控制")
    print("2. LQR控制")
    print("3. MPC控制")
    
    ctrl_choice = input("请输入选择 (1-3): ").strip()
    controller_map = {'1': 'pid', '2': 'lqr', '3': 'mpc'}
    
    if ctrl_choice not in controller_map:
        print("无效选择，使用默认PID控制")
        ctrl_choice = '1'
    
    # 执行仿真
    simulator.start_dynamic_simulation(
        planner_type=planner_map[choice], 
        controller_type=controller_map[ctrl_choice], 
        animation_speed=50       # 动画间隔(毫秒)，值越小动画越快
    )

    #     print("路径规划失败，无法进行跟踪仿真")

def demo_comparison_study():
    """演示对比研究"""
    print("=" * 60)
    print("移动机器人路径规划系统 - 算法对比研究")
    print("=" * 60)
    
    # 创建仿真器
    simulator = DynamicPathPlanningSimulator(create_custom_environment())
    
    # 运行对比研究
    results = simulator.run_comparison_study()
    
    



if __name__ == "__main__":
    print("移动机器人路径规划与跟踪控制系统")
    print("请选择运行模式:")
    print("1. 单算法演示")
    print("2. 算法对比研究") 
    print("3. 退出")
    
    while True:
        choice = input("\n请输入选择 (1-3): ").strip()
        
        if choice == '1':
            demo_single_algorithm()
        elif choice == '2':
            demo_comparison_study()
        elif choice == '3':
            print("已退出!")
            break
        else:
            print("无效选择，请重新输入")
        
        print("\n" + "="*60)
        print("请选择下一个操作:")
        print("1. 单算法演示")
        print("2. 算法对比研究")
        print("3. 退出")