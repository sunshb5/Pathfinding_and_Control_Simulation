"""

Plan and Track Method Selection

"""
parameters = {
    # 选择规划算法
    "Plan": "Dijkstra",  # Dijkstra算法
    # "Plan": "Astar",     # A*算法
    # "Plan": "Bid_Astar",    # 双向A*算法
    # "Plan": "proba_road_map",  # 概率道路地图算法

    # 选择追踪算法
    "Track": "pure_pursuit",  # 纯跟踪算法
    # "Track": "stanley",       # Stanley控制算法
    # "Track": "increase_pid",  # 增量PID控制
    # "Track": "position_pid",   # 位置PID控制
}
