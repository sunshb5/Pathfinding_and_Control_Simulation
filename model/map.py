"""

Map Definition, including start, goal and obstacle

"""


class Map:
    def __init__(self, sx=10.0, sy=10.0, gx=50.0, gy=50.0, grid_size=2.0, robot_radius=1.0):
        self.sx = sx  # 起点的x坐标
        self.sy = sy  # 起点的y坐标
        self.gx = gx  # 终点的x坐标
        self.gy = gy  # 终点的y坐标
        self.grid_size = grid_size  # 网格大小
        self.robot_radius = robot_radius  # 机器人半径
        self.ox, self.oy = self.generate_obstacles()  # 生成障碍物

    @staticmethod
    def generate_obstacles():
        # 在此处定义生成障碍物的逻辑
        ox, oy = [], []
        # 障碍物的x和y坐标
        for i in range(-10, 60):
            ox.append(i)
            oy.append(-10.0)
        for i in range(-10, 60):
            ox.append(60.0)
            oy.append(i)
        for i in range(-10, 61):
            ox.append(i)
            oy.append(60.0)
        for i in range(-10, 61):
            ox.append(-10.0)
            oy.append(i)
        for i in range(-10, 30):
            ox.append(20.0)
            oy.append(i)
        for i in range(0, 20):
            ox.append(40.0)
            oy.append(60.0 - i)
        # 根据需要添加其他障碍物定义
        return ox, oy
