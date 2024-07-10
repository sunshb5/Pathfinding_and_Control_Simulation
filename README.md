# Table of Contents
   * [Introduction](#Introduction)
   * [Path Plan](#Path-Plan)
      * [A* algorithm](#Astar)
      * [Bidirectional A* algorithm](#Bidirectional-Astar)
      * [Dijkstra algorithm](#Dijkstra)
      * [Probabilistic Road-Map (PRM) planning](#Probabilistic-Roadmap)
   * [Path Track](#Path-Track)
      * [Pure Pursuit](#Pure-Pursuit)
      * [Stanley control](#stanley)
      * [Position PID](#Position-PID)
      * [Increase PID](#Increase-PID)
   * [Directory Structure](#Directory-Structure)
   * [Requirements](#Requirements)
   * [Run](#Run)


# Introduction

+ 本项目设计了一个场景，包括起点，终点，障碍物;
+ 分别采用了4种规划方法、4种横向控制方法实现路径规划与跟踪过程，模拟了移动机器人运动的过程;
+ 机器人本体的运动控制模型为差分小车;




# Path plan

### Astar

+ A* 是一种广泛使用的图遍历和路径规划算法。它有效地在加权图中找到从起始节点到目标节点的最短路径；
+ 使用到达节点的成本（g(n)）和到目标的估计成本（h(n)）来优先探索节点，通常通过优先级队列（如最小堆）实现；



### Bidirectional Astar

+ 双向 A* 在 A* 的基础上进行增强，同时从起始节点向目标和从目标向起始节点运行两个 A* 搜索；
+ 对于在大图中找到最短路径而言，通常比传统的 A* 更快，因为它通过中间相遇减少了搜索空间，需要有效的双向图遍历机制；



### Dijkstra

+ Dijkstra 算法用于在加权图中找到从起始节点到目标节点的最短路径；
+ 与 A* 类似，但不考虑到目标的估计成本，而是仅使用到达节点的实际成本（g(n)），通常通过优先级队列实现；



### Probabilistic Roadmap:

+ 概率路径地图是一种基于采样的路径规划方法，通过在自由空间中随机采样点，并连接可行的点来构建路径网络；
+ 可以处理复杂的环境和动态障碍物，通过随机采样和路径连接来生成路径图，适用于高维和连续空间的路径规划；




# Path Track

### Pure Pursuit

+ 纯跟踪算法是一种基于路径跟踪的控制方法，通常用于机器人或车辆沿着预定义路径移动；
+ 它通过计算车辆当前位置到路径上最近点的最短距离，然后计算出一个跟随路径前进的目标点，通过控制车辆前轮朝向目标点来实现跟踪路径的目的；



### Stanley

+ Stanley控制算法是一种基于偏差角度的路径跟踪方法；
+ 它通过测量车辆当前位置与目标路径的横向偏差和车辆当前朝向与目标路径方向之间的差异来计算一个修正的前轮朝向角度，使得车辆能够沿着路径稳定地移动；



### Position PID

+ 位置PID控制是一种基于位置误差、积分误差和微分误差的反馈控制方法;
+ 在路径跟踪中，位置PID控制会根据车辆当前位置与目标位置之间的差异，计算一个控制量，用于调节车辆的速度和方向，使得车辆能够到达目标位置;



### Increase PID

+ 增量PID控制与位置PID控制类似，但是它是基于当前控制量的增量来调节车辆的行为;
+ 增量PID控制通过计算当前位置误差与前一时刻位置误差之间的差异，来生成一个增量控制量，从而使得车辆能够更加平稳地跟踪路径或达到目标位置;




# Directory Structure

    ├── ReadMe.md            // 帮助文档
    
    ├── requirement.txt      // 环境依赖文件

    ├── main_plan_track.py    // 主函数文件，调用实现差分小车在特定场景中的路径规划与跟踪
    
    ├── PathPlan             // 路径规划方法
      
    │   └── astar.py            // A*算法
    
    │   └── bidirectional.py    // 双向A*算法
    
    │   └── dijkstra.py         // Dijkstra算法
    
    │   └── probabilistic_road_map.py   // 概率道路地图算法

    ├── PathTrack           // 跟踪控制方法
    
    │   └── increase_pid_controller.py    // 增量PID控制
    
    │   └── position_pid_controller.py    // 位置PID控制
    
    │   └── pure_pursuit_controller.py    // 纯跟踪算法
    
    │   └── stanley_controller.py         // Stanley控制算法
    
    ├── metric       // 评估指标
    
    │   ├── eval.py        // 路径质量评估函数
    
    ├── model       //  包含地图模型与规划、跟踪方法参数
    
    │   ├── map.py        // 地图模型，包含起点，终点及障碍物
    
    │   ├── parameter.py  // 参数模型，用于规划、跟踪方法的选择

    ├── utils       // 工具
    
    │   ├── angle.py    // 角度计算

    │   ├── arrow.py    // 箭头绘制

    │   ├── cubic_spline.py    // 插值函数

    │   ├── path_length.py    // 路径长度计算

    │   ├── plot.py    // 可视化绘图




# Requirements
+ Notice:Lower versions may also be applicable.
+ Python 3.11.7
+ numpy == 1.24.3
+ scipy == 1.10.1
+ matplotlib == 3.7.5



 
# Run
1. ### 配置运行环境

   在环境中安装依赖库。

   ```python
   pip install requirements.txt
   ```

2. ### 参数调整

   + 在model/parameter.py文件中修改运行参数以选择规划、跟踪方法；

   ```python
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
   ```

   + 在model/map.py文件中修改运行参数以更改场景，包括起点、终点及障碍物；
     
    ```python
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

   ```

3. ### 运行

   在终端输入命令：

   ```python
   python main_plan_track.py
   ```
 


