# 目录 

- [Introduction](#规划器) 
  - [A*](#A*) 
  - [Dijkstra](#Dijkstra) 
- [跟踪器](#跟踪器) 
  - [Pure Pursuit](#Pure Pursuit) 
  - [Stanley](#Stanley) 
  - [增量PID](#增量PID) 
  - [位置PID](#位置PID) 
  - [LQR](#LQR) 

- [Run](#Run)

# Introduction

本项目设计了一个场景，包括起点，终点，障碍物，分别采用4种规划方法、4种横向控制方法实现路径跟踪过程，模拟了移动机器人运动的过程，机器人本体的运动控制模型为差分小车。

# Path plan
### A*

A-star算法使用启发式搜索，启发式搜索是一种通过启发式函数来优化搜索效率的方法。启发式函数H(n)评估了从节点n到目标节点的预期成本，用于指导搜索过程朝着最有希望的方向前进。在A-star算法中，启发式函数结合了实际成本（从起始点到当前节点的路径成本G(n)）和预估成本（从当前节点到目标节点的估计成本H(n)），帮助算法决定下一步要探索的节点。A-star算法通过维护一个优先级队列（通常使用最小堆实现）来按照最小化的总预估成本搜索路径。

### Dijkstra

Dijkstra算法基于贪心策略，通过逐步扩展离起始节点最近的节点来实现最短路径的计算。其核心思想是维护一个距离起始节点的最短路径长度的集合，并通过每一步选择当前最短路径的节点进行扩展，直到所有节点都被遍历。它不需要启发式函数，适用于需要精确最短路径计算的场景。

# Path Track
### Pure Pursuit

纯跟踪算法模拟了人类驾驶车辆时的自然行为：在人类驾驶车辆时，为了跟踪一条期望的轨迹，驾驶员通常会选择一个距离车辆当前位置一定距离的点作为目标，然后调整方向盘，使车辆行驶到该点。纯跟踪算法借鉴了这种思维方式，选择预定路径上的一个目标点，通过调整车辆的转向角度，使车辆沿着圆弧轨迹驶向目标点，进而实现路径跟踪。

纯跟踪算法的核心思想是选择预定路径上的一个目标点（追踪点），计算从当前车辆位置到该点的轨迹曲率，从而生成控制命令，引导车辆沿路径行驶。具体步骤如下：

1. **路径点选择**：根据车辆当前位置，在预定路径上选择一个距离车辆一定距离的目标点。
2. **计算轨迹曲率**：根据车辆位置、目标点位置和路径曲率，计算车辆需要转向的角度。
3. **生成控制命令**：根据计算的转向角度，生成相应的转向命令，控制车辆行驶。

### Stanley

Stanley控制算法是一种基于横向误差的跟踪算法。Stanley 控制器的核心思想是通过调整车辆的转向角度，使车辆的前轴朝向路径上的最近点，并最小化车辆的横向误差和航向误差。与纯跟踪算法不同，Stanley 控制器不仅考虑车辆当前位置与路径的偏差，还考虑车辆的航向角误差。

Stanley 控制器通过以下两个主要步骤实现路径跟踪：

1. **横向误差校正**：计算车辆前轴中心到路径的最近点的横向误差，并生成相应的转向角度来减小该误差。
2. **航向误差校正**：计算车辆朝向与路径切线方向之间的航向误差，并生成相应的转向角度来减小该误差。

### 增量PID

增量式 PID 控制器的核心思想是通过计算当前时刻的控制量相对于上一时刻的增量来调节系统的输出。与传统 PID 控制器不同，增量式 PID 控制器仅计算控制量的增量，避免了累积误差的直接影响，从而提高了系统的抗干扰能力。

### 位置PID

位置式PID控制器基于当前时刻的误差，积分误差 和误差变化率 ，通过如下公式计算控制量：

$$
u(t) = K_p \cdot e(t) + K_i \cdot \int_{0}^{t} e(\tau) \, d\tau + K_d \cdot \frac{de(t)}{dt}
$$


 
# Requirements
+ Notice:Lower versions may also be applicable.
+ Python 3.11.7
+ numpy == 1.24.3
+ scipy == 1.10.1
+ matplotlib == 3.7.5
 



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

 
# Run
1. ### 配置运行环境

   在自己所使用的环境中安装依赖库。

   ```python
   pip install requirements.txt
   ```

2. ### 参数选择

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
 


