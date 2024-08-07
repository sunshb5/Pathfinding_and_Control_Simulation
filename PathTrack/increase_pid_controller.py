"""

Path tracking simulation with Position PID control.

"""
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from model.map import Map
from utils.plot import Visualizer
from utils.arrow import plot_arrow

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheelbase of vehicle


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        if self.old_nearest_point_index is None:
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        lf = k * state.v + Lfc

        while lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break
            ind += 1

        return ind, lf


class IncrementalPIDController:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.prev_error = 0
        self.prev_prev_error = 0
        self.integral = 0

    def control(self, target, current):
        error = target - current
        delta_u = (self.Kp * (error - self.prev_error) + self.Ki * error
                   + self.Kd * (error - 2 * self.prev_error + self.prev_prev_error))

        self.prev_prev_error = self.prev_error
        self.prev_error = error

        return delta_u


def pure_pursuit_steer_control(state, trajectory, p_ind):
    ind, lf = trajectory.search_target_index(state)
    if p_ind >= ind:
        ind = p_ind
    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    delta = math.atan2(2.0 * WB * math.sin(alpha) / lf, 1.0)

    return delta, ind


def track(path):
    start_time = time.time()
    print("Position PID track start!!")

    cx = path[0]
    cy = path[1]

    target_speed = 20.0 / 3.6  # [m/s]
    max_t = 100.0  # max simulation time

    state = State(x=cx[0], y=cy[0], yaw=np.arctan2(cy[1]-cy[0], cx[1]-cx[0]), v=0.0)
    states = States()
    states.append(0.0, state)

    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    pid_controller = IncrementalPIDController(kp=1.0, ki=0.1, kd=0.01)

    time_ = 0.0
    last_index = len(cx) - 1

    # Store actual trajectory
    tx = [state.x]
    ty = [state.y]

    plt.figure(figsize=(8, 6))
    while max_t >= time_ and last_index > target_ind:
        ai = pid_controller.control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)
        state.update(ai, di)

        time_ += dt
        states.append(time_, state)

        # Store actual trajectory
        tx.append(state.x)
        ty.append(state.y)

        plt.cla()
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

        map_obj = Map()
        visualizer = Visualizer(map_obj)
        visualizer.visualize()
        plot_arrow(state.x, state.y, state.yaw)
        plt.plot(cx, cy, "-r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
        plt.pause(0.001)

    assert last_index >= target_ind, "Cannot goal"

    plt.subplots(1)
    plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)

    track_time = time.time() - start_time
    print(f"Path tracking time: {track_time} seconds")

    plt.show()

    return tx, ty
