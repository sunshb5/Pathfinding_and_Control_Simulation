"""

Path tracking simulation with Stanley steering control and PID speed control.

"""
import time
import numpy as np
import matplotlib.pyplot as plt
from utils.arrow import plot_arrow
from utils.angle import angle_mod
from model.map import Map
from utils.plot import Visualizer
from utils import cubic_spline

k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheelbase of vehicle
max_steer = np.radians(40.0)  # [rad] max steering angle


class State:
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super().__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, c_yaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param c_yaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(c_yaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    return angle_mod(angle)


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def track(path):
    """Plot an example of Stanley steering control on a cubic spline."""
    start_time = time.time()
    print("Stanley track start!!")

    #  target course
    ax = path[0]
    ay = path[1]
    cx, cy, c_yaw, ck, s = cubic_spline.calc_spline_course(ax, ay, ds=0.1)

    target_speed = 20.0 / 3.6  # [m/s]

    max_t = 100.0  # max simulation time

    # Initial state
    state = State(x=ax[0], y=ay[0], yaw=np.arctan2(cy[1]-cy[0], cx[1]-cx[0]), v=0.0)

    last_idx = len(cx) - 1
    time_ = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    # Store actual trajectory
    tx = [state.x]
    ty = [state.y]

    plt.figure(figsize=(8, 6))
    while max_t >= time_ and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, c_yaw, target_idx)
        state.update(ai, di)

        time_ += dt

        # Store actual trajectory
        tx.append(state.x)
        ty.append(state.y)

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time_)

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        map_obj = Map()
        visualizer = Visualizer(map_obj)
        visualizer.visualize()

        plot_arrow(state.x, state.y, state.yaw)
        plt.plot(cx, cy, "-r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
        plt.pause(0.001)

    assert last_idx >= target_idx, "Cannot reach goal"

    plt.subplots(1)
    plt.plot(t, [iv * 3.6 for iv in v], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)

    track_time = time.time() - start_time
    print(f"Path tracking time: {track_time} seconds")

    plt.show()

    return tx, ty
