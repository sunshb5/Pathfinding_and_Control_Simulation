"""

Path Quality Evaluator

"""
import numpy as np
import math
from scipy.interpolate import interp1d


class PathEvaluator:
    def __init__(self, plan_path, track_path):
        self.plan_path = plan_path
        self.track_path = track_path

    # 计算路径偏差
    def path_deviation(self):
        plan_x, plan_y = self.plan_path
        track_x, track_y = self.track_path

        # 将 track_path 插值到 plan_path 点上
        track_interp_func_x = interp1d(np.arange(len(track_x)), track_x, kind='linear', fill_value='extrapolate')
        track_interp_func_y = interp1d(np.arange(len(track_y)), track_y, kind='linear', fill_value='extrapolate')

        track_x_interp = track_interp_func_x(np.linspace(0, len(track_x) - 1, len(plan_x)))
        track_y_interp = track_interp_func_y(np.linspace(0, len(track_y) - 1, len(plan_y)))

        # 计算对应点之间的距离
        deviations = np.sqrt((track_x_interp - np.array(plan_x)) ** 2 + (track_y_interp - np.array(plan_y)) ** 2)

        # 平均和最大偏差
        avg_deviation = np.mean(deviations)
        max_deviation = np.max(deviations)

        return avg_deviation, max_deviation

    # 计算路径长度
    @staticmethod
    def path_length(path):
        x, y = path
        length = 0.0
        for i in range(len(x) - 1):
            dx = x[i + 1] - x[i]
            dy = y[i + 1] - y[i]
            segment_length = math.sqrt(dx ** 2 + dy ** 2)
            length += segment_length
        return length

    # 计算路径长度差异
    def path_length_difference(self):
        plan_length = self.path_length(self.plan_path)
        track_length = self.path_length(self.track_path)

        length_difference = abs(track_length - plan_length)

        return length_difference

    # 计算路径平滑度
    def path_smoothness(self):
        track_x, track_y = self.track_path

        angles = []
        for i in range(1, len(track_x) - 1):
            dx1 = track_x[i] - track_x[i - 1]
            dy1 = track_y[i] - track_y[i - 1]
            dx2 = track_x[i + 1] - track_x[i]
            dy2 = track_y[i + 1] - track_y[i]

            # 检查分母是否接近零
            deno = np.sqrt(dx1 ** 2 + dy1 ** 2) * np.sqrt(dx2 ** 2 + dy2 ** 2)
            if np.isclose(deno, 0):
                angle = 0.0  # 或者其他处理方式，比如跳过这个角度
            else:
                angle = np.arccos(np.clip((dx1 * dx2 + dy1 * dy2) / deno, -1.0, 1.0))
            angles.append(angle)

        smoothness = np.std(angles)

        return smoothness

    # 计算最大转向角度
    def max_steering_angle(self):
        track_x, track_y = self.track_path

        max_angle = 0.0
        for i in range(1, len(track_x) - 1):
            dx1 = track_x[i] - track_x[i - 1]
            dy1 = track_y[i] - track_y[i - 1]
            dx2 = track_x[i + 1] - track_x[i]
            dy2 = track_y[i + 1] - track_y[i]

            # 检查分母是否接近零
            deno = np.sqrt(dx1 ** 2 + dy1 ** 2) * np.sqrt(dx2 ** 2 + dy2 ** 2)
            if not np.isclose(deno, 0):
                angle = np.arccos(np.clip((dx1 * dx2 + dy1 * dy2) / deno, -1.0, 1.0))
                max_angle = max(max_angle, angle)

        return max_angle

    # 打印评估结果
    def print_eval_results(self):
        avg_dev, max_dev = self.path_deviation()
        tracked_path_length = self.path_length(self.track_path)
        length_diff = self.path_length_difference()
        smoothness = self.path_smoothness()
        max_angle = self.max_steering_angle()

        print("平均偏差:", avg_dev)
        print("最大偏差:", max_dev)
        print("追踪路径长度:", tracked_path_length)
        print("路径长度差异:", length_diff)
        print("路径平滑度:", smoothness)
        print("最大转向角度:", max_angle)
