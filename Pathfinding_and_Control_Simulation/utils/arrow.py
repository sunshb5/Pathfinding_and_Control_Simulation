"""

Arrow Drawing Tool

"""
import matplotlib.pyplot as plt
import math


def plot_arrow(x, y, yaw, length=1.0, width=1.5, fc="r", ec="purple"):
    """
    Plot arrow
    """
    if not isinstance(x, float):
        for ix, iy, i_yaw in zip(x, y, yaw):
            plot_arrow(ix, iy, i_yaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)
