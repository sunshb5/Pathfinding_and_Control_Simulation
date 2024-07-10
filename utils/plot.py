"""

Visualization Plotting Tool

"""
import matplotlib.pyplot as plt


class Visualizer:
    def __init__(self, map_obj):
        self.map = map_obj

    def visualize(self):
        plt.plot(self.map.ox, self.map.oy, ".k")
        plt.plot(self.map.sx, self.map.sy, "o", color="purple", markersize=7, label='Start')
        plt.plot(self.map.gx, self.map.gy, marker="*", color="red", markersize=8, label='Goal')
        plt.grid(True)
        plt.axis("equal")
        plt.legend()  # 添加图例
        plt.title("Path Planning...,Click Close to Continue")
        plt.tight_layout()

    @staticmethod
    def plot_path(rx, ry):
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()
