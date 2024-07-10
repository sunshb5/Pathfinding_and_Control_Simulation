"""

Path Length Calculation Tool

"""
import math


def path_length(cx, cy):
    length = 0.0
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        segment_length = math.sqrt(dx**2 + dy**2)
        length += segment_length
    return length
