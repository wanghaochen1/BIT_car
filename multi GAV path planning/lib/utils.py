# Utils 中存放的是一些常用的计算工具

import math
import numpy as np
import matplotlib.pyplot as plt

'''
将角度归一化到 [-pi, pi]
'''


def normalize_angle(angle: float):
    normalized_angle = math.fmod(angle + math.pi, 2.0 * math.pi)

    if (normalized_angle < 0.0):
        normalized_angle = normalized_angle + 2.0 * math.pi

    return normalized_angle - math.pi


'''
将一整个列表的角度都归一化（一维）
'''


def normalize_angle_list(angle_list):
    normalized_angles = np.fmod(angle_list + np.pi, 2.0 * np.pi)
    normalized_angles[normalized_angles < 0.0] += 2.0 * np.pi

    return normalized_angles - np.pi


'''
计算两个坐标点 (x1, y1), (x2, y2) 之间的距离
'''


def distance(x1: float, y1: float, x2: float, y2: float):
    dx = x2 - x1
    dy = y2 - y1

    return math.sqrt(dy * dy + dx * dx)


'''
用于辅助绘制圆形
'''


def plot_circle_helper(center_x: float, center_y: float, radius: float):
    theta = np.linspace(0, 2 * math.pi, 100)
    x = radius * np.cos(theta) + center_x
    y = radius * np.sin(theta) + center_y

    return x, y


def path_to_ue4(path, world_offset):
    path.reverse()
    new_path = [(x + world_offset[0], y + world_offset[1]) for (x, y) in path]
    return new_path


# 这个实时绘图函数可以绘制车辆历史路径、当前位置及参考路径信息
# 大家在自己的代码中也可以增加这一部分内容用来可视化规划控制结果
# 方便调试代码
def plot_helper(vehicle_pose, ref_path, vehicle_geometry, actual_x_list, actual_y_list):
    area = 15
    vehicle_shape = vehicle_geometry.get_vehicle_shape(vehicle_pose)

    plt.cla()
    plt.plot(ref_path.interp_x_, ref_path.interp_y_, color='black', label='reference path')
    plt.plot(vehicle_shape[:, 0], vehicle_shape[:, 1], color='orange', label='ego_vehicle')
    plt.plot(actual_x_list, actual_y_list, color='red', label='traveld path')

    plt.axis('equal')
    plt.xlim([vehicle_pose.x_ - area, vehicle_pose.x_ + area])
    plt.ylim([vehicle_pose.y_ - area, vehicle_pose.y_ + area])
    plt.legend()
    plt.grid(True)
    plt.pause(0.01)
