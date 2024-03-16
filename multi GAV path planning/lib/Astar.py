import cv2
import numpy as np
from scipy.interpolate import splprep, splev
import math

# end: 243 2115
# start: 1800 2010
def Astar(maps, start_x, start_y, end_x, end_y):
    print('开始规划')

    maps_size = np.array(maps)  # 获取图像行和列大小
    hight = maps_size.shape[0]  # 行数->y   行数4794
    width = maps_size.shape[1]  # 列数->x   列数4794
    # print(hight)
    # print(width)
    start = {'位置': (start_x, start_y), 'G': 0, 'F': 0, '父节点': (start_x, start_y)}  # 起点
    end = {'位置': (end_x, end_y), 'G': 700, 'F': 700, '父节点': (end_x, end_y)}  # 终点
    print(start_x, start_y)

    openlist = []  # open列表，存储可能路径
    closelist = [start]  # close列表，已走过路径
    step_size = 5  # 搜索步长。

    # 步长太小，搜索速度就太慢。步长太大，可能直接跳过障碍，得到错误的路径
    # 步长大小要大于图像中最小障碍物宽度
    while 1:
        s_point = closelist[-1]['位置']  # 获取close列表最后一个点位置，S点 (x,y)
        G = closelist[-1]['G']
        add = ([0, step_size], [0, -step_size], [step_size, 0], [-step_size, 0])  # 可能运动的四个方向增量
        for i in range(len(add)):
            x = s_point[0] + add[i][0]  # 检索超出图像大小范围则跳过
            if x < 0 or x >= width:
                continue
            y = s_point[1] + add[i][1]
            if y < 0 or y >= hight:  # 检索超出图像大小范围则跳过
                continue
            if maps[y][x] == 0:
                continue
            G = G + 1  # 计算代价
            H = abs(x - end['位置'][0]) + abs(y - end['位置'][1])  # 计算代价
            F = G + H

            if H < 20:  # 当逐渐靠近终点时，搜索的步长变小
                step_size = 1
            addpoint = {'位置': (x, y), 'G': G, 'F': F, '父节点': s_point}  # 更新位置
            count = 0
            for i in openlist:
                if i['位置'] == addpoint['位置']:
                    count += 1
                    if i['F'] > addpoint['F']:
                        i['G'] = addpoint['G']
                        i['F'] = addpoint['F']
                        i['父节点'] = s_point
            for i in closelist:
                if i['位置'] == addpoint['位置']:
                    count += 1
            if count == 0:  # 新增点不在open和close列表中
                openlist.append(addpoint)

        t_point = {'位置': (91, 70), 'G': 10000, 'F': 10000, '父节点': (91, 70)}
        for j in range(len(openlist)):  # 寻找代价最小点
            if openlist[j]['F'] < t_point['F']:
                t_point = openlist[j]
                # print(1)
        for j in range(len(openlist)):  # 在open列表中删除t点
            if t_point == openlist[j]:
                openlist.pop(j)
                break
        closelist.append(t_point)  # 在close列表中加入t点

        if t_point['位置'] == end['位置']:  # 找到终点！！
            print("找到终点")
            break

    # 逆向搜索找到路径
    road = []
    road.append(closelist[-1])
    point = road[-1]

    while 1:
        for i in closelist:
            if i['位置'] == point['父节点']:  # 找到父节点
                point = i
                # print(point)
                road.append(point)
        if point == start:
            print("路径搜索完成")
            break

    informap = cv2.cvtColor(maps, cv2.COLOR_GRAY2BGR)
    path = [item['位置'] for item in road]

    for i in range(len(path)):  # 整数型画路径
        cv2.circle(informap, path[i], 2, (0, 0, 200), -1)
    cv2.circle(informap, start['位置'], 15, (0, 255, 0), -1)  # 起点
    cv2.circle(informap, end['位置'], 15, (0, 0, 255), -1)  # 终点

    # target_width = 900
    # target_height = 900
    cv2.imwrite("informap.png", informap)
    # informap = cv2.resize(informap, (target_width, target_height))
    # cv2.imshow("informap.png", informap)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return path


class Node:
    def __init__(
            self,
            position: tuple,
            g: float,
            f: float,
            parent: 'Node'
    ) -> None:
        self.position = position
        self.g = g
        self.f = f
        self.parent_node = parent


class ClassAStar:
    def __init__(
            self,
            start_x: int,
            start_y: int,
            end_x: int,
            end_y: int,
            map_image: np.ndarray,
            step_size=5,
            search_range=40,
            scale=5
    ) -> None:

        self.start_x_ = start_x // scale
        self.start_y_ = start_y // scale
        self.end_x_ = end_x // scale
        self.end_y_ = end_y // scale
        self.scale_ = scale

        resize_height = map_image.shape[0] // scale
        resize_width = map_image.shape[1] // scale
        self.map_image_ = cv2.resize(map_image, (resize_width, resize_height))

        self.step_size_ = step_size
        self.search_range_ = search_range

    def search_path(self):
        start_node = Node((self.start_x_, self.start_y_), 0, 0, None)
        end_node = Node((self.end_x_, self.end_y_), math.inf, math.inf, None)

        open_list = []
        close_list = [start_node]

        step_size = self.step_size_
        map_image = self.map_image_

        map_height = map_image.shape[0]
        map_width = map_image.shape[1]

        while 1:
            current_node = close_list[-1]

            possible_moves = (
                [0, step_size],
                [0, -step_size],
                [step_size, 0],
                [-step_size, 0]
            )

            for move in possible_moves:
                g_cost = current_node.g + 1
                x = current_node.position[0] + move[0]
                if x < 0 or x >= map_width:
                    continue
                y = current_node.position[1] + move[1]
                if y < 0 or y >= map_height:
                    continue
                if map_image[y][x] == 0:
                    continue
                h_cost = abs(x - end_node.position[0]) + abs(y - end_node.position[1])
                f_cost = g_cost + h_cost

                if h_cost < self.search_range_:
                    step_size = 1
                new_node = Node((x, y), g_cost, f_cost, current_node)
                count = 0
                for node in open_list:
                    if node.position == new_node.position:
                        count += 1
                        if node.f > new_node.f:
                            node.g = new_node.g
                            node.f = new_node.f
                            node.parent_node = current_node
                for node in close_list:
                    if node.position == new_node.position:
                        count += 1
                if count == 0:
                    open_list.append(new_node)

            temp_node = None

            for i in range(len(open_list)):
                if temp_node is None or open_list[i].f < temp_node.f:
                    temp_node = open_list[i]

            if temp_node is None:
                print('No path found to the destination.')
                return []

            for i in range(len(open_list)):
                if temp_node == open_list[i]:
                    open_list.pop(i)
                    break
            close_list.append(temp_node)

            if temp_node.position == end_node.position:
                print('End point found')
                break
        return close_list

    def retrieve_path(self, close_list):
        start_node_position = (self.start_x_, self.start_y_)
        path = []
        path.append(close_list[-1])
        point = path[-1]

        while 1:
            for node in close_list:
                if node.position == (point.parent_node.position if point.parent_node else None):
                    point = node
                    path.append(point)
            if point.position == start_node_position:
                print('Path search completed')
                break
        return path

    def update_planning(self):
        close_list = self.search_path()

        if close_list == []:
            return []

        path = self.retrieve_path(close_list)

        resize_path = []
        for node in path:
            resize_path.append((node.position[0] * self.scale_,
                                node.position[1] * self.scale_))

        return resize_path
    

if __name__ == '__main__':
    maps = cv2.imread("map.jpg")
    maps = cv2.cvtColor(maps, cv2.COLOR_RGB2GRAY)
    imgGaussBlur = cv2.GaussianBlur(maps, ksize=(3, 3), sigmaX=2)  # sigmaX：x 轴方向的高斯核标准差
    # binaryGaussian = cv2.adaptiveThreshold(imgGaussBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 7, 6)  # 局部自适应阈值二值化
    # binary_Erode = cv2.erode(imgGaussBlur, kernel=np.array([[0, 0, 0, 0]], dtype=np.uint8))
    binary_Erode = cv2.erode(imgGaussBlur, kernel=np.ones((10, 10), dtype=np.uint8))
    cv2.imwrite("map_erode.jpg", binary_Erode)
    # import matplotlib.pyplot as plt
    # plt.imshow(binaryGaussian, cmap='gray')
    # plt.show()

    Astar(binary_Erode, 1800, 2010, 233, 2120)
    
