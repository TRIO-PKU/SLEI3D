import numpy as np
import heapq
from math import sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Tuple
import time

class AStar3D:
    def __init__(self, voxel_map):
        """
        初始化A*算法
        :param voxel_map: 3D numpy ndarray, 0表示可通过，1表示障碍物
        """
        self.voxel_map = voxel_map
        self.shape = voxel_map.shape
        self.directions = [
            (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), 
            (0, 0, 1), (0, 0, -1), (1, 1, 0), (-1, -1, 0),
            (1, -1, 0), (-1, 1, 0), (1, 0, 1), (-1, 0, -1),
            (1, 0, -1), (-1, 0, 1), (0, 1, 1), (0, -1, -1),
            (0, 1, -1), (0, -1, 1), (1, 1, 1), (-1, -1, -1),
            (1, 1, -1), (-1, -1, 1), (1, -1, 1), (-1, 1, -1),
            (1, -1, -1), (-1, 1, 1)
        ]  # 26 possible movements in 3D space

    def heuristic(self, a, b):
        """
        计算启发式函数h，用于估计从点a到点b的距离
        :param a: tuple (x1, y1, z1)
        :param b: tuple (x2, y2, z2)
        :return: 直线平方距离
        """
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2

    def is_valid(self, node):
        """
        检查节点是否在地图范围内，且不是障碍物
        :param node: tuple (x, y, z)
        :return: True if valid, False otherwise
        """
        x, y, z = node
        return (0 <= x < self.shape[0] and
                0 <= y < self.shape[1] and
                0 <= z < self.shape[2] and
                self.voxel_map[x, y, z] == 0)

    def find_path(self, start, goal) -> List[Tuple[int, int, int]]:
        """
        使用A*算法寻找从起点到终点的路径
        :param start: tuple (x, y, z), 起点坐标
        :param goal: tuple (x, y, z), 终点坐标
        :return: 路径的list, 或None如果找不到路径
        """
        open_set = []
        heapq.heappush(open_set, (0, start))  # 优先队列，存储 (f_score, node)
        came_from = {}  # 记录路径
        g_score = {start: 0}  # 从起点到当前节点的实际成本
        f_score = {start: self.heuristic(start, goal)}  # 估计成本 (g_score + h)

        while open_set:
            _, current = heapq.heappop(open_set)

            # 如果到达目标
            if current == goal:
                return self.reconstruct_path(came_from, current)

            # 遍历当前节点的邻居
            for direction in self.directions:
                neighbor = (current[0] + direction[0],
                            current[1] + direction[1],
                            current[2] + direction[2])

                if not self.is_valid(neighbor):
                    continue

                # 从当前节点到邻居的距离为1
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # 没有找到路径

    def reconstruct_path(self, came_from, current):
        """
        通过came_from字典重构路径
        :param came_from: 记录路径的字典
        :param current: 当前节点
        :return: 经过的路径
        """
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # 翻转路径
    
    def plot_map_and_path(self, path):
        """
        使用matplotlib绘制体素地图和路径
        :param path: 经过的路径
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 获取障碍物坐标
        obstacles = np.argwhere(self.voxel_map == 1)
        if obstacles.size > 0:
            ax.scatter(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], c='r', marker='s', label='Obstacles')

        # 绘制路径
        if path:
            path = np.array(path)
            ax.plot(path[:, 0], path[:, 1], path[:, 2], c='b', marker='o', label='Path')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Voxel Map with Path')
        ax.legend()
        plt.show()

if __name__ == "__main__":

    # 示例使用：
    voxel_map = np.load("/home/tcxm/ExpInsSim/src/mission_manager/models/voxel_map.npy")
    start = (10,10, 10)
    goal = (100,100, 20)

    astar = AStar3D(voxel_map)
    time_log = time.time()
    path = astar.find_path(start, goal)
    print(f"Time cost: {time.time() - time_log} s")

    if path:
        print("Path found:", path)
        astar.plot_map_and_path(path)
    else:
        print("No path found")
