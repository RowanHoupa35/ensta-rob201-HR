"""
Planner class
Implementation of A*
"""

import copy
import heapq
import math
from collections import defaultdict
from typing import Tuple

import cv2
import numpy as np
from occupancy_grid import OccupancyGrid


class Planner:
    """Simple occupancy grid Planner"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid
        self.map_walls = None

    def get_neighbors(self, current_cell):
        x, y = current_cell
        neighbor_list = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.grid.x_max_map and
                        0 <= ny < self.grid.y_max_map and
                        self.map_walls[nx, ny] < 10):
                    neighbor_list.append((nx, ny))
        return neighbor_list

    def heuristic(self, cell_1: Tuple[int, int], cell_2: Tuple[int, int]):
        return math.sqrt((cell_1[0] - cell_2[0])**2 + (cell_1[1] - cell_2[1])**2)

    def reconstruct_path(self, came_from, goal):
        total_path = [goal]
        cell = goal
        while cell in came_from.keys():
            cell = came_from[cell]
            total_path.insert(0, cell)

        total_path = np.array(total_path)
        traj_world_x, traj_world_y = self.grid.conv_map_to_world(total_path[:, 0], total_path[:, 1])
        return np.vstack((traj_world_x, traj_world_y))

    def plan(self, start, goal):
        start: Tuple[int, int] = self.grid.conv_world_to_map(start[0], start[1])
        goal: Tuple[int, int] = self.grid.conv_world_to_map(goal[0], goal[1])

        self.map_walls = copy.deepcopy(self.grid.occupancy_map)
        wall_mask = (self.map_walls > 5).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        wall_dilated = cv2.dilate(wall_mask, kernel)
        self.map_walls[wall_dilated > 0] = 100

        open_set = [(0.0, start)]
        heapq.heapify(open_set)

        came_from = {}

        g_score = defaultdict(lambda: math.inf)
        g_score[start] = 0.0

        f_score = defaultdict(lambda: math.inf)
        f_score[start] = self.heuristic(start, goal)

        while len(open_set) > 0:
            current_f, current_cell = heapq.heappop(open_set)
            if current_f > f_score[current_cell]:
                continue
            if current_cell == goal:
                return self.reconstruct_path(came_from, goal)

            for cell in self.get_neighbors(current_cell):
                tentative_g = g_score[current_cell] + self.heuristic(current_cell, cell)
                if tentative_g < g_score[cell]:
                    came_from[cell] = current_cell
                    g_score[cell] = tentative_g
                    f_score[cell] = tentative_g + self.heuristic(cell, goal)
                    heapq.heappush(open_set, (f_score[cell], cell))

        print('failed getting to objective')
        return None

    def explore_frontiers(self):
        goal = np.array([0, 0, 0])
        return goal
