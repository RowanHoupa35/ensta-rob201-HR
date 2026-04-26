"""
Robot controller definition
Complete controller including SLAM, planning, path following
"""
import numpy as np

from place_bot.simulation.robot.robot_abstract import RobotAbstract
from place_bot.simulation.robot.odometer import OdometerParams
from place_bot.simulation.ray_sensors.lidar import LidarParams

from tiny_slam import TinySlam

from control import potential_field_control, reactive_obst_avoid
from occupancy_grid import OccupancyGrid
from planner import Planner


class MyRobotSlam(RobotAbstract):
    """A robot controller including SLAM, path planning and path following"""

    def __init__(self, lidar_params: LidarParams = LidarParams(), odometer_params: OdometerParams = OdometerParams()):
        super().__init__(lidar_params=lidar_params, odometer_params=odometer_params)

        self.counter = 0

        size_area = (1400, 1000)
        robot_position = (439.0, 195)
        self.occupancy_grid = OccupancyGrid(x_min=-(size_area[0] / 2 + robot_position[0]),
                                            x_max=size_area[0] / 2 - robot_position[0],
                                            y_min=-(size_area[1] / 2 + robot_position[1]),
                                            y_max=size_area[1] / 2 - robot_position[1],
                                            resolution=2)

        self.tiny_slam = TinySlam(self.occupancy_grid)
        self.planner = Planner(self.occupancy_grid)

        self.corrected_pose = np.array([0, 0, 0])
        self.goal = self._new_random_goal()
        self.d_stop = 60.0
        self.goal_timeout = 0
        self.goal_max_steps = 600

        # TP5
        self.EXPLORE_STEPS = 3000
        self.path = None          # array (2, N) en coordonnées monde, ou None
        self.path_index = 0

    def control(self):
        self.counter += 1
        pose = self.odometer_values()

        if self.counter % 5 == 0:
            if self.counter <= 300:
                self.corrected_pose = self.tiny_slam.get_corrected_pose(pose)
                self.tiny_slam.update_map(self.lidar(), self.corrected_pose)
            else:
                score = self.tiny_slam.localise(self.lidar(), pose)
                self.corrected_pose = self.tiny_slam.get_corrected_pose(pose)
                threshold = max(10, min(50, self.counter // 20))
                if score > threshold:
                    self.tiny_slam.update_map(self.lidar(), self.corrected_pose)
                if self.counter % 50 == 0:
                    print(f"Step {self.counter} - Score : {score:.1f}")

        if self.counter == self.EXPLORE_STEPS:
            origin = np.array([0.0, 0.0, 0.0])
            self.path = self.planner.plan(self.corrected_pose, origin)
            if self.path is not None:
                self.path_index = 0
                self.occupancy_grid.display_cv(self.corrected_pose, goal=origin, traj=self.path)

        if self.path is not None:
            if self.path_index >= self.path.shape[1]:
                print("Returned to origin!")
                return {"forward": 0.0, "rotation": 0.0}
            
            waypoint = self.path[:, self.path_index]
            dist = np.linalg.norm(self.corrected_pose[:2] - waypoint)
            if dist < self.d_stop:
                self.path_index += 1
                if self.path_index >= self.path.shape[1]:
                    print("Returned to origin!")
                    return {"forward": 0.0, "rotation": 0.0}
                waypoint = self.path[:, self.path_index]

            if self.counter % 80 == 0:
                self.occupancy_grid.display_cv(self.corrected_pose, goal=np.array([waypoint[0], waypoint[1], 0.0]), traj=self.path)

            return potential_field_control(self.lidar(), self.corrected_pose, np.array([waypoint[0], waypoint[1], 0.0]))

        if self.counter % 80 == 0:
            self.occupancy_grid.display_cv(self.corrected_pose, goal=self.goal)

        if self.counter < 300:
            return {"forward": 0.0, "rotation": 0.5}

        return self.control_tp2(pose)

    def control_tp1(self):
        self.tiny_slam.compute()
        return reactive_obst_avoid(self.lidar())

    def control_tp2(self, pose):
        diff = self.goal[:2] - pose[:2]
        self.goal_timeout += 1

        if np.linalg.norm(diff) < self.d_stop or self.goal_timeout > self.goal_max_steps:
            if np.linalg.norm(diff) < self.d_stop:
                print("Goal atteint !")
            else:
                print("Timeout, goal abandonné.")
            self.goal = self._new_random_goal()
            self.goal_timeout = 0
            print(f"Nouveau goal : {self.goal[:2]}")

        return potential_field_control(self.lidar(), pose, self.goal)

    def _new_random_goal(self):
        while True:
            x = np.random.uniform(-700, 80)
            y = np.random.uniform(-350, 120)
            if x < -558 and -417 < y < -264:
                continue
            if -432 < x < -302 and -226 < y < -126:
                continue
            if x < -561 and y > 25:
                continue
            return np.array([x, y, 0.0])
