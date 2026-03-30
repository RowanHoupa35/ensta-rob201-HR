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


# Definition of our robot controller
class MyRobotSlam(RobotAbstract):
    """A robot controller including SLAM, path planning and path following"""

    def __init__(self,
                 lidar_params: LidarParams = LidarParams(),
                 odometer_params: OdometerParams = OdometerParams()):
        # Passing parameter to parent class
        super().__init__(lidar_params=lidar_params,
                         odometer_params=odometer_params)

        # step counter to deal with init and display
        self.counter = 0

        # Init SLAM object
        # Here we cheat to get an occupancy grid size that's not too large, by using the
        # robot's starting position and the maximum map size that we shouldn't know.
        size_area = (1400, 1000)
        robot_position = (439.0, 195)
        self.occupancy_grid = OccupancyGrid(x_min=-(size_area[0] / 2 + robot_position[0]),
                                            x_max=size_area[0] / 2 - robot_position[0],
                                            y_min=-(size_area[1] / 2 + robot_position[1]),
                                            y_max=size_area[1] / 2 - robot_position[1],
                                            resolution=2)

        self.tiny_slam = TinySlam(self.occupancy_grid)
        self.planner = Planner(self.occupancy_grid)

        # storage for pose after localization
        self.corrected_pose = np.array([0, 0, 0])
        self.goal = self._new_random_goal()
        self.d_stop = 60.0       # seuil d'arrivée plus large
        self.goal_timeout = 0    # compteur de steps sur le goal courant
        self.goal_max_steps = 600  # ~20s avant d'abandonner un goal inaccessible
                      

    def control(self): 
        self.counter += 1 
        pose = self.odometer_values()

        # Mise à jour carte tous les 5 steps
        if self.counter % 5 == 0:
            self.tiny_slam.update_map(self.lidar(), pose) 

        # Affichage tous les 50 steps
        if self.counter % 50 == 0:
            self.occupancy_grid.display_cv(pose, goal=self.goal)

        return self.control_tp2()




    def control_tp1(self):
        """
        Control function for TP1
        Control funtion with minimal random motion
        """
        self.tiny_slam.compute()

        # Compute new command speed to perform obstacle avoidance
        command = reactive_obst_avoid(self.lidar())
        return command

    def control_tp2(self):
        pose = self.odometer_values()

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
        """Génère un goal aléatoire en évitant les zones solides connues"""
        while True:
            x = np.random.uniform(-700, 80)
            y = np.random.uniform(-350, 120)
            # Exclure box 0 (bas-gauche)
            if x < -558 and -417 < y < -264:
                continue
            # Exclure box 1 (obstacle central)
            if -432 < x < -302 and -226 < y < -126:
                continue
            # Exclure box 3 (haut-gauche)
            if x < -561 and y > 25:
                continue
            return np.array([x, y, 0.0])




