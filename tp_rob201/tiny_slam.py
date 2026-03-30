""" A simple robotics navigation code including SLAM, exploration, planning"""

import cv2
import numpy as np
from occupancy_grid import OccupancyGrid


class TinySlam:
    """Simple occupancy grid SLAM"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

        # Origin of the odom frame in the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def _score(self, lidar, pose):
        """
        Computes the sum of log probabilities of laser end points in the map
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, position of the robot to evaluate, in world coordinates
        """
        # TODO for TP4

        score = 0

        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given,
                        use self.odom_pose_ref if not given
        """
        # TODO for TP4
        corrected_pose = odom_pose

        return corrected_pose

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        # TODO for TP4

        best_score = 0

        return best_score
    
    def update_map(self, lidar, pose):
        """
        Bayesian map update with new observation
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, corrected pose in world coordinates
        """
        laser_dist = lidar.get_sensor_values()
        ray_angles = lidar.get_ray_angles()

        x_r, y_r, theta_r = pose[0], pose[1], pose[2]

        angles_world = ray_angles + theta_r
        pts_x = x_r + laser_dist * np.cos(angles_world)
        pts_y = y_r + laser_dist * np.sin(angles_world)

        for i in range(0, len(laser_dist), 2):
            self.grid.add_value_along_line(
                x_r, y_r,
                x_r + 0.95 * laser_dist[i] * np.cos(angles_world[i]),
                y_r + 0.95 * laser_dist[i] * np.sin(angles_world[i]),
                -0.5
            )
            
            val_occ = 4.0 / (1.0 + laser_dist[i] / 100.0) 
            self.grid.add_map_points(np.array([pts_x[i]]), np.array([pts_y[i]]), val_occ)

        self.grid.occupancy_map = np.clip(self.grid.occupancy_map, -40, 40)



    def compute(self):
        """ Useless function, just for the exercise on using the profiler """
        # Remove after TP1

        ranges = np.random.rand(3600)
        ray_angles = np.arange(-np.pi, np.pi, np.pi / 1800)

        # Poor implementation of polar to cartesian conversion
        points = []
        for i in range(3600):
            pt_x = ranges[i] * np.cos(ray_angles[i])
            pt_y = ranges[i] * np.sin(ray_angles[i])
            points.append([pt_x, pt_y])