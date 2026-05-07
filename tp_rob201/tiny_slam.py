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
        laser_dist = lidar.get_sensor_values()
        ray_angles = lidar.get_ray_angles()

        max_range = lidar.max_range if hasattr(lidar, 'max_range') else 290.0
        valid = laser_dist < max_range * 0.99
        laser_dist = laser_dist[valid]
        ray_angles = ray_angles[valid]

        if len(laser_dist) == 0:
            return 0

        x_r, y_r, theta_r = pose
        angles_world = ray_angles + theta_r
        pts_x = x_r + laser_dist * np.cos(angles_world)
        pts_y = y_r + laser_dist * np.sin(angles_world)

        x_map, y_map = self.grid.conv_world_to_map(pts_x, pts_y)

        # Supprimer les points hors de la carte
        valid = ((x_map >= 0) & (x_map < self.grid.x_max_map) & (y_map >= 0) & (y_map < self.grid.y_max_map))
        x_map = x_map[valid]
        y_map = y_map[valid]

        if len(x_map) == 0:
            return 0

        return np.sum(self.grid.occupancy_map[x_map, y_map])


    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given,
                        use self.odom_pose_ref if not given
        """
        if odom_pose_ref is None:
            odom_pose_ref = self.odom_pose_ref

        x_ref, y_ref, theta_ref = odom_pose_ref
        x_o, y_o, theta_o = odom_pose

        # Rotation + translation : repère odom → repère monde
        x_world = x_ref + x_o * np.cos(theta_ref) - y_o * np.sin(theta_ref)
        y_world = y_ref + x_o * np.sin(theta_ref) + y_o * np.cos(theta_ref)
        theta_world = theta_ref + theta_o

        return np.array([x_world, y_world, theta_world])


    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        N        = 50    # échantillons par itération
        elite_k  = 10   # nombre d'élites gardées (20%)
        n_iter   = 3    # itérations CE
        sigma_min = np.array([1.0, 1.0, 0.003])  # sigma plancher

        mu    = self.odom_pose_ref.copy().astype(float)
        sigma = np.array([15.0, 15.0, 0.03])

        best_score = self._score(lidar, self.get_corrected_pose(raw_odom_pose, mu))
        best_pose  = mu.copy()


        for _ in range(n_iter):
            # 1. Échantillonner N candidats autour de mu
            noise      = np.random.randn(N, 3) * sigma
            candidates = mu + noise

            # 2. Évaluer chaque candidat
            scores = np.zeros(N)
            for i, candidate in enumerate(candidates):
                corrected   = self.get_corrected_pose(raw_odom_pose, candidate)
                scores[i]   = self._score(lidar, corrected)

            # 3. Sélectionner les élites (meilleurs scores)
            elite_idx = np.argsort(scores)[-elite_k:]
            elites    = candidates[elite_idx]

            # 4. Mettre à jour mu et sigma depuis les élites
            mu    = elites.mean(axis=0)
            sigma = np.maximum(elites.std(axis=0), sigma_min)

            # Garder trace du meilleur global
            if scores[elite_idx[-1]] > best_score:
                best_score = scores[elite_idx[-1]]
                best_pose  = candidates[elite_idx[-1]]

        self.odom_pose_ref = best_pose
        return best_score
    
    
    def update_map(self, lidar, pose):
        laser_dist = lidar.get_sensor_values()
        ray_angles = lidar.get_ray_angles()

        x_r, y_r, theta_r = pose[0], pose[1], pose[2]

        angles_world = ray_angles + theta_r
        pts_x = x_r + laser_dist * np.cos(angles_world)
        pts_y = y_r + laser_dist * np.sin(angles_world)

        for i in range(len(laser_dist)):          # tous les rayons
            self.grid.add_value_along_line(
                x_r, y_r,
                x_r + 0.95 * laser_dist[i] * np.cos(angles_world[i]),
                y_r + 0.95 * laser_dist[i] * np.sin(angles_world[i]),
                -1.5                               # espace libre plus marqué
            )

            val_occ = 8.0 / (1.0 + laser_dist[i] / 100.0)   # obstacle plus fort
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