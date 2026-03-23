""" A set of robotics control functions """

import random
import numpy as np


def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """
    laser_dist = lidar.get_sensor_values()
    ray_angles = lidar.get_ray_angles()

    # Zones angulaires
    front_mask = np.abs(ray_angles) < np.pi / 6
    left_mask = (ray_angles > 0) & (ray_angles < np.pi / 2)
    right_mask = (ray_angles < 0) & (ray_angles > -np.pi / 2)

    front_min = np.min(laser_dist[front_mask])
    left_mean = np.mean(laser_dist[left_mask])
    right_mean = np.mean(laser_dist[right_mask])

    if front_min < 150:
        # Obstacle ahead: slow down and turn toward the more open side
        speed = 0.1
        rotation_speed = 1.0 if left_mean > right_mean else -1.0
    elif front_min < 300:
        # Approaching obstacle: reduce speed, slight correction
        speed = 0.3
        rotation_speed = 0.5 if left_mean > right_mean else -0.5
    else:
        # Clear path
        speed = 0.5
        rotation_speed = 0.0

    command = {"forward": speed, "rotation": rotation_speed}
    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """
    # TODO for TP2

    command = {"forward": 0,
               "rotation": 0}

    return command
