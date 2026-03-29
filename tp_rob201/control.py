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
    """
    K_goal = 1.0
    K_obs = 5000000
    d_safe = 100.0
    d_stop = 20.0
    d_quad = 100.0

    # Gradient attractif
    diff = np.array(goal_pose[:2]) - np.array(current_pose[:2])
    d_goal = np.linalg.norm(diff)

    if d_goal < d_stop:
        return {"forward": 0.0, "rotation": 0.0}

    if d_goal > d_quad:
        grad_att = (K_goal / d_goal) * diff        # norme = K_goal = 1.0
    else:
        grad_att = (K_goal / d_quad) * diff        # quadratique, continuité assurée

    # Gradient répulsif (obstacle le plus proche)
    laser_dist = lidar.get_sensor_values()
    ray_angles = lidar.get_ray_angles()

    grad_rep = np.array([0.0, 0.0])
    min_idx = np.argmin(laser_dist)
    d_obs = laser_dist[min_idx]

    if d_obs < d_safe:
        angle_obs = ray_angles[min_idx] + current_pose[2]
        obs_vec = np.array([np.cos(angle_obs), np.sin(angle_obs)])
        grad_rep = -K_obs / (d_obs**3) * (1.0/d_obs - 1.0/d_safe) * obs_vec

    # Gradient total
    grad = grad_att + grad_rep
    grad_norm = np.linalg.norm(grad)

    if grad_norm < 1e-6:
        return {"forward": 0.0, "rotation": 0.0}

    # Commande
    grad_dir = np.arctan2(grad[1], grad[0])
    angle_err = grad_dir - current_pose[2]
    angle_err = (angle_err + np.pi) % (2 * np.pi) - np.pi

    forward = np.clip(grad_norm * max(0.0, np.cos(angle_err)), 0.0, 1.0)
    rotation = np.clip(angle_err / np.pi, -1.0, 1.0)

    return {"forward": forward, "rotation": rotation}

