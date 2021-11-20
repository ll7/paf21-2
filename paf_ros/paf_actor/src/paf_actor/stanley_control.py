"""
A file that contains the Stanley Lateral Controller (inspired by PSAF WS20/21 2)
"""
from typing import Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from paf_actor.helper_functions import calc_egocar_yaw, normalize_angle, calc_path_yaw


class StanleyLateralController:
    """
    StanleyLateral Controller implements Lateral control using a stanley controller
    """

    def __init__(self, k: float = 0.5, Kp: float = 1.0, L: float = 2.9, max_steer: float = 30.0, min_speed: float = 10):
        """
        Constructor of the stanley lateral controller.

        Args:
            k (float, optional): Control gain. Defaults to .5.
            Kp (float, optional): Speed proportional gain. Defaults to 1.0.
            L (float, optional): [m] Wheel base of vehicle. Defaults to 2.9.
            max_steer (float, optional): Maximum steering angle (degrees). Defaults to 30.0.
            min_speed (float, optional): Minimum speed used for calculation, to avoid infinite values
            when standing still. Defaults to 10.
        """
        self.k: float = k
        self.Kp: float = Kp
        self.L: float = L
        self.max_steer: float = np.deg2rad(max_steer)
        self.min_speed: float = min_speed

    def run_step(self, path: Path, pose: PoseStamped, speed: float) -> float:
        """
        Runs the Stanley-Controller calculations once

        Args:
            currentPath (Path): Path to follow
            currentPose (PoseStamped): Pose of Ego Vehicle
            currentSpeed (float): speed of ego_vehicle

        Returns:
           float: Steering angle
        """
        current_target_idx, error_front_axle = self.calc_target_index(path, pose)
        # compute heading error correction
        theta_e = normalize_angle(calc_path_yaw(path, current_target_idx) - calc_egocar_yaw(pose))
        if speed < self.min_speed:
            speed = self.min_speed
        # compute cross track error correction
        theta_d = np.arctan2(self.k * error_front_axle, speed)
        # compute steer
        delta = theta_e + theta_d
        return np.clip(delta, -self.max_steer, self.max_steer)

    def calc_target_index(self, path: Path, pose: PoseStamped) -> Tuple[int, float]:
        """
        Calculates the index of the closest Point on the Path relative to the front axle

        Args:
            currentPath (Path): Path to follow
            currentPose (PoseStamped): Pose of Ego Vehicle

        Returns:
            target_idx [int]: Index of target point
            error_front_axle [float]: Distance from front axle to target point
        """

        if len(path.poses) == 0:
            return 0, 0

        # Calc front axle position
        yaw = calc_egocar_yaw(pose)
        fx = pose.position.x + self.L * np.cos(yaw)
        fy = pose.position.y + self.L * np.sin(yaw)

        # Search nearest point index
        px = [posen.pose.position.x for posen in path.poses]
        py = [posen.pose.position.y for posen in path.poses]
        dx = [fx - icx for icx in px]
        dy = [fy - icy for icy in py]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(yaw + np.pi / 2), -np.sin(yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
