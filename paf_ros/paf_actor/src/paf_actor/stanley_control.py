"""
A file that contains the Stanley Lateral Controller (inspired by PSAF WS20/21 2)
"""
from typing import List, Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped

from paf_actor.helper_functions import calc_egocar_yaw, normalize_angle, calc_path_yaw
from paf_messages.msg import PafLocalPath, Point2D


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

        self.heading_error: float = 0.0

        self._speed_to_change_k_factor: float = 70 / 3.6
        self._k_factor_change_at_high_speed: float = 2.0

    def run_step(
        self, msg: PafLocalPath, pose: PoseStamped, speed: float, is_reverse: bool
    ) -> Tuple[float, float, float]:
        """
        Runs the Stanley-Controller calculations once

        Args:
            msg (PafLocalPath): PafLocalPath message to follow
            pose (PoseStamped): Pose of the Ego Vehicle
            speed (float): speed of Ego Vehicle
            is_reverse (bool): sets the stanley controller to steer backwards

        Returns:
           steering_angle [float]: Steering angle
           target_speed [float]: The speed the vehicle should drive
           distance [float]: distance to the point we want to drive to
        """
        # get the path points of the message
        path: List[Point2D] = msg.points

        # get target point to drive to
        current_target_idx, error_front_axle, target_speed, distance = self.calc_target_index(msg, pose, is_reverse)

        # compute heading error correction (adjusted if reversed)
        theta_e: float = normalize_angle(
            calc_path_yaw(path, current_target_idx) + (calc_egocar_yaw(pose) if is_reverse else -calc_egocar_yaw(pose))
        )
        self.heading_error = theta_e

        # assures to not divide by too small numbers
        if abs(speed) < self.min_speed:
            speed: float = self.min_speed

        # adjust k parameter for steering stability at higher speeds
        if speed > self._speed_to_change_k_factor:
            k: float = self.k * self._k_factor_change_at_high_speed
        else:
            k: float = self.k

        # compute cross track error correction
        theta_d: float = np.arctan2(k * error_front_axle / max([1, 0.4 * speed]), speed)

        # compute steer
        delta: float = theta_e + theta_d

        # return the following: clip steering, the desired end speed, distance to point that is looked at
        return np.clip(delta, -self.max_steer, self.max_steer), target_speed, distance

    def calc_target_index(
        self, msg: PafLocalPath, pose: PoseStamped, is_reverse: bool
    ) -> Tuple[int, float, float, float]:
        """
        Calculates the index of the closest Point on the Path relative to the front axle

        Args:
            msg (PafLocalPath): PafLocalPath message to follow
            pose (PoseStamped): Pose of the Ego Vehicle
            is_reverse (bool): true if we drive backwards

        Returns:
            target_idx [int]: Index of target point
            error_front_axle [float]: Distance from front axle to target point
            target_speed [float]: The speed the vehicle should drive
            distance [float]: distance to the point we want to drive to
        """
        # get points of path to follow
        path: List[Point2D] = msg.points

        # handle edge case if there is no path or no target speeds
        if len(path) == 0 or len(msg.target_speed) == 0:
            return 0, 0, 0, 0

        # Calc front axle position
        yaw: float = calc_egocar_yaw(pose)

        # calculate reference point for the stanley controller
        fx: float = 0.0
        fy: float = 0.0
        if is_reverse:
            fx = pose.position.x - self.L * np.cos(yaw)
            fy = pose.position.y - self.L * np.sin(yaw)
        else:
            fx = pose.position.x + self.L * np.cos(yaw)
            fy = pose.position.y + self.L * np.sin(yaw)

        # Search nearest point index and distance to it
        px: List[float] = [posen.x for posen in path]
        py: List[float] = [posen.y for posen in path]
        dx: List[float] = [fx - icx for icx in px]
        dy: List[float] = [fy - icy for icy in py]
        d: np.ndarray = np.hypot(dx, dy)
        target_idx: int = np.argmin(d)
        distance: float = d[target_idx]

        # Project RMS error onto front axle vector
        front_axle_vec: List[float] = [-np.cos(yaw + np.pi / 2), -np.sin(yaw + np.pi / 2)]
        error_front_axle: float = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        # return:
        # point index on path to drive to
        # error of the front axle to disired path
        # the disired speed of the path that we look at
        # distance to the point we want to drive to
        return target_idx, error_front_axle, msg.target_speed[min([target_idx, len(msg.target_speed) - 1])], distance
