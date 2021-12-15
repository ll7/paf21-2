"""
A file that contains the Stanley Lateral Controller (inspired by PSAF WS20/21 2)
"""
import rospy
from typing import Tuple

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

        self._local_plan_publisher = rospy.Publisher("/paf/paf_actor/path", PafLocalPath, queue_size=1)

    def run_step(self, msg: PafLocalPath, pose: PoseStamped, speed: float, is_reverse: bool) -> float:
        """
        Runs the Stanley-Controller calculations once

        Args:
            currentPath (Path): Path to follow
            currentPose (PoseStamped): Pose of Ego Vehicle
            currentSpeed (float): speed of ego_vehicle
            is_reverse (bool): sets the stanley controller to steer backwards

        Returns:
           float: Steering angle
        """
        path = msg.points
        current_target_idx, error_front_axle, target_speed = self.calc_target_index(msg, pose, is_reverse)
        # compute heading error correction
        theta_e = normalize_angle(
            calc_path_yaw(path, current_target_idx) + (calc_egocar_yaw(pose) if is_reverse else -calc_egocar_yaw(pose))
        )
        if abs(speed) < self.min_speed:
            speed = self.min_speed

        # compute cross track error correction
        theta_d = np.arctan2(self.k * error_front_axle / max([1, 0.4 * speed]), speed)

        # compute steer
        delta = theta_e + theta_d

        # rospy.loginfo_throttle(
        #    1, f"theta_e: {theta_e}, theta_d: {theta_d}, delta: {delta}")

        return np.clip(delta, -self.max_steer, self.max_steer), target_speed

    def calc_target_index(self, msg: PafLocalPath, pose: PoseStamped, is_reverse: bool) -> Tuple[int, float]:
        """
        Calculates the index of the closest Point on the Path relative to the front axle

        Args:
            currentPath (LocalPath): Path to follow
            currentPose (PoseStamped): Pose of Ego Vehicle
            is_reverse (bool): bool if we drive backwards

        Returns:
            target_idx [int]: Index of target point
            error_front_axle [float]: Distance from front axle to target point
        """
        path = msg.points
        if len(path) == 0:
            return 0, 0, 0

        # Calc front axle position
        yaw = calc_egocar_yaw(pose)

        # msg.points = path
        # local_path1 = []
        #
        # pth = PafLocalPath()
        # pth.points = path_g
        # self._local_plan_publisher.publish(pth)

        fx, fy = 0, 0
        if is_reverse:
            fx = pose.position.x - self.L * np.cos(yaw)
            fy = pose.position.y - self.L * np.sin(yaw)
        else:
            fx = pose.position.x + self.L * np.cos(yaw)
            fy = pose.position.y + self.L * np.sin(yaw)

        # Search nearest point index
        px = [posen.x for posen in path]
        py = [posen.y for posen in path]
        dx = [fx - icx for icx in px]
        dy = [fy - icy for icy in py]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(yaw + np.pi / 2), -np.sin(yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle, msg.target_speed[target_idx]

    def _xy_to_point2d(self, points):
        liste = []
        for point in points:
            p = Point2D()
            p.x, p.y = point
            liste.append(p)
        return liste
