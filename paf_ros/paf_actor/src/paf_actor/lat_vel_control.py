"""
A file that contains the Stanley Lateral Controller (inspired by PSAF WS20/21 2)
"""
# from struct import error
import rospy

import numpy as np
from geometry_msgs.msg import PoseStamped

from paf_actor.helper_functions import calc_egocar_yaw, calc_path_yaw
from paf_messages.msg import PafLocalPath


class LatVelController:
    def __init__(self, K_theta: float = 1.0, L: float = 2.9, max_steer: float = 30.0, min_speed: float = 0.01):
        self.K_theta: float = K_theta
        self.L: float = L
        self.max_steer: float = np.deg2rad(max_steer)
        self.min_speed: float = min_speed

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

        current_target_idx, error_back_axle, target_speed, distance = self.calc_target_index(msg, pose, is_reverse)

        path_curvature_at_point = calc_path_yaw(path, current_target_idx)
        # theta_p = normalize_angle(
        #    path_curvature_at_point + (calc_egocar_yaw(pose) if is_reverse else -calc_egocar_yaw(pose))
        # )
        theta_p = calc_egocar_yaw(pose)

        klat = 0.2
        # steering_angle = np.arctan(self.L * (-self.K_theta * np.sin(theta_p) - (self.K_theta * klat * distance)/(
        #    np.max([speed, self.min_speed])) + (error_back_axle * np.cos(theta_p))/(1 - error_back_axle*distance)))

        self.K_theta = 0.25
        heading_error = -self.K_theta * np.sin(theta_p)
        distance_error = -(self.K_theta * klat * distance) / np.max([speed, self.min_speed])
        baxle_error = (path_curvature_at_point * np.cos(theta_p)) / (1 - path_curvature_at_point * distance)

        steering_angle = np.arctan(self.L * (distance_error + baxle_error))

        rospy.loginfo_throttle(5, f"heading_error: {heading_error}")
        rospy.loginfo_throttle(5, f"distance error: {distance_error}")
        rospy.loginfo_throttle(5, f"baxle error: {baxle_error}")
        rospy.loginfo_throttle(5, f"path_curavture: {path_curvature_at_point}")

        rospy.loginfo_throttle(
            5, f"steering angle: {np.rad2deg(np.clip(steering_angle, -self.max_steer, self.max_steer))}\n"
        )
        return np.clip(steering_angle, -self.max_steer, self.max_steer), target_speed, distance

    def calc_target_index(self, msg: PafLocalPath, pose: PoseStamped, is_reverse: bool):
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
        if len(path) == 0 or len(msg.target_speed) == 0:
            return 0, 0, 0, 0

        # Calc front axle position
        yaw = calc_egocar_yaw(pose)

        fx, fy = 0, 0
        if is_reverse:
            fx = pose.position.x + self.L * np.cos(yaw)
            fy = pose.position.y + self.L * np.sin(yaw)
        else:
            fx = pose.position.x - self.L * np.cos(yaw)
            fy = pose.position.y - self.L * np.sin(yaw)

        # Search nearest point index
        px = [posen.x for posen in path]
        py = [posen.y for posen in path]
        dx = [fx - icx for icx in px]
        dy = [fy - icy for icy in py]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        distance = d[target_idx]

        # Project RMS error onto back axle vector
        back_axle_vec = [-np.cos(yaw + np.pi / 2), -np.sin(yaw + np.pi / 2)]
        error_back_axle = np.dot([dx[target_idx], dy[target_idx]], back_axle_vec)

        return target_idx, error_back_axle, msg.target_speed[min([target_idx, len(msg.target_speed) - 1])], distance
