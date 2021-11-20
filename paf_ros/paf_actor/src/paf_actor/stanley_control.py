
import math
import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

from helper_functions import calc_egocar_yaw, normalize_angle, calc_path_yaw

class StanleyLateralController(object):
    """StanleyLateral Controller implements Lateral control using a stanley controller

    """

    def __init__(self, k=.5, Kp=1.0, L=2.9, max_steer=30.0, min_speed=10):
        self.k = k  # control gain
        self.Kp = Kp  # speed proportional gain
        self.L = L  # [m] Wheel base of vehicle
        self.max_steer = np.deg2rad(max_steer) # maximum steering angle (degrees)
        self.min_speed = min_speed # minimum speed used for calculation, to avoid infinite values when standing still

    def run_step(self, currentPath, currentPose, currentSpeed):
        """Runs the Stanley-Controller calculations once

        Args:
            currentPath (nav_msgs.msg.Path): Path to follow
            currentPose (geometry_msgs.msg.PoseStamped): Pose of Ego Vehicle
            currentSpeed (float): speed of ego_vehicle

        Returns:
           float: steering angle
        """
        current_target_idx, error_front_axle = self.calc_target_index(currentPath, currentPose)
        # compute heading error correction
        theta_e = normalize_angle(calc_path_yaw(currentPath, current_target_idx) - calc_egocar_yaw(currentPose))
        if currentSpeed < self.min_speed:
            currentSpeed = self.min_speed
        # compute cross track error correction
        theta_d = np.arctan2(self.k * error_front_axle, currentSpeed)
        # compute steer      
        delta = theta_e + theta_d
        return np.clip(delta, -self.max_steer, self.max_steer)

    def calc_target_index(self, currentPath, currentPose):
        """Calculates the index of the closest Point on the Path relative to the front axle

        Args:
            currentPath (nav_msgs.msg.Path): Path to follow
            currentPose (geometry_msgs.msg.PoseStamped): Pose of Ego Vehicle

        Returns:
            target_idx [int]: index of target point
            error_front_axle [float]: distance from front axle to target point
        """

        if len(currentPath.poses) == 0:
            return 0, 0
        
        # Calc front axle position
        yaw = calc_egocar_yaw(currentPose)
        fx = currentPose.position.x + self.L * np.cos(yaw)
        fy = currentPose.position.y + self.L * np.sin(yaw)

        # Search nearest point index
        px = [posen.pose.position.x for posen in currentPath.poses]
        py = [posen.pose.position.y for posen in currentPath.poses]
        dx = [fx - icx for icx in px]
        dy = [fy - icy for icy in py]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(yaw + np.pi / 2),
                        -np.sin(yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
