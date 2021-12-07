#!/usr/bin/env python
from typing import List, Tuple, Dict

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLocalPath, Point2D as Point, PafLaneletRoute
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    CAR_DECELERATION = -9.81 * rospy.get_param("deceleration_g_factor")  # m/s^2

    def __init__(self):
        self.role_name = rospy.get_param("~role_name", "ego_vehicle")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._target_speed = 180
        self._global_path = []
        self._traffic_signals = []

        rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_updated)
        rospy.Subscriber(rospy.get_param("global_path_topic"), PafLaneletRoute, self._process_global_path)

        # create and start the publisher for the local path
        self.local_plan_publisher = rospy.Publisher(rospy.get_param("local_path_topic"), PafLocalPath, queue_size=1)

    def _current_deceleration_distance(self, target_speed=0):
        deceleration = self.CAR_DECELERATION - 9.81 * np.sin(self._current_pitch)
        deceleration_time = (target_speed - self._current_speed) / deceleration
        return 0.5 * (target_speed + self._current_speed) * deceleration_time

    def _process_global_path(self, msg: PafLaneletRoute):
        self._global_path = msg.points
        self._distances = msg.distances
        self._traffic_signals = msg.traffic_signals

    def get_current_path(self) -> Tuple[List[Point], Dict[Tuple[int, float]]]:
        index = self.closest_index_in_path()
        d_ref = self._distances[index]
        signals = {}
        for s in self._traffic_signals:
            if s.index < index:
                continue
            if s.type in signals:
                continue
            signals[s.type] = self._distances[s.index] - d_ref, s.value
        return self._global_path[index:], signals

    def closest_index_in_path(self) -> int:
        """
        returns the closest index of a point in the local path to the vehicle's current position
        :return index
        """

        def dist(a, b):
            x1, y1 = a
            x2, y2 = b
            return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        if len(self._global_path) == 0:
            return 0
        my_position = [self._current_pose.position.x, self._current_pose.position.y]
        return int(np.argmin([dist([p.x, p.y], my_position) for p in self._global_path]))

    def create_ros_msg(self):
        """create path message for ros

        Returns:
            [type]: [description]
        """
        current_path, signals = self.get_current_path()
        self._update_target_speed(signals)
        path_msg = PafLocalPath()
        path_msg.header.frame_id = "local path"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.target_speed = self._target_speed
        path_msg.points = current_path

        return path_msg

    def odometry_updated(self, odometry: Odometry):
        """Odometry Update Callback"""

        # calculate current speed (m/s) from twist
        self._current_speed = np.sqrt(
            odometry.twist.twist.linear.x ** 2 + odometry.twist.twist.linear.y ** 2 + odometry.twist.twist.linear.z ** 2
        )
        _, self._current_pitch, _ = euler_from_quaternion(odometry.pose.orientation)
        self._current_pose = odometry.pose.pose
        # invert y-coordinate of odometry, because odometry sensor returns wrong values
        self._current_pose.position.y = -self._current_pose.position.y

    def start(self):
        rospy.init_node("local_path_node", anonymous=True)
        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            self.local_plan_publisher.publish(self.create_ros_msg())
            rate.sleep()


if __name__ == "__main__":
    LocalPlanner().start()
