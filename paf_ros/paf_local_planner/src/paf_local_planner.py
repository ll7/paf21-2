#!/usr/bin/env python
from typing import List, Tuple, Dict

from commonroad.scenario.traffic_sign import (
    TrafficSignIDGermany as SignsDE,
    # TrafficSignIDUsa as SignsUS,
    TrafficLightState,
)

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLocalPath, Point2D as Point, PafLaneletRoute
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    DETECT_BEHIND_SIGNALS_M = 5
    TRANSMIT_FRONT_M = 150
    CAR_DECELERATION = -9.81 * rospy.get_param("deceleration_g_factor")  # m/s^2
    QUICK_BRAKE_EVENTS = [SignsDE.STOP]
    ROLLING_EVENTS = ["LIGHT", SignsDE.YIELD]

    UPDATE_HZ = 10

    def __init__(self):
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._target_speed = 250
        self._speed_limit = 250
        self._global_path = []
        self._traffic_signals = []
        self._traffic_light_color = TrafficLightState.GREEN  # or None # todo fill this
        self._following_distance = -1  # todo set following distance
        self._following_speed = -1  # todo set following speed

        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated)
        rospy.Subscriber(rospy.get_param("global_path_topic"), PafLaneletRoute, self._process_global_path)

        # create and start the publisher for the local path
        self.local_plan_publisher = rospy.Publisher(rospy.get_param("local_path_topic"), PafLocalPath, queue_size=1)

    def _current_deceleration_distance(self, target_speed=0):
        deceleration = self.CAR_DECELERATION - 9.81 * np.sin(self._current_pitch)
        deceleration_time = (target_speed - self._current_speed) / deceleration
        delay = self._current_speed * 1 / self.UPDATE_HZ
        return 0.5 * (target_speed + self._current_speed) * deceleration_time + delay

    def _process_global_path(self, msg: PafLaneletRoute):
        self._global_path = msg.points
        self._distances = msg.distances
        self._traffic_signals = msg.traffic_signals

    def _get_current_path(self) -> Tuple[List[Point], Dict[Tuple[int, float]]]:
        index = self._closest_index_in_path()
        d_ref = self._distances[index]
        d_transmit = d_ref + self.TRANSMIT_FRONT_M
        index_end = np.argmin([np.abs(d_transmit - d) for d in self._distances]) + 1
        signals = {}
        for s in self._traffic_signals:
            if s.index < index and d_ref - self._distances[s.index] > self.DETECT_BEHIND_SIGNALS_M:
                continue
            if s.index > index_end:
                continue
            if s.type in signals:
                continue
            signals[s.type] = (self._distances[s.index] - d_ref, s.value)
        return self._global_path[index:index_end], signals

    def _closest_index_in_path(self) -> int:
        """
        returns the closest index of a point in the local path to the vehicle's current position
        :return index
        """
        my_position = (self._current_pose.position.x, self._current_pose.position.y)
        return self._closest_index_of_point_list(self._global_path, my_position)

    def _closest_index_of_point_list(self, pts_list: List[Point], target_pt: Tuple[float, float]):
        def dist(a, b):
            x1, y1 = a
            x2, y2 = b
            return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        if len(self._global_path) == 0:
            return 0
        return int(np.argmin([dist([p.x, p.y], target_pt) for p in pts_list]))

    def _create_ros_msg(self):
        """create path message for ros

        Returns:
            [type]: [description]
        """
        current_path, signals = self._get_current_path()
        self._update_target_speed(signals)
        path_msg = PafLocalPath()
        path_msg.header.frame_id = "local path"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.target_speed = self._target_speed
        path_msg.points = current_path

        return path_msg

    def _update_target_speed(self, signals: dict):
        braking_distance_zero = self._current_deceleration_distance(target_speed=0)
        for signal_type, (distance, value) in signals.items():
            should_stop_now = distance <= braking_distance_zero

            if signal_type in self.QUICK_BRAKE_EVENTS and should_stop_now:
                if self._is_stopped() and self._can_continue_on_path(signal_type):
                    self._target_speed = self._speed_limit
                else:
                    self._target_speed = 0

            elif signal_type in self.ROLLING_EVENTS and should_stop_now:
                if self._can_continue_on_path(signal_type):
                    self._target_speed = self._speed_limit
                else:
                    self._target_speed = 0

            elif signal_type == SignsDE.MAX_SPEED:
                braking_distance_to_speed_limit = self._current_deceleration_distance(target_speed=value)
                if distance <= braking_distance_to_speed_limit:
                    self._speed_limit = value  # m/s (km/h -> m/s is done in PafRoute)

            else:
                rospy.logerr_throttle(
                    1,
                    f"unknown sign detected: type={signal_type} at d={distance} "
                    f"(Check TrafficSignIDGermany or TrafficSignIDUsa for more information)",
                )

        if self._following_distance > 0:
            reduce_speed = (
                self._current_deceleration_distance(target_speed=self._following_speed) <= self._following_distance
            )
            if reduce_speed:
                self._target_speed = self._following_speed

    def _can_continue_on_path(self, signal_type):
        if signal_type == "LIGHT":
            return self._traffic_light_color == TrafficLightState.GREEN
        return True  # todo joining lanes / lane change free ??

    def _is_stopped(self):
        margin = 0.5
        return -margin < self._current_speed < margin

    def _odometry_updated(self, odometry: Odometry):
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
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            self.local_plan_publisher.publish(self._create_ros_msg())
            rate.sleep()


if __name__ == "__main__":
    LocalPlanner().start()
