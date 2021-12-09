#!/usr/bin/env python
import time
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
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    REPLANNING_THRES_DISTANCE_M = 3
    DETECT_BEHIND_SIGNALS_M = 5
    TRANSMIT_FRONT_MIN_M = 100
    TRANSMIT_FRONT_SEC = 5
    CAR_DECELERATION = -9.81 * rospy.get_param("deceleration_g_factor", 8)  # m/s^2
    QUICK_BRAKE_EVENTS = [SignsDE.STOP.value]
    ROLLING_EVENTS = ["LIGHT", SignsDE.YIELD.value]
    DIST_TARGET_REACHED = 5
    UPDATE_HZ = 10
    REPLAN_THROTTLE_SEC = 3.0

    def __init__(self):

        rospy.init_node("local_path_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._current_point_index = 0
        self._target_speed = 250
        self._speed_limit = 250
        self._last_replan_request = time.perf_counter()
        self._global_path = []
        self._distances = []
        self._traffic_signals = []
        self._traffic_light_color = TrafficLightState.GREEN  # or None # todo fill this
        self._following_distance = -1  # todo set following distance
        self._following_speed = -1  # todo set following speed

        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated)
        rospy.Subscriber("/paf/paf_global_planner/routing_response", PafLaneletRoute, self._process_global_path)
        # rospy.Subscriber(rospy.get_param("global_path_topic"), PafLaneletRoute, self._process_global_path)

        # create and start the publisher for the local path
        self._local_plan_publisher = rospy.Publisher("/paf/paf_local_planner/path", PafLocalPath, queue_size=1)
        self._reroute_publisher = rospy.Publisher("/paf/paf_local_planner/reroute", Empty, queue_size=1)

    def _current_deceleration_distance(self, target_speed=0):
        deceleration = self.CAR_DECELERATION - 9.81 * np.sin(self._current_pitch)
        deceleration_time = (target_speed - self._current_speed) / deceleration
        delay = self._current_speed / self.UPDATE_HZ
        return 0.5 * (target_speed + self._current_speed) * deceleration_time + delay

    def _process_global_path(self, msg: PafLaneletRoute):
        if len(self._distances) == 0 or len(msg.distances) == 0 or msg.distances[-1] != self._distances[-1]:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distances[-1])}m")
        self._global_path = msg.points
        self._distances = msg.distances
        self._traffic_signals = msg.traffic_signals
        self._local_plan_publisher.publish(self._create_ros_msg())

    def _get_current_path(self) -> Tuple[List[Point], Dict[str, Tuple[int, float]]]:
        if len(self._global_path) == 0:
            return self._global_path, {}
        index = self._current_point_index
        try:
            d_ref = self._distances[index]
        except IndexError:
            d_ref = 0
        try:
            delta = self._distances[index + 1] - d_ref
            travel_dist = max(self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * self._current_speed)
            index_end = int(np.ceil(travel_dist / delta)) + index
            _ = self._distances[index_end]
        except IndexError:  # todo end of route handling
            # if (
            #         self._dist(
            #             (self._global_path[-1].x, self._global_path[-1].y),
            #             (self._current_pose.position.x, self._current_pose.position.y),
            #         )
            #         > self.DIST_TARGET_REACHED
            # ):
            #     self._send_global_path_request()
            index_end = len(self._distances)

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

    @staticmethod
    def _dist(a, b):
        x1, y1 = a
        x2, y2 = b
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _closest_index_of_point_list(self, pts_list: List[Point], target_pt: Tuple[float, float]):

        if len(self._global_path) == 0:
            return -1
        return int(np.argmin([self._dist([p.x, p.y], target_pt) for p in pts_list]))

    def _create_ros_msg(self):
        """create path message for ros
        Returns:
            [type]: [description]
        """
        current_path, signals = self._get_current_path()
        self._update_target_speed(signals)
        path_msg = PafLocalPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.target_speed = self._target_speed
        path_msg.points = current_path

        return path_msg

    def _update_target_speed(self, signals: dict):
        braking_distance_zero = self._current_deceleration_distance(target_speed=0)
        for signal_type, (distance, value) in signals.items():
            should_stop_now = distance <= braking_distance_zero
            if signal_type in self.QUICK_BRAKE_EVENTS:
                if should_stop_now:
                    if self._is_stopped() and self._can_continue_on_path(signal_type):
                        self._target_speed = self._speed_limit
                    else:
                        self._target_speed = 0

            elif signal_type in self.ROLLING_EVENTS:
                if should_stop_now:
                    if self._can_continue_on_path(signal_type):
                        self._target_speed = self._speed_limit
                    else:
                        self._target_speed = 0

            elif signal_type == SignsDE.MAX_SPEED.value:
                braking_distance_to_speed_limit = self._current_deceleration_distance(target_speed=value)
                if distance <= braking_distance_to_speed_limit:
                    self._speed_limit = value  # m/s (km/h -> m/s is done in PafRoute)

            else:
                rospy.logerr_throttle(
                    10,
                    f"[local planner] unknown signal detected: type={signal_type} at d={distance} "
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
        _, self._current_pitch, _ = euler_from_quaternion(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        )
        self._current_pose = odometry.pose.pose
        # invert y-coordinate of odometry, because odometry sensor returns wrong values
        self._current_pose.position.y = -self._current_pose.position.y

        if len(self._global_path) <= self._current_point_index + 1:
            self._current_point_index = 0

        current_pos = self._current_pose.position

        prev_dist = None
        # calculate distance to current next point
        idx = self._current_point_index
        for i, _ in enumerate(self._global_path[idx:]):
            next_point = self._global_path[i]
            distance = self._dist((current_pos.x, current_pos.y), (next_point.x, next_point.y))
            if prev_dist is None or prev_dist > distance:
                prev_dist = distance
            else:
                self._current_point_index = i - 1
                break

    def _on_global_path(self):
        p = (self._current_pose.position.x, self._current_pose.position.y)
        i = self._current_point_index
        try:
            return self._dist(p, (self._global_path[i].x, self._global_path[i].y)) < self.REPLANNING_THRES_DISTANCE_M
        except IndexError:
            return False

    def _send_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new global route")
            self._reroute_publisher.publish(Empty())

    def start(self):

        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            if not self._on_global_path():
                self._send_global_path_request()
            else:
                rospy.loginfo_throttle(30, "[local planner] car is on route, no need to reroute")

            self._local_plan_publisher.publish(self._create_ros_msg())
            rate.sleep()


if __name__ == "__main__":
    LocalPlanner().start()

# rosservice call /paf_global_planner/routing_request "start:
# - -85.0
# - -75.0
# start_yaw: 1.56
# target:
# - -180.0
# - 180.0
# resolution: 0.0"
