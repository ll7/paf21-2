#!/usr/bin/env python
import time
from collections import deque
from typing import List

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.traffic_sign import (
    TrafficLightState,
    TrafficSignIDGermany,
)

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import (
    PafLocalPath,
    PafLaneletRoute,
    PafTrafficSignal,
    PafTopDownViewPointSet,
    Point2D,
    PafSpeedMsg, PafLanelet, PafLaneletMatrix,
)
from classes.SpeedCalculator import SpeedCalculator
from classes.HelperFunctions import (
    dist,
    closest_index_of_point_list,
    get_angle_between_vectors,
    xy_from_distance_and_angle, k_closest_indices_of_point_in_list, find_closest_lanelet, xy_to_pts, dist_pts,
)
from classes.Spline import calc_spline_course, calc_spline_course_from_point_list
from classes.MapManager import MapManager
from std_msgs.msg import Bool, Empty
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    STEP_SIZE = .125
    REPLANNING_THRES_DISTANCE_M = 15
    TRANSMIT_FRONT_MIN_M = 100
    TRANSMIT_FRONT_SEC = 5
    UPDATE_HZ = 10
    REPLAN_THROTTLE_SEC = 5
    END_OF_ROUTE_SPEED = 5  # todo remove slowdown at end of route
    MAX_ANGULAR_ERROR = np.deg2rad(45)

    rules_enabled = rospy.get_param("rules_enabled", False)

    def __init__(self):

        rospy.init_node("local_path_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        rospy.logwarn(f"[local planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._current_yaw = 0

        self._current_indices = 0, 0, 0
        self._target_indices = 0, 0, 0

        self._current_speed_limit = 50 / 3.6

        self._global_path = []
        self._global_path_length = 0
        self._global_path_lanelets = []
        # self._distances = []
        # self._curve_speed = []
        self._traffic_signals = []  # todo remove
        self._traffic_light_color = TrafficLightState.GREEN  # or None # todo fill this
        # self._following_distance = -1  # todo set following distance
        # self._following_speed = -1  # todo set following speed
        # self._distance_to_global_path = 0
        self._last_local_reroute = rospy.Time.now().to_time()
        self._speed_msg = PafSpeedMsg()

        self._network: LaneletNetwork = MapManager.get_current_scenario().lanelet_network

        # local path params
        self._local_path = []
        self._target_speed = []

        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        rospy.Subscriber(
            "/paf/paf_global_planner/routing_response", PafLaneletRoute, self._process_global_path, queue_size=1
        )
        rospy.Subscriber("/paf/paf_local_planner/rules_enabled", Bool, self._change_rules, queue_size=1)
        self._last_replan_request = time.perf_counter()
        # create and start the publisher for the local path
        self._local_plan_publisher = rospy.Publisher("/paf/paf_local_planner/path", PafLocalPath, queue_size=1)
        self._reroute_publisher = rospy.Publisher("/paf/paf_local_planner/reroute", Empty, queue_size=1)
        self._reroute_random_publisher = rospy.Publisher(
            "/paf/paf_local_planner/routing_request_random", Bool, queue_size=1
        )
        self._sign_publisher = rospy.Publisher(
            "/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1
        )
        self._speed_msg_publisher = rospy.Publisher("/paf/paf_validation/speed_text", PafSpeedMsg, queue_size=1)

    def _change_rules(self, msg: Bool):
        rospy.set_param("rules_enabled", msg.data)
        if msg.data == self.rules_enabled:
            return
        self.rules_enabled = msg.data
        rospy.logwarn(
            f"[local planner] Rules are now {'en' if self.rules_enabled else 'dis'}abled! "
            f"Speed limits will change after starting a new route."
        )

    def _process_global_path(self, msg: PafLaneletRoute):
        if self._global_path_length != msg.distance:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distance)}m")
        self._global_path = msg.sections
        self._global_path_length = msg.distance
        self._target_indices = self._get_matrix_and_lane_indices(msg.target)
        self._global_path_lanelets = []
        for matrix in msg.sections:
            for lane in matrix.lanes:
                self._global_path_lanelets.append(lane.lanelet_id)
        self._create_paf_local_path_msg()

    def _get_track_angle(self, index):
        if len(self._global_path) == 0:
            return 0
        if index == len(self._global_path) - 1:
            index -= 1
        p1 = self._global_path[index]
        p2 = self._global_path[index + 1]
        v2 = [p2.x - p1.x, p2.y - p1.y]
        return get_angle_between_vectors(v2)

    def _get_turning_path_to(self, index, track_angle, track_err, distance):
        def is_left_of_target_line():
            a = p_target_1
            b = p_target_2
            c = self._current_pose.position
            return (b.x - a.x) * (c.y - a.y) > (b.y - a.y) * (c.x - a.x)

        index_forwards = int(min(len(self._global_path) - 1, index + 10 / 0.125))
        p_target_1 = self._global_path[index_forwards - 1]
        p_target_2 = self._global_path[index_forwards]

        # d1, d2, radius = 4, 10, -5
        # turning_dir = 1

        x_list, y_list = [], []
        pts = [
            xy_from_distance_and_angle(self._current_pose.position, 0, -self._current_yaw),
            xy_from_distance_and_angle(self._current_pose.position, 4, -self._current_yaw),
            xy_from_distance_and_angle(p_target_1, 5, track_angle + np.pi / 2),
        ]

        for pt in pts:
            x_list.append(pt[0])
            y_list.append(pt[1])
        x_list = x_list[:2] + [p_target_1.x] + x_list[2:]  # , p_target_2.x]
        y_list = y_list[:2] + [p_target_1.y] + y_list[2:]  # , p_target_2.y]

        x_list_out, y_list_out, _, _, _ = calc_spline_course(x_list, y_list, 0.125)

        self._local_path = [Point2D(x, y) for x, y in zip(x_list_out, y_list_out)]

        self._target_speed = [1 for _ in self._local_path]

    def _create_paf_local_path_msg(self, send_empty=False):
        """create path message for ros
        Returns:
            [type]: [description]
        """
        pth, speeds, is_new_msg = self._get_current_trajectory()

        if is_new_msg:
            path_msg = PafLocalPath()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.target_speed = [] if send_empty else list(speeds)
            path_msg.points = [] if send_empty else pth

            while 0 < len(path_msg.target_speed) < len(path_msg.points):  # threading issue
                path_msg.target_speed.append(path_msg.target_speed[-1])

            self._local_plan_publisher.publish(path_msg)

    def _get_current_trajectory(self):
        m_idx, l_idx, p_idx = self._set_current_indices()
        last_local_reroute = rospy.Time.now().to_time()
        delta_t = last_local_reroute - self._last_local_reroute
        if len(self._local_path) > 0 and self._planner_at_end_of_route(self._local_path):
            self._end_of_route_handling()
            self._local_path = []
            self._target_speed = []
            self._global_path = []
            is_new_message = True
        elif len(self._global_path) > 0 and m_idx >= 0 and (
                delta_t > self.REPLAN_THROTTLE_SEC / 2 or m_idx == len(self._global_path) - 1
        ):
            self._set_current_path()
            is_new_message = True
            rospy.loginfo_throttle(30, "[local planner] local planner is replanning")
            self._last_local_reroute = last_local_reroute
        else:
            is_new_message = False
        return self._local_path, self._target_speed, is_new_message

    def _signal_debug_print(self, signals):

        pts1 = PafTopDownViewPointSet()
        pts1.label = "signals"
        pts1.points = [
            self._global_path[s.index]
            for s in signals
            if s.type in SpeedCalculator.ROLLING_EVENTS + SpeedCalculator.QUICK_BRAKE_EVENTS
        ]
        pts1.color = [255, 0, 0]
        self._sign_publisher.publish(pts1)

        pts2 = PafTopDownViewPointSet()
        pts2.label = "speed_signs"
        pts2.points = [
            self._global_path[s.index]
            for s in signals
            if s.type == TrafficSignIDGermany.MAX_SPEED.value or s.type == "MERGE"
        ]
        pts2.color = (255, 204, 0)
        self._sign_publisher.publish(pts2)

        out = []
        for s in signals:
            if self._current_point_index <= s.index < end_idx:
                try:
                    n = TrafficSignIDGermany(s.type).name
                except Exception:
                    n = s.type
                m = self._distances[s.index] - self._distances[self._current_point_index]
                m = np.round(m, 1)
                str_signal = f"{n} ({m}m)"
                if s.value >= 0:
                    v = np.round(s.value, 1)
                    if n == "MAX_SPEED":
                        v = np.round(v * 3.6, 1)
                    str_signal += f": {v}"
                out.append(str_signal)
        if len(out) > 0:
            rospy.loginfo_throttle(2, f"[local planner] Upcoming Traffic Signs: {', '.join(out)}")

    def _speed_debug_print(self, speeds, number_of_values=100):
        if len(self._distances) == 0:
            return
        step = self._distances[-1] / len(self._distances)
        delta_m = 1  # meter
        delta_idx = int(delta_m / step)
        n = number_of_values * delta_idx
        out = []
        for speed in speeds[:n:delta_idx]:
            msg = f"{np.round(speed * 3.6, 1)}"
            out.append(msg)
        if len(out) > 0:
            rospy.loginfo_throttle(1, f"[local planner] Upcoming Speeds: {', '.join(out)}")

    def _allowed_from_stop(self):
        return True  # todo

    def _can_continue_on_path(self, signal_type):
        if signal_type == "LIGHT":
            return self._traffic_light_color == TrafficLightState.GREEN
        return True  # todo joining lanes / lane change free ??

    def _is_stopped(self):
        margin = 0.5
        stop = -margin < self._current_speed < margin

        rospy.loginfo_throttle(5, stop)
        return stop

    def _planner_at_end_of_route(self, pth=None):
        if pth is not None:
            return len(pth) < 100
        if len(self._global_path) == 0:
            return False

        p_m, p_l, p_pt = self._current_indices
        return p_m == len(self._global_path) - 1 and p_pt == len(self._global_path[p_m].lanes[0]) - 1

    def _odometry_updated(self, odometry: Odometry):
        """Odometry Update Callback"""

        # calculate current speed (m/s) from twist
        self._current_speed = np.sqrt(
            odometry.twist.twist.linear.x ** 2 + odometry.twist.twist.linear.y ** 2 + odometry.twist.twist.linear.z ** 2
        )
        _, self._current_pitch, self._current_yaw = euler_from_quaternion(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        )
        self._current_pose = odometry.pose.pose

    def _get_matrix_and_lane_indices(self, position):
        current_lanelets = find_closest_lanelet(self._network, position)
        if hasattr(position, "x"):
            ref = (position.x, position.y)
        else:
            ref = position
        found = []
        found_pt = []
        for i, matrix in enumerate(self._global_path):
            lane: PafLanelet
            for j, lane in enumerate(matrix.lanes):
                if lane.lanelet_id in current_lanelets:
                    current_matrix_idx = i
                    current_lane_idx = j
                    try:
                        idx, d = k_closest_indices_of_point_in_list(2, lane.points, ref)
                        current_lane_pt_idx = max(idx)
                    except ValueError:
                        current_lane_pt_idx = 0
                    found_pt += [lane.points[current_lane_pt_idx]]
                    found.append((current_matrix_idx, current_lane_idx, current_lane_pt_idx))
        if len(found_pt) == 0:
            found = -1, -1, -1
        elif len(found_pt) == 1:
            found = found[0]
        else:
            idx = k_closest_indices_of_point_in_list(2, found_pt, ref)
            found = found[max(idx)]
        return found

    def _set_current_indices(self):
        self._current_indices = self._get_matrix_and_lane_indices(self._current_pose.position)
        return self._current_indices

    # def _publish_speed_msg(self):
    #     # idx = self._current_point_index + 100
    #     try:
    #         limit = self._speed_msg.limit if self._speed_msg.limit > 0 else 250
    #         sign: PafTrafficSignal
    #         for sign in self._traffic_signals:
    #             if sign.index > idx:
    #                 break
    #             if sign.type == TrafficSignIDGermany.MAX_SPEED.value:
    #                 limit = sign.value
    #             if sign.type in SpeedCalculator.SPEED_LIMIT_RESTORE_EVENTS:
    #                 limit = SpeedCalculator.CITY_SPEED_LIMIT
    #         target = speed[idx]
    #         if limit != self._speed_msg.limit or target != self._speed_msg.target:
    #             self._speed_msg.limit = limit
    #             self._speed_msg.target = target
    #             self._speed_msg_publisher.publish(self._speed_msg)
    #     except IndexError:
    #         pass

    def _on_global_path(self):
        p = (self._current_pose.position.x, self._current_pose.position.y)
        lanelets = find_closest_lanelet(self._network, p)
        for lanelet in lanelets:
            if lanelet in self._global_path_lanelets:
                return True
        return False

    def _send_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new global route")
            self._reroute_publisher.publish(Empty())

    def _send_random_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new random global route")
            self._reroute_random_publisher.publish(Bool(self.rules_enabled))

    def _end_of_route_handling(self):
        if self._current_speed < 5:
            self._send_random_global_path_request()  # todo remove in production

    def start(self):
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            if self._planner_at_end_of_route():
                self._end_of_route_handling()
            elif not self._on_global_path():
                self._create_paf_local_path_msg(send_empty=True)
                self._send_global_path_request()
            else:
                rospy.loginfo_throttle(30, "[local planner] car is on route, no need to reroute")
                self._create_paf_local_path_msg()
            rate.sleep()


if __name__ == "__main__":
    LocalPlanner().start()
