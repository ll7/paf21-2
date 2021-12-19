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
    xy_from_distance_and_angle, k_closest_indices_of_point_in_list, find_closest_lanelet,
)
from classes.Spline import calc_spline_course, calc_spline_course_from_point_list
from classes.MapManager import MapManager
from std_msgs.msg import Bool, Empty
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

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
        m_i, l_i, p_i = self._get_matrix_and_lane_indices(msg.target)
        self._target_indices = m_i, l_i, p_i + 1
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

    def _set_current_path(self):
        if len(self._global_path) == 0:
            self._local_path = []
            self._distances_delta = 0.01
            return self._local_path, self._target_speed

        target_distance = max(self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * self._current_speed)
        distance_planned = 0

        sparse_local_path = []
        sparse_local_path_speeds = []
        sparse_traffic_signals = []

        _, current_lane, _ = self._current_indices
        _, target_index, _ = self._target_indices

        lane_change_secs = 3  # 3s for a lane change (distance depends on speed)

        row_generator = self._yield_next_matrix_rows()

        pts_limits, _, _, _ = next(row_generator)
        pt, current_speed = pts_limits[current_lane]
        current_speed = current_speed if current_speed > 0 else self._current_speed_limit
        sparse_local_path.append(pt)
        sparse_local_path_speeds.append(current_speed)

        while distance_planned < target_distance:

            matrix_temp = []
            target_lanes = [0]
            shift_idx = 0
            for i, item in enumerate(row_generator):
                pts_limits, trg_idx, shift_idx, distance_planned = item
                matrix_temp.append(pts_limits)
                if trg_idx != 0 or shift_idx != 0:
                    cur_len = len(matrix_temp[-1])
                    target_lanes = list(range(-shift_idx, -shift_idx + cur_len))
                    target_index = trg_idx
                    break
                if distance_planned > target_distance:
                    break

            next_lane_change_index = 0
            for i, pts_limits in enumerate(matrix_temp):  # i is the next (unmapped row of possible points)
                while next_lane_change_index >= i:
                    continue
                indices_count_for_lane_change = max(1, current_speed * lane_change_secs)

                # todo multiple target lanes ??
                number_of_lanes_off = int(np.abs(target_index - current_lane))  # n lanes to change to target lane
                j = i - 1 + indices_count_for_lane_change * number_of_lanes_off
                j_max = i - 1 + indices_count_for_lane_change * len(pts_limits)

                target_is_left = target_index < current_lane
                l_change, r_change = False, False
                l_limit, r_limit = 0, len(pts_limits) - 1
                l_change_allowed, r_change_allowed = current_lane > l_limit, current_lane < r_limit
                if j + 1 >= len(matrix_temp):
                    # need to lane change here (the latest possibility)
                    l_change = target_is_left
                    r_change = not target_is_left
                if j_max + 1 >= len(matrix_temp):
                    # no lane changes in "wrong" direction allowed anymore
                    l_change_allowed = target_is_left
                    r_change_allowed = not target_is_left

                probabilities = self._test_lane_change(
                    can_go_left=l_change or (l_change_allowed and not r_change),
                    can_go_right=r_change or (r_change_allowed and not l_change),
                    can_go_straight=not (l_change or r_change)
                )
                choice = np.random.choice(["left", "straight", "right"], 1, p=probabilities)

                if choice == "left" or choice == "right":
                    next_lane_change_index = min(j, len(matrix_temp) - 1)
                    if choice == "left":
                        current_lane -= 1
                    else:
                        current_lane += 1
                    new_pt, current_speed = matrix_temp[next_lane_change_index]
                else:  # straight
                    new_pt, current_speed = pts_limits[current_lane]
                sparse_local_path.append(new_pt)
                sparse_local_path_speeds.append(current_speed)

            # switch to new lanelet group / matrix (or is at target point)
            current_lane = current_lane - target_index + shift_idx

        self._local_path = calc_spline_course_from_point_list(sparse_local_path, ds=.1)
        speeds = self._update_target_speed(self._local_path, sparse_local_path_speeds, traffic_signals)
        return self._local_path, speeds

    @staticmethod
    def _test_lane_change(can_go_left, can_go_straight, can_go_right):
        left = can_go_left * 1
        right = can_go_right * 1
        straight = can_go_straight * 2
        _sum = left + right + straight
        return left / _sum, straight / _sum, right / _sum

    def _yield_next_matrix_rows(self):
        m_idx, _, p_idx = self._current_indices
        m_idx_t, _, p_idx_t = self._target_indices
        m_idx_start, p_idx_start = m_idx, p_idx

        def get_speed_limit_by_index(lanes, prev_speed_limits, prev_target_index, prev_shift):
            prev_speed_limits = [lane[-1] for lane in prev_speed_limits]
            speed_limit_lanes = [lane.speed_limits for lane in lanes]
            result = []
            for k, lane_limits in enumerate(speed_limit_lanes):
                limits_in_lane = []
                speed_sign: PafTrafficSignal
                for speed_sign in lane_limits:
                    if speed_sign.index >= num_pts:
                        break
                    while len(limits_in_lane) - 1 < speed_sign.index:
                        limits_in_lane.append(speed_sign.value)
                while len(limits_in_lane) < num_pts:
                    limits_in_lane.append(limits_in_lane[-1])
                try:
                    prev_idx = k + prev_target_index + prev_shift
                    prev_limit = prev_speed_limits[prev_idx]
                    for j, lim in enumerate(limits_in_lane):
                        if lim > 0:
                            break
                        limits_in_lane[j] = prev_limit
                except IndexError:
                    pass
                result.append(limits_in_lane)
            return result

        distance_m = 0
        total_num_pts = 0

        speed_limits = []
        target_index, shift = 0, 0

        plan_matrix: PafLaneletMatrix
        for plan_matrix in self._global_path[m_idx:]:
            distance_per_index = plan_matrix.distance_per_index
            if m_idx == m_idx_t:
                plan_matrix.lanes = [lane[:p_idx_t + 1] for lane in plan_matrix.lanes]  # pointer(!)
            elif m_idx == m_idx_start:
                plan_matrix.lanes = [lane[p_idx_start:] for lane in plan_matrix.lanes]
            # lane_count = len(plan_matrix.lanes)
            num_pts = len(plan_matrix.lanes[0].points)
            speed_limits = get_speed_limit_by_index(plan_matrix.lanes, speed_limits, target_index, shift)
            target_index = plan_matrix.target_index
            shift = plan_matrix.shift
            # target_index = l_idx_t if m_idx == m_idx_t else plan_matrix.target_index
            # future_lane_count = 1 if m_idx == m_idx_t else len(self._global_path[m_idx + 1].lanes)
            points = [lane.points for lane in plan_matrix.lanes]
            zipped = list(zip(zip(*points), zip(*speed_limits)))
            signs_per_lane = [lane.signals for lane in plan_matrix.lanes]
            for p_idx, (points_row, limits_row) in enumerate(zipped):
                signs = [s for s in signs_per_lane]
                pts_limits = list(zip(points_row, limits_row))
                total_num_pts += 1
                distance_m += distance_per_index
                if p_idx == len(zipped) - 1:
                    t_idx = target_index
                    shift_idx = shift
                else:
                    t_idx = 0
                    shift_idx = 0
                yield pts_limits, signs, t_idx, shift_idx, distance_m - distance_per_index
                # [((x,y), (speed_limit)), ...], signs_per_row, left_anchor_to_next_row, shift_idx right for next row, distance_planned
            m_idx += 1

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
        elif len(self._global_path) > 0 and (
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

    def _update_target_speed(self, local_path, local_speeds, traffic_signals):
        if len(self._global_path) == 0:
            return 0
        calc = SpeedCalculator(local_path)
        # self._signal_debug_print(self._traffic_signals)
        speed_limit = calc.expand_sparse_speeds(local_speeds, len(local_path))
        speed = calc.get_curve_speed(local_path)
        speed = np.clip(speed, 0, speed_limit)
        if self.rules_enabled:
            speed = calc.add_stop_events(speed, traffic_signals, target_speed=0, buffer_m=6, shift_m=-3)
            speed = calc.add_roll_events(speed, traffic_signals, target_speed=0, buffer_m=6, shift_m=-3)

        if self._current_speed < 1 and self._allowed_from_stop():
            rospy.sleep(2)
            rospy.logwarn_throttle(5, "[local planner] continuing from stop")
            speed = calc.remove_stop_event(speed, buffer_m=10)

        speed = calc.add_linear_deceleration(speed)
        # pts = PafTopDownViewPointSet()
        # pts.label = "low_speed"
        # pts.points = [self._global_path[i + start_idx] for i, s in enumerate(speed) if s < 0.1]
        # pts.color = (255, 0, 255)
        # self._sign_publisher.publish(pts)

        # self._speed_debug_print(speed)
        self._target_speed = speed
        # self._publish_speed_msg()
        return speed

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
        current_lanelet = find_closest_lanelet(self._network, position)
        if hasattr(position, "x"):
            ref = (position.x, position.y)
        else:
            ref = position
        for i, matrix in enumerate(self._global_path):
            lane: PafLanelet
            for j, lane in enumerate(matrix.lanes):
                if current_lanelet == lane.lanelet_id:
                    idx, d = k_closest_indices_of_point_in_list(2, lane.points, ref)
                    current_matrix_idx = i
                    current_lane_idx = j
                    current_lane_pt_idx = min(idx)
                    return current_matrix_idx, current_lane_idx, current_lane_pt_idx
        else:
            return -1, -1, -1

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
        lanelet = find_closest_lanelet(self._network, p)
        return lanelet in self._global_path_lanelets

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
