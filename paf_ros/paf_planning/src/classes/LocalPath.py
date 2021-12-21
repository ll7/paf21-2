from typing import Tuple, List

import rospy
from paf_messages.msg import PafLocalPath, Point2D, PafRouteSection, PafTopDownViewPointSet
from std_msgs.msg import Header
from .GlobalPath import GlobalPath
from .HelperFunctions import xy_to_pts, expand_sparse_list

import numpy as np

from .SpeedCalculator import SpeedCalculator
from .Spline import calc_spline_course_from_point_list


class LocalPath:
    REPLANNING_THRESHOLD_DISTANCE_M = 15
    STEP_SIZE = 0.125

    def __init__(self, global_path: GlobalPath, rules_enabled: bool = None):
        self._local_path_start_section = None
        self.traffic_signals = []
        self.global_path = global_path
        self.message = PafLocalPath()
        self.rules_enabled = rules_enabled
        self.speed_calc = SpeedCalculator(self.STEP_SIZE)

    def __len__(self):
        return len(self.message.points)

    def current_indices(self, position: Point2D):
        return self.global_path.get_section_and_lane_indices(position)

    def next_traffic_signal(self, position: Point2D, within_distance=100, buffer_idx=1):
        distance = 0
        distance_buffer = 0
        sec_idx, l_idx = self.current_indices(position)
        sec_idx -= buffer_idx
        idx_1 = self._local_path_start_section
        delta_sec = sec_idx - idx_1
        found = None

        section: PafRouteSection
        for i, (section, sig) in enumerate(
            zip(self.global_path.route.sections[idx_1:], self.traffic_signals[delta_sec:])
        ):
            if len(sig) != 0 or distance >= within_distance:
                found = sig
                break
            if buffer_idx > 0:
                buffer_idx -= 1
                distance_buffer -= section.distance_from_last_section
            distance += section.distance_from_last_section

        return found, distance if distance > 0 else distance_buffer

    def calculate_new_local_path(self, from_position: Point2D, ignore_signs_distance: float = 0):
        local_path = PafLocalPath()
        # float32[]             target_speed
        # Point2D[]             points
        # PafTrafficSignalList[]    signals

        section_from, current_lane = self.global_path.get_section_and_lane_indices(from_position)
        section_target, lane_target = self.global_path.get_section_and_lane_indices(self.global_path.route.target)

        if section_from < 0 or section_target < 0:
            rospy.logerr("unable to calculate local path")
            self.message = local_path
            return local_path

        self._local_path_start_section = section_from

        lane_change_secs = 1
        min_lane_change_meters = 30
        buffer = 5

        target_distance = 250  # max(self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * self._current_speed)
        distance_planned = 0

        point, current_speed, signals = self.global_path.get_local_path_values(section_from, current_lane)

        sparse_local_path = [point]
        # lane_change_pts = []
        sparse_local_path_speeds = [current_speed]
        sparse_traffic_signals = [signals]

        last_lane_change_target_dist = None

        idx1 = section_from + 1

        s: PafRouteSection
        for i, s in enumerate(self.global_path.route.sections[idx1:]):
            i += idx1
            # Point2D[] points
            # float32[] speed_limits
            # PafTrafficSignal[] signals
            # int32[] target_lanes
            # int64 target_lanes_index_distance
            # int32 target_lanes_left_shift
            # float32 distance_from_last_section
            # float32 target_lanes_distance

            if last_lane_change_target_dist is not None and last_lane_change_target_dist < s.target_lanes_distance:
                distance_planned += s.distance_from_last_section
                continue

            distance_planned += s.distance_from_last_section

            sparse_local_path.append(s.points[current_lane])
            sparse_local_path_speeds.append(s.speed_limits[current_lane])
            if distance_planned < ignore_signs_distance:
                sparse_traffic_signals.append([])
            else:
                sparse_traffic_signals.append(self.global_path.get_signals(s, current_lane))

            # print(
            #     f"{len(sparse_local_path) - 1}: width={len(s.points)}, lane={current_lane}, "
            #     f"speed={int(sparse_local_path_speeds[-1] * 3.6)}kmh, "
            #     f"planned={np.round(distance_planned, 1)}m, target_lane_dist={int(s.target_lanes_distance)}"
            # )

            if s.target_lanes_distance == 0:
                # lane_change_pts.append(s.points[current_lane])
                current_lane += -s.target_lanes[0] + s.target_lanes_left_shift
                continue

            if distance_planned > target_distance:
                break

            if current_lane in s.target_lanes:
                number_of_lanes_off = 0
            elif current_lane > s.target_lanes[-1]:
                number_of_lanes_off = current_lane - s.target_lanes[-1]  # target is left == number of lanes off > 0
            else:
                number_of_lanes_off = current_lane - s.target_lanes[0]  # target is left == number of lanes off < 0

            distance_for_one_lane_change = max([current_speed * lane_change_secs, min_lane_change_meters]) + buffer
            distance_to_next_lane_change = s.target_lanes_distance
            distance_to_off_lanes_change = np.abs(number_of_lanes_off) * distance_for_one_lane_change
            distance_to_off_plus_1_lanes_change = (np.abs(number_of_lanes_off) + 1) * distance_for_one_lane_change

            l_limit, r_limit = 0, len(s.target_lanes) - 1
            l_change_allowed, r_change_allowed = current_lane > l_limit, current_lane < r_limit
            l_change, r_change = False, False

            if number_of_lanes_off != 0:
                if distance_to_next_lane_change <= distance_to_off_lanes_change:
                    # need to lane change here (the latest possibility)
                    l_change = l_change_allowed = number_of_lanes_off > 0
                    r_change = r_change_allowed = not l_change
                elif distance_to_next_lane_change <= distance_to_off_plus_1_lanes_change:
                    # no lane changes in "wrong" direction allowed anymore
                    l_change_allowed = number_of_lanes_off > 0
                    r_change_allowed = not l_change_allowed
            if last_lane_change_target_dist is not None:
                # lane_change_pts.append(s.points[current_lane])
                last_lane_change_target_dist = None
                l_change_allowed = l_change = False
                r_change_allowed = r_change = False

            probabilities = self._choose_lane(
                can_go_left=l_change or l_change_allowed,
                can_go_right=r_change or r_change_allowed,
                can_go_straight=not (l_change or r_change),
            )
            choice = np.random.choice(["left", "straight", "right"], p=probabilities)

            if choice == "left" or choice == "right":
                # print(f"lane change {choice}, {l_change} {l_change_allowed} / {r_change} {r_change_allowed}")
                current_lane += -1 if choice == "left" else 1
                last_lane_change_target_dist = max(0, s.target_lanes_distance - distance_for_one_lane_change)
                # lane_change_pts.append(s.points[current_lane])

        if len(sparse_local_path) > 1:
            points = xy_to_pts(calc_spline_course_from_point_list(sparse_local_path, ds=self.STEP_SIZE))
            traffic_signals = expand_sparse_list(sparse_traffic_signals, len(points), fill_value=[])
            speed_limit = expand_sparse_list(sparse_local_path_speeds, len(points))
        else:
            points = sparse_local_path
            traffic_signals = sparse_traffic_signals
            speed_limit = sparse_local_path_speeds

        target_speed = self._update_target_speed(points, speed_limit, traffic_signals)

        local_path.points = points
        local_path.target_speed = target_speed
        local_path.header = Header()
        try:
            local_path.header.stamp = rospy.Time()
        except Exception:
            pass

        self._draw_path_pts(sparse_local_path)
        # self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))
        self.message = local_path
        self.traffic_signals = sparse_traffic_signals

        return local_path

    def _update_target_speed(self, local_path, speed_limit, traffic_signals):
        speed = self.speed_calc.get_curve_speed(local_path)

        if self.rules_enabled:
            speed = np.clip(speed, 0, speed_limit)
            speed = self.speed_calc.add_stop_events(speed, traffic_signals, target_speed=0, buffer_m=6, shift_m=-3)
            speed = self.speed_calc.add_roll_events(speed, traffic_signals, target_speed=0, buffer_m=6, shift_m=-3)

        speed = self.speed_calc.add_linear_deceleration(speed)
        return speed

    @staticmethod
    def _draw_path_pts(points: List[Point2D], lbl: str = "lp_pts", color=(0, 45, 123)):
        pts1 = PafTopDownViewPointSet()
        pts1.label = lbl
        pts1.points = points
        pts1.color = color
        rospy.Publisher("/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1).publish(pts1)

    @staticmethod
    def _choose_lane(can_go_left: bool, can_go_straight: bool, can_go_right: bool) -> Tuple[float, float, float]:
        left = can_go_left * 1
        right = can_go_right * 1
        straight = can_go_straight * 5
        _sum = left + right + straight
        return left / _sum, straight / _sum, right / _sum
