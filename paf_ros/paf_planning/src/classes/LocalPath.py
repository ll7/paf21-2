from typing import Tuple, List

import rospy
from paf_messages.msg import PafLocalPath, Point2D, PafRouteSection, PafTopDownViewPointSet, PafTrafficSignal
from .GlobalPath import GlobalPath
from .HelperFunctions import xy_to_pts, expand_sparse_list, dist_pts, closest_index_of_point_list

import numpy as np

from .SpeedCalculator import SpeedCalculator
from .Spline import calc_spline_course_from_point_list


class LocalPath:
    REPLANNING_THRESHOLD_DISTANCE_M = 15
    STEP_SIZE = 0.125
    TRANSMIT_FRONT_MIN_M = 100
    TRANSMIT_FRONT_SEC = 10

    def __init__(self, global_path: GlobalPath, rules_enabled: bool = None):
        self._local_path_start_section = 0
        self._sparse_local_path_speeds = []
        self._sparse_traffic_signals = []
        self._sparse_local_path = []
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

    def _calculate_intermediate_pts(self, s: PafRouteSection, from_lane: int, to_lane: int, fraction: float):

        p1 = np.array([s.points[to_lane].x, s.points[to_lane].y])
        try:
            p0 = np.array([s.points[from_lane].x, s.points[from_lane].y])
        except IndexError:
            return None, None, None

        # calculate intermediate point

        intermediate_p = np.sum([p1 * fraction, p0 * (1 - fraction)], axis=0)
        intermediate_p = Point2D(intermediate_p[0], intermediate_p[1])

        # choose higher speed of the two lanes
        speed = max([s.speed_limits[to_lane], s.speed_limits[from_lane]])

        # merge traffic signals of the two lanes, remove duplicates
        sig1 = self.global_path.get_signals(s, to_lane)
        sig2 = self.global_path.get_signals(s, from_lane)
        signals = []
        sig: PafTrafficSignal
        added_sig: PafTrafficSignal
        for sig in sig1 + sig2:
            for added_sig in signals:
                if added_sig.value == sig.value and added_sig.type == sig.type:
                    break
            else:
                signals.append(sig)

        return intermediate_p, speed, signals

    def calculate_new_local_path(
        self, from_position: Point2D, current_speed: float = 0, ignore_signs_distance: float = 0
    ):
        local_path = PafLocalPath()
        # float32[]             target_speed
        # Point2D[]             points
        # PafTrafficSignalList[]    signals

        section_target, lane_target = self.global_path.get_section_and_lane_indices(self.global_path.route.target)
        sparse_local_path, sparse_local_path_speeds, sparse_traffic_signals = None, None, None
        prev_idx, _ = closest_index_of_point_list(self._sparse_local_path, from_position)

        lane_change_secs = 5
        min_lane_change_meters = 30
        buffer = 5

        target_distance = max([self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * current_speed])
        distance_planned = 0

        lane_change_pts = []  # debug points
        section_from = -1
        current_lane = 0

        if prev_idx > 0:
            prev_idx = max(0, prev_idx - 5)
            end_index = min(prev_idx + 5, len(self.message.points) - 1)
            sparse_local_path = self._sparse_local_path[prev_idx:end_index]
            sparse_local_path_speeds = self._sparse_local_path_speeds[prev_idx:end_index]
            sparse_traffic_signals = self._sparse_traffic_signals[prev_idx:end_index]
            section_from, current_lane = self.global_path.get_section_and_lane_indices(self.message.points[end_index])
            if dist_pts(self.global_path.route.sections[section_from].points[0], from_position) > 5:
                prev_idx = -1  # sanity check
        if prev_idx <= 0:
            section_from, current_lane = self.global_path.get_section_and_lane_indices(from_position)
            point, current_speed, signals = self.global_path.get_local_path_values(section_from, current_lane)
            sparse_local_path = [point]
            sparse_local_path_speeds = [current_speed]
            sparse_traffic_signals = [signals]

        if section_from < 0 or section_target < 0:
            rospy.logerr("unable to calculate local path")
            self.message = local_path
            return local_path

        self._local_path_start_section = section_from
        lane_change_until_distance = None
        lane_change_start_distance = None
        last_lane = current_lane

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
            # print(f"section {i}, available lanes: {list(range(len(s.points)))}, "
            #       f"target={list(s.target_lanes)}, dist={s.target_lanes_distance}")

            # test if current lane exists on this section
            if not 0 <= current_lane < len(s.points):
                rospy.logerr(f"{current_lane} does not exist in available lanes: {list(range(len(s.points)))}")
                if current_lane < 0:
                    current_lane = 0
                else:
                    current_lane = len(s.points) - 1

            if lane_change_until_distance is not None and s.target_lanes_distance > 0:
                fraction_completed = (distance_planned - lane_change_start_distance) / (
                    lane_change_until_distance - lane_change_start_distance
                )
                rospy.logwarn(fraction_completed)

                if 0 <= fraction_completed < 1 and i <= len(self.global_path) - 1:
                    distance_planned += s.distance_from_last_section

                    point, speed, signs = self._calculate_intermediate_pts(
                        s, last_lane, current_lane, fraction_completed
                    )
                    if point is not None:
                        sparse_local_path.append(point)
                        if speed < 0:
                            speed = 50 / 3.6
                        sparse_local_path_speeds.append(speed)
                        sparse_traffic_signals.append(signs)
                    continue

            lane_change_until_distance = None
            distance_planned += s.distance_from_last_section
            sparse_local_path.append(s.points[current_lane])

            speed = s.speed_limits[current_lane]
            if s.speed_limits[current_lane] < 0:
                rospy.logerr(f"[local planner] Speed limit < 0kmh, correcting.. {s.speed_limits[current_lane]}")
                speed = 50 / 3.6
            sparse_local_path_speeds.append(speed)
            if distance_planned < ignore_signs_distance:
                sparse_traffic_signals.append([])
            else:
                sparse_traffic_signals.append(self.global_path.get_signals(s, current_lane))

            if s.target_lanes_distance == 0:
                if not 0 <= current_lane < len(s.points):
                    rospy.logerr(f"{current_lane} does not exist in available lanes: {list(range(len(s.points)))}")
                    if current_lane < 0:
                        current_lane = 0
                    else:
                        current_lane = len(s.points) - 1
                current_lane += -s.target_lanes[0] + s.target_lanes_left_shift
                continue

            if distance_planned > target_distance:
                break

            # target is right == number of lanes off > 0
            # target is left == number of lanes off < 0
            # on target lanes: number of lanes off == 0
            if current_lane > s.target_lanes[-1]:
                number_of_lanes_off = s.target_lanes[-1] - current_lane
            elif current_lane < s.target_lanes[0]:
                number_of_lanes_off = current_lane - s.target_lanes[0]
            else:
                number_of_lanes_off = 0

            distance_for_one_lane_change = max([current_speed * lane_change_secs, min_lane_change_meters]) - buffer
            distance_to_next_lane_change = s.target_lanes_distance
            distance_to_off_lanes_change = np.abs(number_of_lanes_off) * distance_for_one_lane_change
            distance_to_off_plus_1_lanes_change = (np.abs(number_of_lanes_off) + 2) * distance_for_one_lane_change

            l_limit, r_limit = 0, len(s.points) - 1
            l_change_allowed, r_change_allowed = current_lane > l_limit, current_lane < r_limit
            l_change, r_change = False, False

            # print(l_limit, r_limit, current_lane)
            if 0 < distance_to_next_lane_change <= distance_to_off_lanes_change:
                # need to lane change here (the latest possibility)
                l_change = l_change_allowed and number_of_lanes_off > 0  # must go left is isR
                r_change = r_change_allowed and number_of_lanes_off < 0  # must go right if isL
            elif distance_to_next_lane_change <= distance_to_off_plus_1_lanes_change:
                # no lane changes in "wrong" direction allowed anymore
                l_change_allowed = l_change_allowed and number_of_lanes_off > 0  # can go left if isR
                r_change_allowed = r_change_allowed and number_of_lanes_off < 0  # can go right if isL
            if lane_change_until_distance is not None:
                lane_change_until_distance = None
                l_change_allowed = l_change = False
                r_change_allowed = r_change = False

            probabilities = self._choose_lane(
                can_go_left=l_change or l_change_allowed,
                can_go_right=r_change or r_change_allowed,
                can_go_straight=not (l_change or r_change),
            )
            choice = np.random.choice(["left", "straight", "right"], p=probabilities)

            if choice == "left" or choice == "right":
                last_lane = current_lane
                current_lane += -1 if choice == "left" else 1
                lane_change_start_distance = distance_planned
                lane_change_until_distance = min(
                    [s.target_lanes_distance, distance_planned + distance_for_one_lane_change]
                )

                # print(
                #     f"lanes={list(range(len(s.points)))}, target_lanes={list(s.target_lanes)}, "
                #     f"lane change {last_lane}->{current_lane} ({choice}), "
                #     f"must:{l_change}/{r_change}, opt: {l_change_allowed}/{r_change_allowed}, "
                #     f"dist={round(lane_change_start_distance)}->"
                #     f"{round(lane_change_until_distance)}, {s.target_lanes_distance}"
                # )
                if i + 1 < len(self.global_path):
                    lane_change_pts.append(self.global_path.route.sections[i + 1].points[current_lane])

        points = sparse_local_path
        traffic_signals = sparse_traffic_signals
        speed_limit = sparse_local_path_speeds
        if len(sparse_local_path) > 1:
            _points = calc_spline_course_from_point_list(sparse_local_path, ds=self.STEP_SIZE)
            if not np.isnan(_points).any():
                points = xy_to_pts(_points)
                traffic_signals = expand_sparse_list(sparse_traffic_signals, len(points), fill_value=[])
                speed_limit = expand_sparse_list(sparse_local_path_speeds, len(points))

        self._sparse_local_path = sparse_local_path
        self._sparse_traffic_signals = sparse_traffic_signals
        self._sparse_local_path_speeds = sparse_local_path_speeds

        target_speed = self._update_target_speed(points, speed_limit, traffic_signals)

        local_path.points = points
        local_path.target_speed = target_speed
        try:
            local_path.header.stamp = rospy.Time()
        except Exception:
            pass

        self._draw_path_pts(sparse_local_path[::2])
        self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))
        self.message = local_path
        self.traffic_signals = sparse_traffic_signals

        return local_path

    def _update_target_speed(self, local_path, speed_limit, traffic_signals):
        speed = self.speed_calc.get_curve_speed(local_path)

        if self.rules_enabled:
            speed = np.clip(speed, 0, speed_limit)
            # speed = self.speed_calc.add_stop_events(speed, traffic_signals, target_speed=0, buffer_m=6, shift_m=-3)
            # speed = self.speed_calc.add_roll_events(speed, traffic_signals, target_speed=0, buffer_m=6, shift_m=-3)

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
        # print(f"can go left: {can_go_left}, can_go_straight: {can_go_straight}, can_go_right: {can_go_right}, ")
        left = can_go_left * 1
        right = can_go_right * 1
        straight = can_go_straight * 5
        _sum = left + right + straight
        return left / _sum, straight / _sum, right / _sum
