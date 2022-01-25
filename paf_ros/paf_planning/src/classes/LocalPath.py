from time import perf_counter
from typing import List

import rospy
from paf_messages.msg import PafLocalPath, Point2D, PafRouteSection, PafTopDownViewPointSet, PafTrafficSignal
from .GlobalPath import GlobalPath
from .HelperFunctions import dist_pts, closest_index_of_point_list, expand_sparse_list

import numpy as np

from .SpeedCalculator import SpeedCalculator
from .Spline import calc_bezier_curve_from_pts


class LocalPath:
    REPLANNING_THRESHOLD_DISTANCE_M = 15
    STEP_SIZE = 5
    TRANSMIT_FRONT_MIN_M = 300
    TRANSMIT_FRONT_SEC = 10
    LANE_CHANGE_SECS = 7
    STRAIGHT_TO_TARGET_DIST = 10
    THRES_RESTART_BEZIER_EVERY_M = 50
    END_OF_ROUTE_SPEED = 0

    def __init__(self, global_path: GlobalPath, rules_enabled: bool = None):
        self._local_path_start_section = 0
        self._sparse_local_path_speeds = []
        self._sparse_traffic_signals = []
        self._sparse_local_path = []
        self._lane_change_indices = []
        self.traffic_signals = []
        self.global_path = global_path
        self.message = PafLocalPath()
        self.rules_enabled = rules_enabled
        self.speed_calc = SpeedCalculator(self.STEP_SIZE)

    def __len__(self):
        return len(self.message.points)

    def current_indices(self, position: Point2D):
        return self.global_path.get_section_and_lane_indices(position)

    def _next_lanechange_index(self, from_index: int = 0):
        max_idx = len(self._sparse_local_path) - 1
        for idx in self._lane_change_indices:
            if idx > max_idx:
                return max_idx
            if idx >= from_index:
                return idx
        return from_index

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

        try:
            p1 = np.array([s.points[to_lane].x, s.points[to_lane].y])
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
        if speed < 0:
            speed = 50 / 3.6
        return intermediate_p, speed, signals

    def _calculate_lane_options(
        self, start_idx, lane_change_distance: float, current_lane, left_lane, right_lane, can_go_straight: bool
    ):
        distance_planned = 0
        l_pts, r_pts, s_pts = [], [], []
        l_speed, r_speed, s_speed = [], [], []
        l_signs, r_signs, s_signs = [], [], []
        end_idx = start_idx
        additional_points = 10
        break_idx = None
        for end_idx, s in enumerate(self.global_path.route.sections[start_idx:]):
            end_idx += start_idx
            distance_planned += s.distance_from_last_section
            fraction_completed = distance_planned / lane_change_distance
            if break_idx is None and distance_planned > lane_change_distance:
                break_idx = end_idx + additional_points
            if break_idx == end_idx:
                break
            if end_idx <= len(self.global_path) - 1:
                fraction_completed = float(np.clip(fraction_completed, 0, 1))
                distance_planned += s.distance_from_last_section
                if 0.2 > fraction_completed > 0.8:
                    continue
                if left_lane is not None:
                    point, speed, signs = self._calculate_intermediate_pts(
                        s, current_lane, left_lane, fraction_completed
                    )
                    if point is not None:
                        l_pts.append(point)
                        l_speed.append(speed)
                        l_signs.append(signs)
                if right_lane is not None:
                    point, speed, signs = self._calculate_intermediate_pts(
                        s, current_lane, right_lane, fraction_completed
                    )
                    if point is not None:
                        r_pts.append(point)
                        r_speed.append(speed)
                        r_signs.append(signs)
                continue

        if can_go_straight and end_idx != start_idx:
            _temp = end_idx + 1
            s_pts = [
                s.points[current_lane]
                for s in self.global_path.route.sections[start_idx:_temp]
                if 0 <= current_lane < len(s.points)
            ]
            s_speed = [
                s.speed_limits[current_lane]
                for s in self.global_path.route.sections[start_idx:_temp]
                if 0 <= current_lane < len(s.points)
            ]
            s_signs = [
                self.global_path.get_signals(s, current_lane)
                for s in self.global_path.route.sections
                if 0 <= current_lane < len(s.points)
            ]

        left = (l_pts, l_speed, l_signs) if len(l_pts) > 0 else None
        straight = (s_pts, s_speed, s_signs) if len(s_pts) > 0 else None
        right = (r_pts, r_speed, r_signs) if len(r_pts) > 0 else None

        return end_idx, distance_planned, left, straight, right

    def publish(self, msg=None, send_empty=False):
        if send_empty:
            msg = PafLocalPath()
            msg.target_speed = []
            msg.points = []
        elif msg is None:
            msg = self.message
        elif len(msg.points) == 0:
            rospy.logwarn("[local planner] skip publishing, cause len=0")
            return
        if not send_empty:
            rospy.logwarn(f"[local planner] publishing {len(msg.points)} points to acting...")
        publisher = rospy.Publisher("/paf/paf_local_planner/path", PafLocalPath, queue_size=1)
        publisher.publish(msg)

    def calculate_new_local_path(
        self,
        from_position: Point2D,
        current_speed: float = 0,
        ignore_signs_distance: float = 0,
        ignore_previous: bool = False,
        min_section: int = 0,
    ):

        t0 = perf_counter()
        local_path = PafLocalPath()
        section_target, target_lane = self.global_path.get_section_and_lane_indices(
            self.global_path.route.target, min_section=min_section
        )
        if section_target < 0:
            section_target = len(self.global_path)
        sparse_local_path, sparse_local_path_speeds, sparse_traffic_signals = [], [], []

        if not ignore_previous:
            prev_idx, _ = closest_index_of_point_list(self._sparse_local_path, from_position)
        else:
            self._sparse_local_path = []
            self._sparse_traffic_signals = []
            self._sparse_local_path_speeds = []
            prev_idx = -1

        lane_change_secs = self.LANE_CHANGE_SECS
        buffer = 5
        num_points_previous_plan = 10

        target_distance = max([self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * current_speed])
        distance_planned = 0

        lane_change_pts = []  # debug points
        section_from = -1
        current_lane = 0

        if prev_idx > 0:
            current_idx = prev_idx
            prev_idx = max(0, prev_idx - num_points_previous_plan)
            end_index = self._next_lanechange_index(current_idx)
            _temp = end_index + 1
            sparse_local_path = self._sparse_local_path[prev_idx:_temp]
            sparse_local_path_speeds = self._sparse_local_path_speeds[prev_idx:_temp]
            sparse_traffic_signals = self._sparse_traffic_signals[prev_idx:_temp]
            section_from, current_lane = self.global_path.get_section_and_lane_indices(
                self._sparse_local_path[end_index]
            )
            offset2 = current_idx + 1
            for p1, p2 in zip(self._sparse_local_path[current_idx:], self._sparse_local_path[offset2:]):
                distance_planned += dist_pts(p1, p2)

        self._lane_change_indices = []

        if prev_idx < 0 or section_from < 0:
            section_from, current_lane = self.global_path.get_section_and_lane_indices(from_position)
            if section_from > 0:
                point, current_speed, signals = self.global_path.get_local_path_values(section_from, current_lane)
                sparse_local_path = [point]
                sparse_local_path_speeds = [current_speed]
                sparse_traffic_signals = [signals]

        if section_from < 0 or section_target < 0:
            rospy.logerr_throttle(
                1,
                f"unable to calculate local path (index=={section_from},{section_target}) "
                f"{self.global_path.route.target}, {len(self.global_path)}",
            )
            self.message = local_path
            return local_path, 0, 0

        self._local_path_start_section = section_from
        end_idx_lane_change = 0
        idx1 = section_from + 1
        s: PafRouteSection
        for i, s in enumerate(self.global_path.route.sections[idx1:]):
            i += idx1
            # test if current lane exists on this section
            current_lane = np.abs(current_lane)
            if not 0 <= current_lane < len(s.points):
                rospy.logerr(f"{current_lane} does not exist in available lanes: {list(range(len(s.points)))}")
                if current_lane < 0:
                    current_lane = 0
                else:
                    current_lane = len(s.points) - 1

            if distance_planned > target_distance:
                # rospy.logerr("break1")
                break
            if i <= end_idx_lane_change:
                # rospy.logerr("continue1")
                continue

            lane_change_until_distance = None

            # end of route handling
            dist_to_target = dist_pts(s.points[current_lane], self.global_path.target)
            if dist_to_target < self.STRAIGHT_TO_TARGET_DIST and len(sparse_local_path) > 0:
                sparse_local_path.append(self.global_path.target)
                sparse_local_path_speeds += [self.END_OF_ROUTE_SPEED]
                for _i in range(1, 10):
                    try:
                        sparse_local_path_speeds[-i] = self.END_OF_ROUTE_SPEED + 5
                    except IndexError:
                        break
                sparse_traffic_signals += [[]]
                distance_planned += dist_to_target
                # rospy.logerr("break2")
                break

            distance_planned += s.distance_from_last_section
            sparse_local_path.append(s.points[current_lane])

            target_speed = s.speed_limits[current_lane]
            if s.speed_limits[current_lane] < 30 / 3.6:
                rospy.logerr_throttle(
                    5, f"[local planner] Speed limit < 30, correcting.. {s.speed_limits[current_lane]}"
                )
                target_speed = 30 / 3.6
            sparse_local_path_speeds.append(target_speed)
            if distance_planned < ignore_signs_distance:
                sparse_traffic_signals.append([])
            else:
                sparse_traffic_signals.append(self.global_path.get_signals(s, current_lane))

            if s.target_lanes_distance == 0:
                current_lane = np.abs(current_lane)
                if not 0 <= current_lane < len(s.points):
                    rospy.logerr(f"{current_lane} does not exist in available lanes: {list(range(len(s.points)))}")
                    if current_lane < 0:
                        current_lane = 0
                    else:
                        current_lane = len(s.points) - 1
                current_lane += -s.target_lanes[0] + s.target_lanes_left_shift
                # rospy.logerr("continue2")
                continue

            # target is right == number of lanes off > 0
            # target is left == number of lanes off < 0
            # on target lanes: number of lanes off == 0
            if current_lane > s.target_lanes[-1]:
                number_of_lanes_off = current_lane - s.target_lanes[-1]
            elif current_lane < s.target_lanes[0]:
                number_of_lanes_off = s.target_lanes[0] - current_lane
            else:
                number_of_lanes_off = 0

            distance_for_one_lane_change = (
                max([target_speed * lane_change_secs, current_speed * lane_change_secs]) - buffer
            )
            distance_to_next_lane_change = s.target_lanes_distance
            distance_to_off_lanes_change = np.abs(number_of_lanes_off) * distance_for_one_lane_change
            distance_to_off_plus_1_lanes_change = (np.abs(number_of_lanes_off) + 2) * distance_for_one_lane_change

            l_limit, r_limit = 0, len(s.points) - 1
            l_change_allowed, r_change_allowed = current_lane > l_limit, current_lane < r_limit
            l_change, r_change = False, False

            if 0 < distance_to_next_lane_change <= distance_to_off_lanes_change:
                # need to lane change here (the latest possibility)
                l_change = l_change_allowed and number_of_lanes_off > 0  # must go left is isR
                r_change = r_change_allowed and number_of_lanes_off < 0  # must go right if isL
            elif distance_to_next_lane_change <= distance_to_off_plus_1_lanes_change:
                # no lane changes in "wrong" direction allowed anymore
                l_change_allowed = l_change_allowed and number_of_lanes_off > 0  # can go left if isR
                r_change_allowed = r_change_allowed and number_of_lanes_off < 0  # can go right if isL
            if lane_change_until_distance is not None:
                l_change_allowed = l_change = False
                r_change_allowed = r_change = False
            lane_change_distance = min(distance_for_one_lane_change, dist_to_target)

            left_lane, right_lane = None, None
            if l_change or l_change_allowed:
                if lane_change_until_distance == s.target_lanes_distance:
                    left_lane = current_lane - number_of_lanes_off
                else:
                    left_lane = current_lane - 1
            if r_change or r_change_allowed:
                if lane_change_until_distance == s.target_lanes_distance:
                    right_lane = current_lane - number_of_lanes_off
                else:
                    right_lane = current_lane + 1
            end_idx_lane_change, distance_changed, left, straight, right = self._calculate_lane_options(
                i, lane_change_distance, current_lane, left_lane, right_lane, not (l_change or r_change)
            )
            try:
                choice = self._choose_lane(left, straight, right)
            except ValueError:
                choice = "straight"
                rospy.logerr("[local planner] unable to determine new target lane")
                rospy.logerr(
                    f"lanes={list(range(len(s.points)))}, target_lanes={list(s.target_lanes)}, "
                    f"lanes {left_lane}|{current_lane}|{right_lane} ({choice}), "
                    f"must:{l_change}/{r_change}, opt: {l_change_allowed}/{r_change_allowed}, "
                    f"dist: {distance_changed}, {lane_change_distance}"
                )
            if choice == "left" or choice == "right":
                pts, speeds, signs = left if choice == "left" else right
                current_lane = left_lane if choice == "left" else right_lane
                for pt, sp, sgn in zip(pts, speeds, signs):
                    sparse_local_path.append(pt)
                    sparse_local_path_speeds.append(sp)
                    sparse_traffic_signals.append(sgn)
                distance_planned += distance_changed
                lane_change_pts += [pts[0], pts[-1]]
                self._lane_change_indices.append(end_idx_lane_change)
            else:
                end_idx_lane_change = 0

        t1 = f"{(perf_counter() - t0):.2f}s"
        self._sparse_local_path = sparse_local_path
        self._sparse_traffic_signals = sparse_traffic_signals
        self._sparse_local_path_speeds = sparse_local_path_speeds

        local_path.points = sparse_local_path
        local_path.target_speed = sparse_local_path_speeds
        if len(sparse_local_path) > 3:
            self.publish(local_path)

        t0 = perf_counter()
        points, target_speed = self._smooth_out_path(sparse_local_path, sparse_local_path_speeds)
        t2 = f"{(perf_counter() - t0):.2f}s"
        t0 = perf_counter()
        target_speed = self._update_target_speed(points, target_speed)
        t3 = f"{(perf_counter() - t0):.2f}s"

        msg = (
            f"[local planner] planned local path with length={distance_planned:.1f}m "
            f"over {len(sparse_local_path)} sections local_points: {len(local_path.points)} "
            f"(sparse={t1}, smooth={t2}, speeds={t3})"
        )

        local_path.points = points
        local_path.target_speed = target_speed

        self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))
        self.message = local_path
        self.traffic_signals = sparse_traffic_signals

        if distance_planned < 50:
            rospy.logwarn(msg)
        else:
            rospy.loginfo(msg)

        return local_path, section_from, section_target

    @staticmethod
    def _smooth_out_path(sparse_pts, sparse_speeds):
        try:
            pts = calc_bezier_curve_from_pts(sparse_pts)
            if len(pts) == 0:
                raise ValueError
        except ValueError:
            rospy.logerr("[local planner] Bezier / Spline curve could not be calculated")
            return sparse_pts, sparse_speeds
        try:
            speeds, _ = expand_sparse_list(sparse_speeds, sparse_pts, pts)
        except ValueError:
            rospy.logerr("[local planner] speed could not be calculated")
            speeds = [SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in pts]
        return pts, speeds

    def _update_target_speed(self, local_path, speed_limit):
        speed = self.speed_calc.get_curve_speed(local_path)
        if self.rules_enabled:  # todo speed limit logic for no rules
            speed_limit = np.clip(speed_limit, 33 / 3.6, 666 / 3.6)
            speed = np.clip(speed, 0, speed_limit)
        speed = self.speed_calc.add_linear_deceleration(speed)
        return speed

    @staticmethod
    def _draw_path_pts(points: List[Point2D], lbl: str = "lp_pts", color=(0, 45, 123)):
        try:
            pts1 = PafTopDownViewPointSet()
            pts1.label = lbl
            pts1.points = points
            pts1.color = color
            rospy.Publisher("/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1).publish(pts1)
        except rospy.exceptions.ROSException:
            pass

    @staticmethod
    def _draw_path_line(points: List[Point2D], lbl: str = "lp_line", color=(0, 45, 123)):
        try:
            pts1 = PafTopDownViewPointSet()
            pts1.label = lbl
            pts1.points = points
            pts1.color = color
            rospy.Publisher("/paf/paf_validation/draw_map_lines", PafTopDownViewPointSet, queue_size=1).publish(pts1)
        except rospy.exceptions.ROSException:
            pass

    @staticmethod
    def _choose_lane(left, straight, right) -> str:
        # print(f"can go left: {can_go_left}, can_go_straight: {can_go_straight}, can_go_right: {can_go_right}, ")

        if left is None and straight is None and right is None:
            raise ValueError

        left_percent = int(left is not None) * 100
        right_percent = int(right is not None) * 100
        straight_percent = int(straight is not None) * 1

        # l_pts, l_speed, l_signs = left
        # s_pts, s_speed, s_signs = straight
        # r_pts, r_speed, r_signs = right

        _sum = left_percent + right_percent + straight_percent
        probabilities = left_percent / _sum, straight_percent / _sum, right_percent / _sum
        choice = np.random.choice(["left", "straight", "right"], p=probabilities)
        # rospy.logwarn(f"lane choice: {choice} [l={left_percent}, s={straight_percent}, r={right_percent}]")
        return choice
