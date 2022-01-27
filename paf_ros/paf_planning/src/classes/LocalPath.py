from time import perf_counter
from typing import List

import rospy
from paf_messages.msg import PafLocalPath, Point2D, PafRouteSection, PafTopDownViewPointSet
from .GlobalPath import GlobalPath
from .HelperFunctions import dist_pts, closest_index_of_point_list, expand_sparse_list, pts_to_xy

import numpy as np

from .SpeedCalculator import SpeedCalculator
from .Spline import (
    calc_bezier_curve,
    bezier_refit_all_with_tangents,
    calc_spline_course_from_point_list,
)


class LocalPath:
    STEP_SIZE = 5
    TRANSMIT_FRONT_MIN_M = 300
    TRANSMIT_FRONT_SEC = 10
    LANE_CHANGE_SECS = 7
    STRAIGHT_TO_TARGET_DIST = 15
    END_OF_ROUTE_SPEED = 5

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

    def _calculate_lane_options(
        self, start_idx, lane_change_distance: float, current_lane, left_lane, right_lane, can_go_straight: bool
    ):
        if start_idx == 0:
            start_idx = 1
            rospy.logerr("[local planner] lane change at index 0 not possible (no tangent)")

        distance_planned = 0
        l_pts, r_pts = [], []
        l_speed, r_speed = [], []
        end_idx = start_idx
        # additional_points = 10

        ds = 0.5

        s_pts = []
        s_speed = []
        s_signs = []

        start = self.global_path.route.sections[start_idx]
        for end_idx, s in enumerate(self.global_path.route.sections[start_idx:-1]):
            end_idx += start_idx
            d2 = distance_planned + s.distance_from_last_section
            if d2 > lane_change_distance or len(start.points) != len(s.points):
                break
            distance_planned = d2
            s_pts += [s.points[current_lane]]
            s_speed += [s.speed_limits[current_lane]]
            s_signs += [s.signals]

        relevant = [x for x in self.global_path.route.sections[start_idx:end_idx]]

        def get_pts(target_lane):
            _pts = []
            j = -1
            for _i, section in enumerate(relevant):
                if _i + 1 < len(relevant) / 2:
                    _pts.append(section.points[current_lane])
                    j = _i + 1
                elif j == _i:
                    continue
                else:
                    _pts.append(section.points[target_lane])
            return pts_to_xy(_pts)

        def get_speed(target_lane):
            sp2 = [section.speed_limits[target_lane] for section in relevant]
            out = []
            for sp in zip(s_speed, sp2):
                selection = [x for x in sp if x > 0]
                if len(selection) == 0:
                    out.append(SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED)
                else:
                    out.append(np.min(selection))
            return out

        delta_i = end_idx - start_idx
        if delta_i < 5:
            return end_idx - 1, 0, None, None, None

        indices_map = None
        if left_lane is not None:
            pts = get_pts(left_lane)
            try:
                l_pts = calc_bezier_curve(pts, ds=ds, convert_to_pts=True)[1:]
            except ValueError:
                l_pts = pts
            sp = get_speed(left_lane)
            l_speed, indices_map = expand_sparse_list(sp, s_pts[: len(sp)], l_pts, indices_new=indices_map)
            # l_signs = expand_sparse_list(get_signs(left_lane), s_pts, l_pts, indices_new=indices_map)
        if right_lane is not None:
            pts = get_pts(right_lane)
            try:
                r_pts = calc_bezier_curve(pts, ds=ds, convert_to_pts=True)[1:]
            except ValueError:
                l_pts = pts
            sp = get_speed(right_lane)
            r_speed, indices_map = expand_sparse_list(sp, s_pts[: len(sp)], r_pts, indices_new=indices_map)
            # r_signs = expand_sparse_list(get_signs(right_lane), s_pts, r_pts, indices_new=indices_map)
        if not can_go_straight:
            s_pts = []
            s_speed = []
            # s_signs = []

        l_signs = [[] for _ in l_pts]
        s_signs = [[] for _ in s_pts]
        r_signs = [[] for _ in r_pts]

        left = (l_pts, l_speed, l_signs) if len(l_pts) > 0 else None
        straight = (s_pts, s_speed, s_signs) if len(s_pts) > 0 else None
        right = (r_pts, r_speed, r_signs) if len(r_pts) > 0 else None

        for i, d in enumerate([left, straight, right]):
            if d is None:
                continue
            pts, speeds, signs = d
            if not (len(pts) == len(speeds) == len(signs)):
                raise RuntimeError(f"wrong sizes {i}: {len(pts)} {len(speeds)} {len(signs)}")

        return end_idx - 1, distance_planned, left, straight, right

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
        if not send_empty and msg is not self.message:
            rospy.logwarn_throttle(1, f"[local planner] publishing {len(msg.points)} points to acting...")
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

        end_of_route = False

        if prev_idx > 0:
            current_idx = prev_idx
            prev_idx = max(0, prev_idx - num_points_previous_plan)
            end_index = self._next_lanechange_index(current_idx) + 1
            sparse_local_path = self._sparse_local_path[prev_idx:end_index]
            sparse_local_path_speeds = self._sparse_local_path_speeds[prev_idx:end_index]
            sparse_traffic_signals = self._sparse_traffic_signals[prev_idx:end_index]
            section_from, current_lane = self.global_path.get_section_and_lane_indices(sparse_local_path[-1])
            offset2 = current_idx + 1
            for p1, p2 in zip(
                self._sparse_local_path[current_idx:end_index], self._sparse_local_path[offset2:end_index]
            ):
                distance_planned += dist_pts(p1, p2)
            #
            # prev_idx = end_index  # set for reference later
            #
            # rospy.logwarn(f"1) GOOOO! {distance_planned}")
            # lane_change_pts = [self._sparse_local_path[idx] for idx in self._lane_change_indices]
            # lane_change_pts = self._sparse_local_path[:end_index]
            # self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))
            # if hasattr(self, "blubb"):
            #     raise RuntimeError()
            # self.blubb = None
            # pts = self.global_path.route.sections[section_from].points
            # sp_pt = self._sparse_local_path[end_index]
            # raise RuntimeError(f"{pts_to_xy(pts)} {sp_pt}")

        self._lane_change_indices = []
        if prev_idx < 0 or section_from < 0:
            section_from, current_lane = self.global_path.get_section_and_lane_indices(from_position)
            if section_from > 0:
                point, current_speed, signals = self.global_path.get_local_path_values(section_from, current_lane)
                sparse_local_path = [point]
                sparse_local_path_speeds = [current_speed]
                sparse_traffic_signals = [signals]
            # prev_idx = 0

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
            # rospy.logwarn(
            #     f"{s.target_lanes_index_distance} ({i}): {current_lane} {s.target_lanes}, {s.target_lanes_distance}, "
            #     f"{list(range(len(s.points)))}"
            # )

            s_prev = None if i == 0 else self.global_path.route.sections[i - 1]
            if s_prev.target_lanes_distance == 0:
                future_lane = current_lane - s_prev.target_lanes[0] + s_prev.target_lanes_left_shift
                lane_change_pts += s.points
                # rospy.logerr(
                #     f"end of segment ({i})! current_lane={current_lane}, target_lanes={list(s_prev.target_lanes)},"
                #     f" shift={s_prev.target_lanes_left_shift} => {future_lane}"
                # )
                current_lane = future_lane

            # test if current lane exists on this section
            if not 0 <= current_lane < len(s.points):
                rospy.logerr(f"{current_lane} does not exist in available lanes: {list(range(len(s.points)))}")
                rospy.logerr(
                    f"{s.target_lanes_index_distance}: {s.target_lanes}, {s.target_lanes_distance}, "
                    f"{list(range(len(s.points)))}"
                )
                try:
                    rospy.logerr(f"{pts_to_xy(s.points)}, {pts_to_xy(self.global_path.route.sections[i - 1].points)}")
                except IndexError:
                    pass
                if current_lane < 0:
                    current_lane = 0
                else:
                    current_lane = len(s.points) - 1

                self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))
                raise RuntimeError()

            if distance_planned > target_distance:
                # rospy.logerr(f"break! distance_planned > target_distance: {distance_planned} > {target_distance}")
                break
            if i <= end_idx_lane_change:
                # rospy.logerr("continue1")
                continue

            lane_change_until_distance = None

            # end of route handling
            dist_to_target = dist_pts(s.points[current_lane], self.global_path.target)
            if dist_to_target < self.STRAIGHT_TO_TARGET_DIST and len(sparse_local_path) > 0:
                pth = sparse_local_path[-10:] + [self.global_path.target]
                pth = calc_bezier_curve(pts_to_xy(pth), ds=1, convert_to_pts=True)
                sp = [sparse_local_path_speeds[-1] for _ in pth]

                sparse_local_path = sparse_local_path[:-5] + pth
                sparse_local_path_speeds = sparse_local_path_speeds[:-5] + sp
                sparse_traffic_signals = sparse_traffic_signals[:-5] + [[] for _ in pth]
                distance_planned += dist_to_target
                end_of_route = True
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

            # target is right == number of lanes off > 0
            # target is left == number of lanes off < 0
            # on target lanes: number of lanes off == 0
            if current_lane > s.target_lanes[-1]:
                number_of_lanes_off = current_lane - s.target_lanes[-1]  # eg. 3 - 0 = 3 (go right)
            elif current_lane < s.target_lanes[0]:
                number_of_lanes_off = current_lane - s.target_lanes[0]  # eg. 0 - 3 = -3 (go left)
            else:
                number_of_lanes_off = 0
            # rospy.loginfo(
            #     f"{s.target_lanes_index_distance}: {current_lane}, "
            #     f"{list(s.target_lanes)}, {number_of_lanes_off} {current_lane in s.target_lanes}")

            # assert (current_lane in s.target_lanes and number_of_lanes_off == 0) or (
            #         current_lane not in s.target_lanes and number_of_lanes_off != 0)

            distance_for_one_lane_change = (
                max([target_speed * lane_change_secs, current_speed * lane_change_secs]) - buffer
            )

            if lane_change_until_distance is not None:
                l_change_allowed = l_change = False
                r_change_allowed = r_change = False
            else:
                distance_to_off_lanes_change = np.abs(number_of_lanes_off) * distance_for_one_lane_change
                distance_to_off_plus_1_lanes_change = (np.abs(number_of_lanes_off) + 2) * distance_for_one_lane_change

                l_limit, r_limit = 0, len(s.points) - 1
                l_change_allowed, r_change_allowed = current_lane > l_limit, current_lane < r_limit
                l_change, r_change = False, False

                # rospy.loginfo(f"A ({number_of_lanes_off}): {s.target_lanes_distance} <= "
                #   f"{distance_to_off_lanes_change} -> {s.target_lanes_distance <= distance_to_off_lanes_change}")
                # rospy.loginfo(f"B ({l_change_allowed}|{r_change_allowed}): {s.target_lanes_distance} <= "
                #               f"{distance_to_off_plus_1_lanes_change} -> "
                #               f"{s.target_lanes_distance <= distance_to_off_plus_1_lanes_change}")

                if s.target_lanes_distance <= distance_to_off_lanes_change:
                    # need to lane change here (the latest possibility)
                    l_change = number_of_lanes_off > 0
                    r_change = number_of_lanes_off < 0
                elif s.target_lanes_distance <= distance_to_off_plus_1_lanes_change:
                    l_change_allowed = l_change_allowed and number_of_lanes_off > 0
                    r_change_allowed = r_change_allowed and number_of_lanes_off < 0

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
            # rospy.logerr(f"{s.target_lanes_index_distance}: lane={current_lane}->{list(s.target_lanes)}, avail:|"
            #              f"{'|'.join([str(x) for x in [left_lane, current_lane, right_lane] if x is not None])}"
            #              f"|, must:{l_change}/{r_change}, opt: {l_change_allowed}/{r_change_allowed}, "
            #              f"result={left is not None}/{straight is not None}/{right is not None}")
            try:
                choice = self._choose_lane(left, straight, right)
            except ValueError:
                choice = "straight"
            if choice == "left" or choice == "right":
                # rospy.logerr(
                #     f"lanes={list(range(len(s.points)))}, target_lanes={list(s.target_lanes)}, "
                #     f"lanes {left_lane}|{current_lane}|{right_lane} ({choice}), "
                #     f"must:{l_change}/{r_change}, opt: {l_change_allowed}/{r_change_allowed}, "
                #     f"dist: {distance_changed}, {lane_change_distance}"
                # )
                pts, speeds, signs = left if choice == "left" else right
                future_lane = left_lane if choice == "left" else right_lane
                # rospy.logerr(f"changing lanes: {current_lane}->{future_lane} ({distance_changed:.1f}m)")
                current_lane = future_lane
                for pt, sp, sgn in zip(pts, speeds, signs):
                    sparse_local_path.append(pt)
                    sparse_local_path_speeds.append(sp)
                    sparse_traffic_signals.append(sgn)
                distance_planned += distance_changed
                self._lane_change_indices.append(len(sparse_local_path) - 1)
            else:
                end_idx_lane_change = 0

        t1 = f"{(perf_counter() - t0):.2f}s"
        self._sparse_local_path = sparse_local_path
        self._sparse_traffic_signals = sparse_traffic_signals
        self._sparse_local_path_speeds = sparse_local_path_speeds

        t0 = perf_counter()
        points, target_speed = self._smooth_out_path(sparse_local_path, sparse_local_path_speeds)
        target_speed = self._update_target_speed(points, target_speed, end_of_route)

        t3 = f"{(perf_counter() - t0):.2f}s"

        local_path.points = points
        local_path.target_speed = target_speed

        msg = (
            f"[local planner] planned local path with length={distance_planned:.1f}m "
            f"over {len(sparse_local_path)} sections local_points: {len(local_path.points)} "
            f"(sparse={t1}, smooth={t3})"
        )

        self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))
        # self._draw_path_pts(points, "lanechnge2", (200, 24, 200))
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
            pts = bezier_refit_all_with_tangents(sparse_pts, ds=0.25, ds2=0.2)
            if len(pts) == 0:
                pts = calc_spline_course_from_point_list(sparse_pts, ds=0.25)
            if len(pts) == 0:
                raise ValueError(f"len: {len(sparse_pts)}->0, {pts_to_xy(sparse_pts)}")
        except ValueError as e:
            rospy.logerr(f"[local planner] Bezier / Spline curve could not be calculated {e}")
            return sparse_pts, sparse_speeds
        try:
            speeds, _ = expand_sparse_list(sparse_speeds, sparse_pts, pts)
        except ValueError:
            rospy.logerr("[local planner] speed could not be calculated")
            speeds = [SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in pts]
        return pts, speeds

    def _update_target_speed(self, local_path, speed_limit, end_of_route=False):
        speed = self.speed_calc.get_curve_speed(local_path)
        if self.rules_enabled:  # todo speed limit logic for no rules
            speed = np.clip(speed, 0, speed_limit)
        if end_of_route and speed[-1] > self.END_OF_ROUTE_SPEED:
            n = int(10 / 0.25)
            speed = list(speed[:-n])
            for _ in range(len(speed[-n:])):
                speed.append(self.END_OF_ROUTE_SPEED)

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

        left_percent = int(left is not None) * 1
        right_percent = int(right is not None) * 1
        straight_percent = int(straight is not None) * 1000

        # l_pts, l_speed, l_signs = left
        # s_pts, s_speed, s_signs = straight
        # r_pts, r_speed, r_signs = right

        _sum = left_percent + right_percent + straight_percent
        probabilities = left_percent / _sum, straight_percent / _sum, right_percent / _sum
        choice = np.random.choice(["left", "straight", "right"], p=probabilities)
        # rospy.logwarn(f"lane choice: {choice} [l={left_percent}, s={straight_percent}, r={right_percent}]")
        return choice
