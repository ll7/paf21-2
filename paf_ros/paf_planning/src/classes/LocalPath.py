from time import perf_counter
from typing import List, Tuple, Optional, Union

import rospy
from paf_messages.msg import PafLocalPath, Point2D, PafRouteSection, PafTopDownViewPointSet, PafTrafficSignal
from .GlobalPath import GlobalPath
from .HelperFunctions import dist, closest_index_of_point_list, expand_sparse_list, pts_to_xy

import numpy as np

from .SpeedCalculator import SpeedCalculator
from .Spline import (
    calc_bezier_curve,
    bezier_refit_all_with_tangents,
    calc_spline_course_from_point_list,
)


class LocalPath:
    STEP_SIZE = 5
    DENSE_POINT_DISTANCE = 0.25
    TRANSMIT_FRONT_MIN_M = 300
    TRANSMIT_FRONT_SEC = 10
    LANE_CHANGE_SECS = 7
    STRAIGHT_TO_TARGET_DIST = 10
    END_OF_ROUTE_SPEED = 5
    SPLINE_IN_CURVE_RADIUS = 15

    def __init__(self, global_path: GlobalPath, rules_enabled: bool = None):
        self._local_path_start_section = 0
        self._sparse_local_path_speeds = []
        self._sparse_traffic_signals = []
        self.sparse_local_path = []
        self._lane_change_indices = []
        self.traffic_signals = []
        self.global_path = global_path
        self.message = PafLocalPath()
        self.alternate_speeds = []
        self.rules_enabled = rules_enabled
        self.speed_calc = SpeedCalculator(self.DENSE_POINT_DISTANCE)

    def current_indices(self, position: Point2D) -> Tuple[int, int]:
        """
        get indices in current global path of a position
        :param position: search point
        :return: section, lane
        """
        return self.global_path.get_section_and_lane_indices(position)

    def _next_lanechange_index(self, from_index: int = 0) -> int:
        """
        Get index of next lane change in sparse_local_path
        :param from_index: search from index
        :return: index
        """
        max_idx = len(self.sparse_local_path) - 1
        for idx in self._lane_change_indices:
            if idx > max_idx:
                return max_idx
            if idx >= from_index:
                return idx
        return from_index

    def next_traffic_signal(
        self, position: Point2D, within_distance: float = 100, buffer_idx: int = 1
    ) -> Tuple[Optional[PafTrafficSignal], float]:
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

    def reset_alternate_speed(self):
        self.alternate_speeds = None

    def set_alternate_speed(self, index_start_dense: int, speed: Optional[Union[np.ndarray, List[float]]]):
        if speed is None or len(speed) == 0 or len(self) == 0:
            rospy.logerr(f"skip alterations speed_unset={speed is None or len(speed) == 0} dense_len={len(self) == 0}")
            return
        if self.alternate_speeds is None:
            self.alternate_speeds = np.ones_like(self.message.target_speed) * SpeedCalculator.MAX_SPEED
        index_end = index_start_dense + len(speed)
        self.alternate_speeds[index_start_dense:index_end] = speed
        if speed[-1] < 1e-3:
            self.alternate_speeds[index_end:] *= 0
        # lane_change_pts = self.message.points[index_start_dense:index_end]
        # self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))

    def set_alternate_speed_next_sign(self, current_sparse_index, current_dense_index, skip_first_hit=False):
        alternate_speed, breaking_m, idx_found_sparse = self.speed_calc.get_next_traffic_signal_deceleration(
            self.traffic_signals, current_sparse_index, skip_first_hit
        )
        idx_dense = self.sparse_to_dense_index(idx_found_sparse, current_dense_index)
        if idx_dense < 0:
            return
        n = int(np.ceil(1 / self.DENSE_POINT_DISTANCE))
        pts = list(reversed(self.message.points[: idx_dense + 1 : n]))
        distance = 0
        index = idx_dense - n
        for p0, p1 in zip(pts, pts[1:]):
            distance += dist(p0, p1)
            index -= n
            if distance >= breaking_m:
                break
        lane_change_pts = [
            self.sparse_local_path[idx_found_sparse],
            self.message.points[idx_dense],
            self.message.points[index],
        ]
        self._draw_path_pts(lane_change_pts, "lanechnge", (200, 24, 0))
        rospy.logerr_throttle(5, (idx_dense, index))
        if index < 0:
            return
        self.set_alternate_speed(index, alternate_speed)

    def sparse_to_dense_index(self, index, start_pos):
        if start_pos < 0:
            return -1
        i, _ = closest_index_of_point_list(self.message.points[start_pos:], self.sparse_local_path[index])
        rospy.logwarn_throttle(3, (index, start_pos, i + start_pos, _))
        if i < 0:
            return i
        return i + start_pos

    def _calculate_lane_options(
        self,
        start_idx,
        lane_change_distance: float,
        current_lane: int,
        left_lane: int = None,
        right_lane: int = None,
        can_go_straight: bool = True,
    ) -> Tuple[
        int,
        float,
        Optional[Tuple[List[Point2D], List[float], List[PafTrafficSignal]]],
        Optional[Tuple[List[Point2D], List[float], List[PafTrafficSignal]]],
        Optional[Tuple[List[Point2D], List[float], List[PafTrafficSignal]]],
    ]:
        """
        Calculates left, straight and right lane points, signals and speeds with given parameters
        :param start_idx: global section index to start the lane change on
        :param lane_change_distance: distance to do the lanechange in
        :param current_lane: current lane index
        :param left_lane: left lane index (can be two lanes over in extreme cases)
        :param right_lane: right lane index (can be two lanes over in extreme cases)
        :param can_go_straight: boolean if straight is allowed (force lane change if false)
        :return: end index, distance for lane change,
                    left (pts, speed, signs), straight (pts, speed, signs), right (pts, speed, signs)
        """
        if start_idx == 0:
            start_idx = 1
            rospy.logerr("[local planner] lane change at index 0 not possible (no tangent)")
        distance_planned = 0
        l_pts, r_pts = [], []
        l_speed, r_speed = [], []
        end_idx = start_idx

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

    def publish(self, send_empty: bool = False):
        """
        Publish a local path message (last one calculated OR the msg specified)
        :param send_empty: send an empty message ("stop now" signal for acting)
        """
        msg = PafLocalPath()
        msg.target_speed = []
        msg.points = []
        if not send_empty:
            if self.alternate_speeds is not None:
                msg.target_speed = list(np.clip(self.message.target_speed, 0, self.alternate_speeds))
            else:
                msg.target_speed = self.message.target_speed
            msg.points = self.message.points
            if len(msg.points) == 0:
                rospy.logwarn("[local planner] skip publishing, cause len=0")
                return
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
    ) -> Tuple[PafLocalPath, int, int]:
        """
        Calculates the local path from from_position, if ignore_previous==False,
        path starts at the end of the next lanechange (smoother driving)
        :param from_position: start position of calculation
        :param current_speed: current speed of car
        :param ignore_signs_distance: ignore signs within distance of car
        :param ignore_previous: start path at from_position, not after last lane change is completed
        :param min_section: starting section
        :return: path tuple (msg, section from, section to)
        """
        t0 = perf_counter()
        local_path = PafLocalPath()
        section_target, target_lane = self.global_path.get_section_and_lane_indices(
            self.global_path.route.target, min_section=min_section
        )
        if section_target < 0:
            section_target = len(self.global_path)
        sparse_local_path, sparse_local_path_speeds, sparse_traffic_signals = [], [], []

        if not ignore_previous:
            prev_idx, _ = closest_index_of_point_list(self.sparse_local_path, from_position)
        else:
            self.sparse_local_path = []
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
            sparse_local_path = self.sparse_local_path[prev_idx:end_index]
            sparse_local_path_speeds = self._sparse_local_path_speeds[prev_idx:end_index]
            sparse_traffic_signals = self._sparse_traffic_signals[prev_idx:end_index]
            section_from, current_lane = self.global_path.get_section_and_lane_indices(sparse_local_path[-1])
            offset2 = current_idx + 1
            for p1, p2 in zip(self.sparse_local_path[current_idx:end_index], self.sparse_local_path[offset2:end_index]):
                distance_planned += dist(p1, p2)

        self._lane_change_indices = []
        if prev_idx < 0 or section_from < 0:
            section_from, current_lane = self.global_path.get_section_and_lane_indices(from_position)
            if section_from > 0:
                point, current_speed, signals = self.global_path.get_local_path_values(section_from, current_lane)
                if current_speed < 0:
                    rospy.logerr(f"section {section_from}, lane {current_lane} is undefined")
                else:
                    sparse_local_path = [point]
                    sparse_local_path_speeds = [current_speed]
                    sparse_traffic_signals = [signals]
                    section_from += 2
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
            if s_prev is not None and s_prev.target_lanes_distance == 0:
                future_lane = current_lane - s_prev.target_lanes[0] + s_prev.target_lanes_left_shift
                # lane_change_pts += s.points
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
                # from paf_ros.paf_planning.src.classes.MapManager import MapManager
                # MapManager.visualize_pts_list(sparse_local_path)
                raise RuntimeError()

            if distance_planned > target_distance:
                # rospy.logerr(f"break! distance_planned > target_distance: {distance_planned} > {target_distance}")
                break
            if i <= end_idx_lane_change:
                # rospy.logerr("continue1")
                continue

            # end of route handling
            dist_to_target = dist(s.points[current_lane], self.global_path.target)
            if dist_to_target < self.STRAIGHT_TO_TARGET_DIST and len(sparse_local_path) > 0:
                n = 5
                pth = sparse_local_path[-n:] + [self.global_path.target]
                pth = calc_bezier_curve(pts_to_xy(pth), ds=1, convert_to_pts=True)
                sp = [sparse_local_path_speeds[-1] for _ in pth]

                sparse_local_path = sparse_local_path[:-n] + pth
                sparse_local_path_speeds = sparse_local_path_speeds[:-n] + sp
                sparse_traffic_signals = sparse_traffic_signals[:-n] + [[] for _ in pth]
                distance_planned += dist_to_target
                end_of_route = True
                break

            distance_planned += s.distance_from_last_section
            sparse_local_path.append(s.points[current_lane])

            target_speed = s.speed_limits[current_lane]
            if s.speed_limits[current_lane] < 25 / 3.6:
                rospy.logerr_throttle(
                    5, f"[local planner] Speed limit < 25, correcting.. {s.speed_limits[current_lane]}"
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
                l_change_allowed = l_change_allowed and number_of_lanes_off > 0 or (current_lane - 1 in s.target_lanes)
                r_change_allowed = r_change_allowed and number_of_lanes_off < 0 or (current_lane + 1 in s.target_lanes)

            if l_change or r_change:
                l_change_allowed = r_change_allowed = False
            lane_change_distance = min(distance_for_one_lane_change, dist_to_target)

            left_lane, right_lane = None, None

            if l_change_allowed and not l_change:
                left_lane = current_lane - 1
            if r_change_allowed and not r_change:
                right_lane = current_lane + 1

            if s.target_lanes_distance <= distance_to_off_lanes_change:
                change_num_lanes = abs(number_of_lanes_off)
                if l_change:
                    left_lane = current_lane - change_num_lanes
                elif r_change:
                    right_lane = current_lane + change_num_lanes
            elif l_change:
                left_lane = current_lane - 1
            elif r_change:
                right_lane = current_lane + 1

            end_idx_lane_change, distance_changed, left, straight, right = self._calculate_lane_options(
                i, lane_change_distance, current_lane, left_lane, right_lane, not (l_change or r_change)
            )
            # avail = [str(x) for x in [left_lane, current_lane, right_lane] if x is not None]
            # cur = [int(x.x) for x in s.points]
            # print(
            #     f"{s.target_lanes_index_distance} ({cur}): "
            #     f"lane={current_lane}/{len(s.points)}->{list(s.target_lanes)}, avail:|"
            #     f"{'|'.join(avail)}|, must:{l_change}/{r_change}, opt: {l_change_allowed}/{r_change_allowed}, "
            #     f"result={left is not None}/{straight is not None}/{right is not None}, {l_limit, r_limit}"
            # )
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

        self.sparse_local_path = sparse_local_path
        self._sparse_traffic_signals = sparse_traffic_signals
        self._sparse_local_path_speeds = sparse_local_path_speeds
        self.reset_alternate_speed()

        t0 = perf_counter()
        points, target_speed = self._smooth_out_path(sparse_local_path, sparse_local_path_speeds)
        target_speed = self._update_target_speed(points, target_speed, end_of_route)

        t3 = f"{(perf_counter() - t0):.2f}s"

        local_path.points = points
        local_path.target_speed = target_speed

        msg = (
            f"[local planner] planned local path from ({from_position.x:.1f}, {from_position.y:.1f}) "
            f"with length={distance_planned:.1f}m "
            f"over {len(sparse_local_path)} sections local_points: {len(local_path.points)} "
            f"(sparse={t1}, smooth={t3})"
        )

        # lane_change_pts += sparse_local_path
        # for s in self.global_path.route.sections:
        #     lane_change_pts += s.points
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
    def _smooth_out_path(sparse_pts: List[Point2D], sparse_speeds: List[float]) -> Tuple[List[Point2D], List[float]]:
        """
        Expand sparse path to dense path. Curves with radius > SPLINE_IN_CURVE_RADIUS will use spline,
        others will use Bezier calculation for smoothness
        :param sparse_pts: point list input
        :param sparse_speeds: speed list input (will be expanded as well)
        :return: dense lists (pts, speeds)
        """
        min_radius = LocalPath.SPLINE_IN_CURVE_RADIUS
        n1, n2 = 1, 4  # indices to add before and after high curve radius
        curve_radius_list = SpeedCalculator.get_curve_radius_list(sparse_pts, equal_dist=True)
        change_idx = []
        first_smaller = None
        for i, r in enumerate(curve_radius_list):
            if first_smaller is not None:
                if r >= min_radius:
                    change_idx.append((first_smaller, i + n2))
                    first_smaller = None
                else:
                    continue
            else:
                if r >= min_radius:
                    continue
                else:
                    first_smaller = i - n1
        if first_smaller is not None:
            change_idx.append((first_smaller, len(curve_radius_list)))

        pts = []
        try:
            ds = LocalPath.DENSE_POINT_DISTANCE
            prev = 0
            for i, j in change_idx:
                if prev < i:
                    pts += bezier_refit_all_with_tangents(sparse_pts[prev : i + 1], ds, ds, convert_to_pts=True)[1:-1]
                prev = j
                spline = calc_spline_course_from_point_list(pts_to_xy(sparse_pts[i:j]), ds, to_pts=True)
                pts += spline
            pts += bezier_refit_all_with_tangents(sparse_pts[prev:], ds, ds, convert_to_pts=True)[1:]
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

    def _update_target_speed(
        self, local_path: List[Point2D], speed_limit: List[float], end_of_route: bool = False
    ) -> List[float]:
        """
        Update target speeds with speed limit list and end of route slowdown
        :param local_path: list of points
        :param speed_limit: list of speed limits
        :param end_of_route: bool if end of point list is end of global path
        :return: list of speeds
        """
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
    def _draw_path_pts(
        points: List[Point2D],
        lbl: str = "lp_pts",
        color: Tuple[int, int, int] = (0, 45, 123),
        topic: str = "/paf/paf_validation/draw_map_points",
    ):
        """
        Draw points on the topdown view
        :param points: pts to draw
        :param lbl: label in tdv
        :param color: color of points
        """
        try:
            pts1 = PafTopDownViewPointSet()
            pts1.label = lbl
            pts1.points = points
            pts1.color = color
            rospy.Publisher(topic, PafTopDownViewPointSet, queue_size=1).publish(pts1)
        except rospy.exceptions.ROSException:
            pass

    @staticmethod
    def _draw_path_line(points: List[Point2D], lbl: str = "lp_line", color=(0, 45, 123)):
        """
        Draw line on the topdown view
        :param points: pts to draw
        :param lbl: label in tdv
        :param color: color of points
        """
        LocalPath._draw_path_pts(points, lbl, color, "/paf/paf_validation/draw_map_lines")

    @staticmethod
    def _choose_lane(
        left: Optional[Tuple[List[Point2D], List[float], List[PafTrafficSignal]]],
        straight: Optional[Tuple[List[Point2D], List[float], List[PafTrafficSignal]]],
        right: Optional[Tuple[List[Point2D], List[float], List[PafTrafficSignal]]],
    ) -> str:
        """
        Chose lane from lane change options
        :param left: left option (or None)
        :param straight: straight option (or None)
        :param right: right option (or None)
        :return: keyword "left", "straight" or "right"
        """
        if left is None and straight is None and right is None:
            raise ValueError

        left_percent = int(left is not None) * 1000
        right_percent = int(right is not None) * 1000
        straight_percent = int(straight is not None) * 1

        # l_pts, l_speed, l_signs = left
        # s_pts, s_speed, s_signs = straight
        # r_pts, r_speed, r_signs = right

        _sum = left_percent + right_percent + straight_percent
        probabilities = left_percent / _sum, straight_percent / _sum, right_percent / _sum
        choice = np.random.choice(["left", "straight", "right"], p=probabilities)

        return choice

    def __len__(self) -> int:
        """
        Length of local path message
        :return: number
        """
        return len(self.message.points)
