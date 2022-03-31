import warnings
from collections import deque
from warnings import catch_warnings

import numpy as np
from typing import List, Tuple, Union, Optional

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.traffic_sign import (
    TrafficLightState,
    TrafficSignIDGermany,
)
from numpy import ndarray

import rospy
from paf_messages.msg import PafLaneletRoute, Point2D, PafTrafficSignal, PafRouteSection

from geometry_msgs.msg import Point
from .HelperFunctions import closest_index_of_point_list, dist, sparse_list_from_dense_pts, on_bridge
from .SpeedCalculator import SpeedCalculator
from .Spline import bezier_refit_all_with_tangents
from .MapManager import MapManager


class GlobalPath:
    """Everything not handled by ROS or Commonroad for global route planning and conversion is handled here"""

    SPEED_KMH_TO_MS = 1 / 3.6
    POINT_DISTANCE = 2.5
    TARGET_Z = 0

    def __init__(
        self,
        lanelet_network: LaneletNetwork = None,
        lanelet_ids: List[int] = None,
        target: Union[Point2D, tuple] = None,
        msg: PafLaneletRoute = None,
    ):
        """
        Initialisation can either happen with a given set of lanelet ids and CR-Information
        or with a PafLaneletRoute Message
        :param lanelet_network: CR network
        :param lanelet_ids: list of lanelet ids
        :param target: target point for planner
        :param msg: lanelet route message (for object creation in the local planner)
        """
        SpeedCalculator.set_limits()
        self._adjacent_lanelets = None
        self.lanelet_network = None
        self.route = None
        self.sections_visited = 0

        self.lanelet_network = lanelet_network
        self.lanelet_ids = lanelet_ids

        if msg is not None:
            self.route = msg
            self.target = Point(msg.target.x, msg.target.y, self.TARGET_Z)
        elif lanelet_ids is not None:
            if not hasattr(target, "x"):
                target = Point(*target) if len(target) == 3 else Point2D(*target)
            self.target = target if hasattr(target, "z") else Point(target.x, target.y, self.TARGET_Z)
            let = self.lanelet_network.find_lanelet_by_id(self.lanelet_ids[-1])
            target_index, d = closest_index_of_point_list(list(let.center_vertices), self.target)
            if False and target_index >= 0 and d < 5:
                prev = target_index
                target_index = min(target_index + 4, len(let.center_vertices) - 1)
                rospy.logerr(
                    f"[global planner] correcting target forward on lanelet "
                    f"{np.round(dist(let.center_vertices[target_index], let.center_vertices[prev]), 1)}m "
                    f"(for competition manager)..."
                )
                x, y = let.center_vertices[target_index]
                self.target = Point(x, y, self.target.z)

            if len(self.lanelet_ids) == 1:  # if target is on current lanelet, continue randomly forward, replan later
                while len(self.lanelet_ids) <= 2:
                    successors = self.lanelet_network.find_lanelet_by_id(self.lanelet_ids[-1]).successor
                    self.lanelet_ids.append(np.random.choice(successors))
            self._adjacent_lanelets = self._calc_adjacent_lanelet_routes()
        else:
            self.route = PafLaneletRoute()
            self.target = None
        self.signals_on_path = self.route.signals if self.route is not None else []
        if self.lanelet_network is None:
            self.lanelet_network = MapManager.get_current_scenario().lanelet_network

    def get_signals(self, section: Union[int, PafRouteSection], lane_idx: int) -> List[PafTrafficSignal]:
        """
        Get signals in a specific lane section and lane
        :param section: section number
        :param lane_idx: lane number
        :return: list of signals in lane
        """
        if type(section) is not PafRouteSection:
            section = self.route.sections[section]
        return [sig for sig in section.signals if sig.index == lane_idx]

    def get_local_path_values(self, section: int, lane_idx: int) -> Tuple[Point2D, float, List[PafTrafficSignal]]:
        """
        Get point, speed limit and signals of a lane section and lane number
        :param section: section number
        :param lane_idx: lane number
        :return: point, speed limit and signals
        """
        if type(section) is not PafRouteSection:
            try:
                section: PafRouteSection = self.route.sections[section]
            except IndexError:
                return Point2D(), -1, []
        try:
            return section.points[lane_idx], section.speed_limits[lane_idx], self.get_signals(section, lane_idx)
        except IndexError:
            lane_idx = 0
            return section.points[lane_idx], section.speed_limits[lane_idx], self.get_signals(section, lane_idx)

    def get_section_and_lane_indices(
        self,
        position: Optional[Point],
        not_found_threshold_meters: float = 100.0,
        min_section: int = None,
        max_section: int = None,
    ) -> Tuple[int, int]:
        """
        Calculate the closest section and lane index on the global path
        :param position: searching position
        :param not_found_threshold_meters: (-1,-1) is returned if this distance is exceeded
        :param min_section: start search from this section forward
        :param max_section: end search at this section forward
        :return: (section index, lane index)
        """
        if position is None:
            return -1, -1

        ref = (position.x, position.y)

        if not hasattr(position, "z"):
            raise RuntimeError("unable to search section without z value (bridges!)")

        if len(self.route.sections) == 0:
            return -1, -1

        if min_section is None:
            min_section = self.sections_visited
        if max_section is None:
            max_section = len(self.route.sections) - 1
        elif max_section < 0:
            max_section = len(self.route.sections) - max_section

        car_on_bridge = on_bridge(position, self.lanelet_network)

        relevant = [
            (i, s)
            for i, s in enumerate(self.route.sections)
            if car_on_bridge == s.on_bridge and min_section <= i <= max_section
        ]
        relevant_indices, relevant_sections = zip(*relevant)

        filter1 = [section.points[int(len(section.points) / 2 - 0.5)] for section in relevant_sections]
        idx, d = closest_index_of_point_list(filter1, ref)
        section = relevant_indices[idx]

        if d > not_found_threshold_meters:
            return -2, -2

        filter2 = self.route.sections[section].points
        lane, d = closest_index_of_point_list(filter2, ref)

        return section, lane

    def _calc_adjacent_lanelet_routes(self) -> List[Tuple[int, List[int], List[int]]]:
        """
        Calculates the drivable space as (adjacent) alternative lanelets on a route.
        :return: list of tuples (planned lanelet id, [list of drivable lanelets left, list of drivable lanelets right])
        """

        def get_right_lanelets_same_directions(_lanelet_id: int, lanelets: List[int] = None) -> List[int]:
            lanelet = self.lanelet_network.find_lanelet_by_id(_lanelet_id)
            if lanelets is None:
                lanelets = []
            if lanelet.adj_right_same_direction:
                lanelets.append(lanelet.adj_right)
                lanelets = get_right_lanelets_same_directions(lanelet.adj_right, lanelets)
            return lanelets

        def get_left_lanelets_same_directions(_lanelet_id: int, lanelets: List[int] = None) -> List[int]:
            lanelet = self.lanelet_network.find_lanelet_by_id(_lanelet_id)
            if lanelets is None:
                lanelets = []
            if lanelet.adj_left_same_direction:
                lanelets.append(lanelet.adj_left)
                lanelets = get_left_lanelets_same_directions(lanelet.adj_left, lanelets)
            return lanelets

        adj_route = []
        for lanelet_id in self.lanelet_ids:
            adj_left = get_left_lanelets_same_directions(lanelet_id)
            adj_right = get_right_lanelets_same_directions(lanelet_id)
            adj_route.append((lanelet_id, adj_left, adj_right))
        return adj_route

    @staticmethod
    def _locate_obj_on_lanelet(route_pts: np.ndarray, object_position: Tuple[float, float]) -> int:
        """
        Returns the closest index on the lanelet to the object position
        :param route_pts: list of vertices
        :param object_position: point in tuple format
        :return: index
        """
        return int(np.argmin([dist(x, object_position) for x in route_pts]))

    def _extract_traffic_signals(
        self, lanelet_id: int, new_vertices: np.ndarray
    ) -> Tuple[List[PafTrafficSignal], List[PafTrafficSignal]]:
        """
        searches lanelet for traffic signals and locates them on the vertices list
        :param lanelet_id: lanelet to search
        :param new_vertices: vertices list
        :return: traffic_signals, speed_limits
        """
        traffic_signals = []
        speed_limits = []

        lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
        vertices = lanelet.center_vertices

        # lane merge events
        if SpeedCalculator.APPLY_MERGING_RESET and len(lanelet.predecessor) > 1:
            merging_pt = vertices[0]
            paf_sign = PafTrafficSignal()
            paf_sign.type = "MERGE"
            idx = self._locate_obj_on_lanelet(vertices, merging_pt) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            paf_sign.value = SpeedCalculator.MERGE_SPEED_RESET
            paf_sign.point = Point2D(merging_pt[0], merging_pt[1])
            speed_limits += [paf_sign]

            self.signals_on_path.append(paf_sign)

        # all traffic signs
        for sign_id in lanelet.traffic_signs:
            paf_sign = PafTrafficSignal()
            sign = self.lanelet_network.find_traffic_sign_by_id(sign_id)
            paf_sign.type = sign.traffic_sign_elements[0].traffic_sign_element_id.value
            is_speed_limit = paf_sign.type == TrafficSignIDGermany.MAX_SPEED.value
            try:
                paf_sign.value = float(sign.traffic_sign_elements[0].additional_values[0])
                if is_speed_limit:
                    paf_sign.value *= self.SPEED_KMH_TO_MS
            except IndexError:
                paf_sign.value = SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED
            idx = self._locate_obj_on_lanelet(vertices, tuple(sign.position)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            paf_sign.point = Point2D(sign.position[0], sign.position[1])
            if is_speed_limit:
                speed_limits += [paf_sign]
            else:
                traffic_signals += [paf_sign]

            self.signals_on_path.append(paf_sign)

        # all traffic lights
        for light_id in lanelet.traffic_lights:
            light = self.lanelet_network.find_traffic_light_by_id(light_id)
            paf_sign = PafTrafficSignal()
            paf_sign.type = "LIGHT"
            paf_sign.value = 0.0
            red_value = 0
            for phase in light.cycle:
                if phase.state == TrafficLightState.RED:
                    red_value += phase.duration
                else:
                    paf_sign.value += phase.duration
            try:
                paf_sign.value /= paf_sign.value + red_value
            except ZeroDivisionError:
                paf_sign.value = -33
            paf_sign.point = Point2D(light.position[0], light.position[1])
            idx = self._locate_obj_on_lanelet(vertices, tuple(light.position)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            traffic_signals += [paf_sign]
            self.signals_on_path.append(paf_sign)

        traffic_signals = sorted(traffic_signals, key=lambda elem: elem.index)

        for i in range(25):
            pre = lanelet.predecessor
            lanelet_id = lanelet.predecessor[int(len(pre) / 2)]
            lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
            for sign_id in lanelet.traffic_signs:
                sign = self.lanelet_network.find_traffic_sign_by_id(sign_id)
                if sign.traffic_sign_elements[0].traffic_sign_element_id.name == TrafficSignIDGermany.MAX_SPEED.value:
                    _, speed_limits = self._extract_traffic_signals(lanelet.lanelet_id, np.array([[0.0, 0.0]]))
                    break
        return traffic_signals, speed_limits

    def get_lanelet_groups(self) -> List[Tuple[List[int], Tuple[int, int, int]]]:
        """
        Matches neighbouring lanes (self._adjacent_lanelets) to successors and calculates lane offsets for sections
        :return: list of tuples ( lanelets , anchors )
        """
        with catch_warnings():
            warnings.simplefilter("ignore", category=UserWarning)

            def match_lane(_l_id, successors):
                for _i, successor in enumerate(successors):
                    p0 = lanelets[_l_id].center_vertices[-1]
                    p1 = lanelets[successor].center_vertices[0]
                    if dist(p0, p1) < 0.5:  # check if last and first point are close
                        return _i, successor
                return None, None

            def sort_blob(blob):
                _li = [self.lanelet_network.find_lanelet_by_id(_id) for _id in blob]
                _lanelets = deque(_li[1:])
                _out = deque(_li[:1])
                for _let in _lanelets:
                    if _let in _out:
                        continue
                    for _let2 in _out:
                        if _let2.adj_left == _let.lanelet_id:
                            _out.appendleft(_let)
                            break
                        if _let2.adj_right == _let.lanelet_id:
                            _out.append(_let)
                            break

                return [_let.lanelet_id for _let in _out], _li

            blobs = []
            anchors = []
            lanelets = {}
            for a, b, c in self._adjacent_lanelets:
                li = b + [a] + c
                if len(blobs) > 0 and a in blobs[-1]:
                    continue
                li, __lanelets = sort_blob(li)
                for __let in __lanelets:
                    lanelets[__let.lanelet_id] = __let
                blobs.append(li)

            for blob0, blob1 in zip(blobs, blobs[1:]):
                anchor_l = 99  # leftmost lane to continue on
                anchor_r = -1  # rightmost lane to continue on
                shift_l = 99  # merging lanes from next segment on the left
                for i, l_id in enumerate(blob0):
                    successor_idx, _ = match_lane(l_id, blob1)
                    if successor_idx is None:
                        continue
                    anchor_l = min(i, anchor_l)
                    anchor_r = max(i, anchor_r)
                    shift_l = min(successor_idx, shift_l)
                if shift_l != 0 and len(blob0) == len(blob1) and anchor_l == 0 and anchor_r == len(blob1) - 1:
                    shift_l = 0
                anchors.append((shift_l, anchor_l, anchor_r))

            i, _ = closest_index_of_point_list(
                [self.lanelet_network.find_lanelet_by_id(l_id).center_vertices[-1] for l_id in blobs[-1]], self.target
            )
            anchors.append((0, i, i))
            return list(zip(blobs, anchors))

    def get_paf_lanelet_matrix(
        self, groups: List[Tuple[List[int], Tuple[int, int, int]]]
    ) -> List[Tuple[List[ndarray], Tuple[int, int, int], float]]:
        """
        Converts all relevant lanelet vertices to sparse and lane-parallel vertices
        :param groups: groups from get_lanelet_groups()
        :return: result
        """

        def normalize_lanelet_vertices(_pts: np.ndarray):
            return bezier_refit_all_with_tangents(_pts, GlobalPath.POINT_DISTANCE / resolution, convert_to_pts=False)

        def get_ref_indices(_vertices):
            _indices = []
            _remove = []
            _i_prev = 0
            _search_space = 25
            for _j, ref_pt in enumerate(ref_pts):
                _i_max = _i_prev + _search_space
                _i, _ = closest_index_of_point_list(_vertices[_i_prev:_i_max], tuple(ref_pt))
                _i += _i_prev
                _i_prev = _i
                if _i in _indices:
                    _remove.append(_j)
                    continue
                _indices.append(_i)
            return _indices, _remove

        out = []
        resolution = 10
        for blob, anchor in groups:
            lanelets = [self.lanelet_network.find_lanelet_by_id(lanelet) for lanelet in blob]
            distances = [
                [np.sum(np.abs(x - y)) for x, y in zip(lanelet.center_vertices, lanelet.center_vertices[1:])]
                for lanelet in lanelets
            ]
            tot_lengths = [np.sum(length) for length in distances]
            ref_i = np.argmax(tot_lengths)
            num_pts = max(5, int(tot_lengths[ref_i] / self.POINT_DISTANCE))
            avg_len = float(np.average(tot_lengths))
            vertices = []

            lanelet_vertices = []
            for i, (let, lengths) in enumerate(zip(lanelets, distances)):
                max_delta = np.max(lengths)
                if max_delta < self.POINT_DISTANCE:
                    lanelet_vertices += [let.center_vertices]
                else:
                    lanelet_vertices += [normalize_lanelet_vertices(let.center_vertices)]

            ref_pts = sparse_list_from_dense_pts(lanelet_vertices[ref_i], num_pts)

            for i, _vertices in enumerate(lanelet_vertices):
                if i == ref_i:
                    pts = ref_pts
                else:
                    indices, remove = get_ref_indices(_vertices)
                    if indices is None:
                        raise RuntimeError("[global planner] Unable to calculate pts of lanelet")

                    pts = np.array(_vertices)[indices]
                    ref_pts = np.delete(ref_pts, remove, axis=0)

                vertices.append(np.array(pts))
            out.append((vertices, anchor, avg_len))

        return out

    @staticmethod
    def _end_of_lanelet_blob(
        lanes_l: int, anchor_l: int, anchor_r: int, num_lanelets: int, num_lanelets_after: int
    ) -> bool:
        """
        determines, if a lanelet blob (list of lanelets) is proceeded by a blob,
        where the car is possibly forced to change lanes
        :param lanes_l: "overhanging" lanes to the left on next blob
        :param anchor_l: leftmost lane to continue on
        :param anchor_r: rightmost lane to continue on
        :param num_lanelets: current amount of lanes
        :param num_lanelets_after: following amount of lanes
        :return: boolean value
        """
        if lanes_l != 0 or anchor_l != 0 or anchor_r != num_lanelets - 1:
            return True
        return num_lanelets != num_lanelets_after

    def as_msg(self) -> PafLaneletRoute:
        """
        Calculate all the information for driving given route (given the lanelets)
        and create route object for local planner.
        :return:
        """
        if self.route is not None:
            return self.route

        msg = PafLaneletRoute()
        rospy.loginfo(
            f"[global planner] creating plan. target, ids = {(self.target.x, self.target.y)}, {self.lanelet_ids}"
        )
        groups = self.get_lanelet_groups()
        matrix = self.get_paf_lanelet_matrix(groups)
        last_limits = None
        self.signals_on_path = []
        for i, ((lanelet_id_list, (lanes_l, anchor_l, anchor_r)), (lanes, _, length)) in enumerate(zip(groups, matrix)):
            if last_limits is None:
                last_limits = [SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in lanes]
            else:
                missing = len(lanes) - len(last_limits)
                if missing > 0:
                    last_limits = last_limits + [SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in range(missing)]

            signals_per_lane, speed_limits_per_lane = [], []
            is_on_bridge = MapManager.lanelet_on_bridge(lanelet_id_list[0])

            for lanelet_id, vertices in zip(lanelet_id_list, lanes):
                signals, speed_limits = self._extract_traffic_signals(lanelet_id, vertices)
                signals_per_lane.append(signals)
                speed_limits_per_lane.append(speed_limits)

            for j, section_points in enumerate(zip(*lanes)):
                paf_section = PafRouteSection()
                paf_section.on_bridge = is_on_bridge
                paf_section.points = [Point2D(p[0], p[1]) for p in section_points]
                paf_section.speed_limits = [x for x in last_limits]
                paf_section.signals = []
                paf_section.target_lanes_index_distance = -1
                p1 = paf_section.points[int(len(paf_section.points) / 2)]
                if len(msg.sections) == 0:
                    p2 = p1
                else:
                    p2 = msg.sections[-1].points[int(len(msg.sections[-1].points) / 2 - 0.5)]
                paf_section.distance_from_last_section = dist(p1, p2)
                msg.distance += paf_section.distance_from_last_section
                for lane_number, (signal_lane, speed_lane) in enumerate(zip(signals_per_lane, speed_limits_per_lane)):
                    for signal in signal_lane:
                        if signal.index == j:
                            signal.index = lane_number
                            paf_section.signals.append(signal)
                            break
                        if signal.index > j:
                            break
                    for signal in speed_lane:
                        if signal.index == j:
                            paf_section.speed_limits[lane_number] = signal.value
                            break
                        if signal.index > j:
                            break
                paf_section.speed_limits = [
                    s * SpeedCalculator.SPEED_LIMIT_MULTIPLIER for s in paf_section.speed_limits
                ]
                last_limits = [x for x in paf_section.speed_limits]
                msg.sections.append(paf_section)

            next_lanelet_list_len = 1 if i == len(groups) - 1 else len(groups[i + 1][0])
            if i == len(groups) - 1 or self._end_of_lanelet_blob(
                lanes_l, anchor_l, anchor_r, len(lanelet_id_list), next_lanelet_list_len
            ):
                # shift speed limit lanes
                last_limits_new = [-1 for _ in range(anchor_l, anchor_r + 1)]
                for index in range(len(last_limits_new)):
                    if index + 1 <= lanes_l:
                        continue
                    index_old_limits = index - lanes_l + anchor_l
                    if index_old_limits < 0:
                        continue
                    try:
                        last_limits_new[index] = last_limits[index_old_limits]
                    except IndexError:
                        break

                unknown_limits = [
                    SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in range(msg.sections[-1].target_lanes_left_shift)
                ]
                last_limits = unknown_limits + last_limits_new

                target_lanes = list(range(anchor_l, anchor_r + 1))
                if len(target_lanes) == 0:
                    RuntimeError(f"target lanes unspecified (lanes {anchor_l} to {anchor_r})")
                target_lanes_left_shift = lanes_l
                distance_to_target = 0
                for target_lanes_index_distance, paf_section in enumerate(reversed(msg.sections)):
                    if len(paf_section.target_lanes) > 0:
                        break
                    paf_section.target_lanes = target_lanes
                    paf_section.target_lanes_index_distance = target_lanes_index_distance
                    paf_section.target_lanes_distance = distance_to_target
                    paf_section.target_lanes_left_shift = target_lanes_left_shift

                    distance_to_target += paf_section.distance_from_last_section
                    for target in paf_section.target_lanes:
                        if not 0 <= target < len(paf_section.points):
                            raise RuntimeError(f"target_lanes incorrect: {anchor_l}->{anchor_r}")
        msg.target = self.target

        self.route = msg

        for i, paf_section in enumerate(msg.sections):
            if len(paf_section.target_lanes) == 0:
                raise RuntimeError(
                    f"[global planner] no target lanes specified in section {i}, "
                    f"{[s.target_lanes for s in msg.sections]}"
                )
            if len(paf_section.points) == 0:
                rospy.logerr(f"[global planner] section {i} has no points")

        try:
            from paf_messages.msg import PafTopDownViewPointSet

            pts1 = PafTopDownViewPointSet()
            pts1.label = "signals_global"
            pts1.points = [x.point for x in self.signals_on_path]
            pts1.color = (0, 255, 0)
            rospy.Publisher("/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1).publish(pts1)
        except rospy.exceptions.ROSException:
            pass
        msg.signals = self.signals_on_path
        self.route = msg
        return msg

    def __len__(self) -> int:
        """
        Length of global path message
        :return:
        """
        if self.route is None:
            return 0
        return len(self.route.sections)
