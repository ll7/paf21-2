import numpy as np
from typing import List, Tuple

from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.traffic_sign import (
    SupportedTrafficSignCountry as Country,
    TrafficLightState,
    TrafficSignIDGermany,
)
from commonroad.scenario.traffic_sign_interpreter import TrafficSigInterpreter

import rospy
from paf_messages.msg import PafLaneletRoute, Point2D, PafTrafficSignal, PafRouteSection

from .HelperFunctions import dist_pts, closest_index_of_point_list, dist, sparse_list_from_dense_pts
from .SpeedCalculator import SpeedCalculator


class GlobalPath:
    SPEED_KMH_TO_MS = 1 / 3.6
    MERGE_SPEED_RESET = 50 / 3.6
    APPLY_MERGING_RESET = True
    POINT_DISTANCE = 2.5

    def __init__(
        self,
        lanelet_network: LaneletNetwork = None,
        lanelet_ids: List[int] = None,
        target=None,
        traffic_sign_country: Country = Country.GERMANY,
        msg: PafLaneletRoute = PafLaneletRoute(),
    ):
        SpeedCalculator.set_limits()
        if lanelet_network is None:
            self.route = msg
            self._graph = None
            self._adjacent_lanelets = None
            self.lanelet_ids = None
            self._lanelet_network = None
            self.target = msg.target
        else:
            if hasattr(target, "x"):
                target = target.x, target.y
            assert lanelet_ids is not None and lanelet_network is not None
            self.target = target if hasattr(target, "x") else Point2D(target[0], target[1])
            self.lanelet_ids = lanelet_ids
            self._lanelet_network = lanelet_network
            self._adjacent_lanelets = self._calc_adjacent_lanelet_routes()
            self._graph = None
            self._traffic_sign_interpreter = TrafficSigInterpreter(traffic_sign_country, lanelet_network)
            self.route = None
            self.signal_positions = []

    def __len__(self):
        if self.route is None:
            return 0
        return len(self.route.sections)

    def get_signals(self, section, lane_idx):
        if type(section) is not PafRouteSection:
            section = self.route.sections[section]
        return [sig for sig in section.signals if sig.index == lane_idx]

    def get_local_path_values(self, section, lane_idx):
        if type(section) is not PafRouteSection:
            try:
                section: PafRouteSection = self.route.sections[section]
            except IndexError:
                return -1, -1
        try:
            return section.points[lane_idx], section.speed_limits[lane_idx], self.get_signals(section, lane_idx)
        except IndexError:
            lane_idx = 0
            return section.points[lane_idx], section.speed_limits[lane_idx], self.get_signals(section, lane_idx)

    def get_section_and_lane_indices(self, position, not_found_threshold_meters=100, min_section=0):
        if hasattr(position, "x"):
            ref = (position.x, position.y)
        else:
            ref = position

        if len(self.route.sections) == 0:
            return -1, -1

        filter1 = [section.points[int(len(section.points) / 2 - 0.5)] for section in self.route.sections[min_section:]]
        section, d = closest_index_of_point_list(filter1, ref)

        if d > not_found_threshold_meters:
            return -2, -2

        filter2 = self.route.sections[section].points
        lane, d = closest_index_of_point_list(filter2, ref)

        return section + min_section, lane

    def _calc_adjacent_lanelet_routes(self) -> List[Tuple[int, List[int], List[int]]]:
        """
        Calculates the drivable (adjacent) alternative lanelets on a route.
        :return: list of tuples (planned lanelet id, [list of drivable lanelets left, list of drivable lanelets right])
        """

        def get_right_lanelets_same_directions(_lanelet_id, lanelets=None):
            lanelet = self._lanelet_network.find_lanelet_by_id(_lanelet_id)
            if lanelets is None:
                lanelets = []
            if lanelet.adj_right_same_direction:
                lanelets.append(lanelet.adj_right)
                lanelets = get_right_lanelets_same_directions(lanelet.adj_right, lanelets)
            return lanelets

        def get_left_lanelets_same_directions(_lanelet_id, lanelets=None):
            lanelet = self._lanelet_network.find_lanelet_by_id(_lanelet_id)
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

    def _calc_lane_change_graph(self):
        """
        Calculates the options to drive on for each lanelet
        :return: dict { lanelet_id1: [ allowed_other_lanelet_id, ...], ... }
        """
        print("WARNING _calc_lane_change_graph may be wrong")

        def valid_successor_from_list(check_lanelet: Lanelet, l_id_check: list):
            try:
                l_id_check.remove(check_lanelet.lanelet_id)
            except ValueError:
                ...
            if check_lanelet.lanelet_id in l_id_check:
                return check_lanelet.lanelet_id
            for l_successor in check_lanelet.successor:
                successor_lanelet = self._lanelet_network.find_lanelet_by_id(l_successor)
                if l_successor not in l_id_check or 0.5 < dist(
                    successor_lanelet.center_vertices[0], check_lanelet.center_vertices[-1]
                ):
                    continue
                return l_successor
            return None

        lane_change_graph = {}
        for (planned_id_0, l_ids_0, r_ids_0), (planned_id_1, l_ids_1, r_ids_1) in zip(
            self._adjacent_lanelets, self._adjacent_lanelets[1:]
        ):
            possible_0 = [planned_id_0] + l_ids_0 + r_ids_0
            possible_1 = [planned_id_1] + l_ids_1 + r_ids_1
            for l_id in possible_0:
                lanelet = self._lanelet_network.find_lanelet_by_id(l_id)
                can_turn_left = lanelet.adj_left in (possible_1 + possible_0)
                can_turn_right = lanelet.adj_right in (possible_1 + possible_0)

                id_straight = valid_successor_from_list(lanelet, possible_1)
                id_l = lanelet.adj_left if can_turn_left else None
                id_r = lanelet.adj_right if can_turn_right else None
                lane_change_graph[l_id] = (id_l, id_straight, id_r)

        return lane_change_graph

    def get_route_instructions(self):
        """
        Calculates the route in a human-readable format
        :return:
        """
        route_ids = self.lanelet_ids
        out = f"start route from {route_ids[0]}\n"
        for lanelet_id, next_lanelet_id in zip(route_ids, route_ids[1:]):
            lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
            l_set = frozenset([lanelet_id])
            speed_limit = self._traffic_sign_interpreter.speed_limit(l_set)
            speed_minimum = self._traffic_sign_interpreter.required_speed(l_set)
            speed_minimum = 0 if speed_minimum is None else speed_minimum

            for sign in lanelet.traffic_signs:
                sign = self._lanelet_network.find_traffic_sign_by_id(sign)
                sign_name = sign.traffic_sign_elements[0].traffic_sign_element_id.name
                if sign_name == "MAX_SPEED":
                    continue
                # sign_value = sign.traffic_sign_elements[0].traffic_sign_element_id.name
                out += f"sign on road: {sign_name}\n"
            for light in lanelet.traffic_lights:
                print(f"traffic light: {light}\n")

            # next_lanelet = lanelet_network.find_lanelet_by_id(next_lanelet_id)
            if lanelet.adj_left == next_lanelet_id:
                out += f"left lane change to {next_lanelet_id} (speed_limit={speed_limit}, min={speed_minimum})\n"
            elif lanelet.adj_right == next_lanelet_id:
                out += f"right lane change to {next_lanelet_id} (speed_limit={speed_limit}, min={speed_minimum})\n"
            elif len(lanelet.successor) == 1 and lanelet.successor[0] == next_lanelet_id:
                out += f"go straight to {next_lanelet_id} (speed_limit={speed_limit}, min={speed_minimum})\n"
            elif len(lanelet.successor) > 1:
                out += f"intersection, go to {next_lanelet_id} (speed_limit={speed_limit}, min={speed_minimum})\n"
            else:
                out += (
                    f"?? go from {lanelet_id} to {next_lanelet_id} "
                    f"(speed_limit={speed_limit}, min={speed_minimum})\n"
                )

            if len(lanelet.predecessor) > 1:
                print(f"merging with {len(lanelet.predecessor) - 1} other lane(s)")

        out += f"Route completed at {route_ids[-1]}"
        return out

    @staticmethod
    def _locate_obj_on_lanelet(route_pts: np.ndarray, object_position: List[float]):
        if len(route_pts) == 0:
            return -1
        return int(np.argmin([dist(x, object_position) for x in route_pts]))

    def _calc_lane_rightmost_leftmost_routes(self) -> Tuple[List[int], List[int]]:
        """
        Calculates alternatives to the current route
        :return: tuple (leftmost_route, rightmost_route)
        """
        if self._graph is None:
            self._calc_lane_change_graph()

        def calc_extremum(is_left_extremum):
            l_id = self.lanelet_ids[0]
            target_id = self.lanelet_ids[-1]
            route_ = []

            def test_direction(l_dir):
                # don't go back left when forced right before
                return l_dir is not None and (len(route_) < 2 or route_[-2] != l_dir)

            counter = 0
            while l_id != target_id:
                if is_left_extremum:
                    a, b, c = self._graph[l_id]
                else:
                    c, b, a = self._graph[l_id]
                step = None
                assert l_id not in (a, b, c), "Circular dependencies are not permitted"
                if test_direction(a):
                    step = a
                elif test_direction(b):
                    step = b
                elif test_direction(c):
                    step = c
                else:
                    # backtrack and choose other direction
                    while len([x for x in self._graph[l_id] if x is not None]) < 2:
                        assert len(route_) > 0
                        del route_[-1]
                        l_id = route_[-1]
                    if is_left_extremum:
                        a, b, c = self._graph[l_id]
                    else:
                        c, b, a = self._graph[l_id]

                    if test_direction(b):
                        step = b
                    elif test_direction(c):
                        step = c
                assert step is not None
                route_.append(step)
                l_id = step
                assert counter < 10000, "Unable to find exit of route graph"
            return route_

        return calc_extremum(is_left_extremum=True), calc_extremum(is_left_extremum=False)

    def _extract_traffic_signals(self, lanelet_id, new_vertices):
        traffic_signals = []
        speed_limits = []

        lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
        vertices = lanelet.center_vertices

        # lane merge events
        if self.APPLY_MERGING_RESET and len(lanelet.predecessor) > 1:
            merging_pt = vertices[0]
            paf_sign = PafTrafficSignal()
            paf_sign.type = "MERGE"
            idx = self._locate_obj_on_lanelet(vertices, list(merging_pt)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            paf_sign.value = self.MERGE_SPEED_RESET
            speed_limits += [paf_sign]

            # self.signal_positions.append(Point2D(merging_pt[0], merging_pt[1]))

        # all traffic signs
        for sign_id in lanelet.traffic_signs:
            paf_sign = PafTrafficSignal()
            sign = self._lanelet_network.find_traffic_sign_by_id(sign_id)
            paf_sign.type = sign.traffic_sign_elements[0].traffic_sign_element_id.value
            is_speed_limit = paf_sign.type == TrafficSignIDGermany.MAX_SPEED.value
            try:
                paf_sign.value = float(sign.traffic_sign_elements[0].additional_values[0])
                if is_speed_limit:
                    paf_sign.value *= self.SPEED_KMH_TO_MS
            except IndexError:
                paf_sign.value = SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED
            idx = self._locate_obj_on_lanelet(vertices, list(sign.position)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            if is_speed_limit:
                speed_limits += [paf_sign]
            else:
                traffic_signals += [paf_sign]

            self.signal_positions.append(Point2D(sign.position[0], sign.position[1]))

        # all traffic lights
        for light_id in lanelet.traffic_lights:
            light = self._lanelet_network.find_traffic_light_by_id(light_id)
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
            idx = self._locate_obj_on_lanelet(vertices, list(light.position)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            traffic_signals += [paf_sign]
            self.signal_positions.append(Point2D(light.position[0], light.position[1]))

        traffic_signals = sorted(traffic_signals, key=lambda elem: elem.index)

        for i in range(25):
            pre = lanelet.predecessor
            lanelet_id = lanelet.predecessor[int(len(pre) / 2)]
            lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
            for sign_id in lanelet.traffic_signs:
                sign = self._lanelet_network.find_traffic_sign_by_id(sign_id)
                if sign.traffic_sign_elements[0].traffic_sign_element_id.name == TrafficSignIDGermany.MAX_SPEED.value:
                    _, speed_limits = self._extract_traffic_signals(lanelet.lanelet_id, [[0, 0]])
                    break
        return traffic_signals, speed_limits

    def get_lanelet_groups(self):
        def match_lane(lanelet_needle, successors):
            successor = None
            for _i, successor in enumerate(successors):
                if successor in lanelet_needle.successor:
                    return _i, successor
            return None, successor

        blobs = []
        anchors = []
        for a, b, c in self._adjacent_lanelets:
            li = b + [a] + c
            if len(blobs) > 0 and a in blobs[-1]:
                continue
            blobs.append(li)

        for blob0, blob1 in zip(blobs, blobs[1:]):
            anchor_l = 99  # leftmost lane to continue on
            anchor_r = -1  # rightmost lane to continue on
            shift_l = 99  # merging lanes from next segment on the left
            for i, l_id in enumerate(blob0):
                lanelet = self._lanelet_network.find_lanelet_by_id(l_id)
                successor_idx, successor_id = match_lane(lanelet, blob1)
                if successor_idx is None:
                    continue
                anchor_l = min(i, anchor_l)
                anchor_r = max(i, anchor_r)
                shift_l = min(successor_idx, shift_l)

            anchors.append((shift_l, anchor_l, anchor_r))

        i, _ = closest_index_of_point_list(
            [self._lanelet_network.find_lanelet_by_id(l_id).center_vertices[-1] for l_id in blobs[-1]], self.target
        )
        anchors.append((0, i, i))

        return [x for x in zip(blobs, anchors)]

    def get_paf_lanelet_matrix(self, groups):
        out = []
        for blob, anchor in groups:
            lanelets = [self._lanelet_network.find_lanelet_by_id(lanelet) for lanelet in blob]
            lengths = [
                [np.abs(x - y) for x, y in zip(lanelet.center_vertices, lanelet.center_vertices[1:])]
                for lanelet in lanelets
            ]
            tot_lengths = [np.sum(length) for length in lengths]
            ref_i = np.argmax(tot_lengths)
            num_pts = int(tot_lengths[ref_i] / self.POINT_DISTANCE)
            avg_len = np.average(tot_lengths)
            vertices = []

            ref_pts = sparse_list_from_dense_pts(lanelets[ref_i].center_vertices, num_pts)

            def get_ref_indices(_vertices):
                _indices = []
                _remove = []
                _i_prev = 0
                _search_space = 25
                for _j, ref_pt in enumerate(ref_pts):
                    _i_max = _i_prev + _search_space
                    _i, _ = closest_index_of_point_list(_vertices[_i_prev:_i_max], ref_pt)
                    _i += _i_prev
                    _i_prev = _i
                    if _i in _indices:
                        _remove.append(_j)
                        continue
                    _indices.append(_i)
                return _indices, _remove

            for i, lanelet in enumerate(lanelets):
                if i == ref_i:
                    pts = ref_pts
                else:
                    _vertices = lanelet.center_vertices
                    # _vertices = bezier_refit_all_with_tangents(_vertices, ds=.1, convert_to_pts=False)
                    indices, remove = get_ref_indices(_vertices)
                    if indices is None:
                        raise RuntimeError("[global planner] Unable to calculate pts of lanelet")

                    pts = np.array(_vertices)[indices]
                    ref_pts = np.delete(ref_pts, remove, axis=0)

                vertices.append(np.array(pts))
            out.append((vertices, anchor, avg_len))
        return out

    @staticmethod
    def _end_of_segment(lanes_l, anchor_l, anchor_r, num_lanelets):
        if lanes_l != 0 or anchor_l != 0 or anchor_r != num_lanelets - 1:
            return True
        return False

    def as_msg(self) -> PafLaneletRoute:

        if self.route is not None:
            return self.route

        msg = PafLaneletRoute()
        rospy.loginfo(
            f"[global planner] creating plan to {[self.target.x, self.target.y]} "
            f"with lanelet ids {self.lanelet_ids}"
        )
        groups = self.get_lanelet_groups()
        matrix = self.get_paf_lanelet_matrix(groups)
        last_limits = None
        self.signal_positions = []
        for i, ((lanelet_id_list, (lanes_l, anchor_l, anchor_r)), (lanes, _, length)) in enumerate(zip(groups, matrix)):
            if last_limits is None:
                last_limits = [SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in lanes]
            else:
                missing = len(lanes) - len(last_limits)
                if missing > 0:
                    last_limits = last_limits + [SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in range(missing)]

            signals_per_lane, speed_limits_per_lane = [], []
            for lanelet_id, vertices in zip(lanelet_id_list, lanes):
                signals, speed_limits = self._extract_traffic_signals(lanelet_id, vertices)
                signals_per_lane.append(signals)
                speed_limits_per_lane.append(speed_limits)

            for j, section_points in enumerate(zip(*lanes)):
                paf_section = PafRouteSection()
                paf_section.points = [Point2D(p[0], p[1]) for p in section_points]
                paf_section.speed_limits = [x for x in last_limits]
                paf_section.signals = []
                paf_section.target_lanes_index_distance = -1
                p1 = paf_section.points[int(len(paf_section.points) / 2)]
                if len(msg.sections) == 0:
                    p2 = p1
                else:
                    p2 = msg.sections[-1].points[int(len(msg.sections[-1].points) / 2 - 0.5)]
                paf_section.distance_from_last_section = dist_pts(p1, p2)
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
                    # if len(paf_section.signals) == 0:
                    #     paf_section.signals.append(dummy_signal)

                last_limits = [x for x in paf_section.speed_limits]
                # print(len(section_points))
                msg.sections.append(paf_section)

            if i == len(groups) - 1:
                self.route = msg
                section_idx, target_lane = self.get_section_and_lane_indices(self.target)
                self.route.sections = self.route.sections[: section_idx + 1]
                self.route.sections[-1].target_lanes = [target_lane]

            if i == len(groups) - 1 or self._end_of_segment(lanes_l, anchor_l, anchor_r, len(lanelet_id_list)):
                # shift speed limit lanes
                # print(lanes_l, anchor_l, anchor_r)
                last_limits_new = [-1 for _ in range(anchor_l, anchor_r + 1)]
                for index, entry in enumerate(last_limits_new):
                    if index + 1 <= lanes_l:
                        continue
                    index_old_limits = index - lanes_l + anchor_l
                    if index_old_limits < 0:
                        continue
                    try:
                        last_limits_new[index] = last_limits[index_old_limits]
                    except IndexError:
                        break
                # print(last_limits_new)
                unknown_limits = [
                    SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED for _ in range(msg.sections[-1].target_lanes_left_shift)
                ]
                last_limits = unknown_limits + last_limits_new

                # set target lanes etc. for all previous lanes
                target_lanes = list(range(anchor_l, anchor_r + 1))
                target_lanes_left_shift = lanes_l
                distance_to_target = 0
                for target_lanes_index_distance, paf_section in enumerate(reversed(msg.sections)):
                    if paf_section.target_lanes_index_distance != -1:
                        break
                    paf_section.target_lanes = target_lanes
                    paf_section.target_lanes_index_distance = target_lanes_index_distance
                    paf_section.target_lanes_distance = distance_to_target
                    paf_section.target_lanes_left_shift = target_lanes_left_shift

                    distance_to_target += paf_section.distance_from_last_section
                    for target in paf_section.target_lanes:
                        if not 0 <= target < len(paf_section.points):
                            rospy.logerr_throttle(1, f"target_lanes incorrect: {anchor_l}->{anchor_r}")
                            paf_section.target_lanes = [0]
        msg.target = self.target

        self.route = msg
        for i, paf_section in enumerate(msg.sections):
            # rospy.logwarn(f"section {i}: {[(round(p.x), round(p.y)) for p in paf_section.points]}, "
            #               f"{paf_section.target_lanes}, {paf_section.target_lanes_left_shift}, "
            #               f"{paf_section.target_lanes_distance}")
            if len(paf_section.target_lanes) == 0:
                paf_section.target_lanes = [0]
                rospy.logerr_throttle(1, f"[global planner] no target lanes specified in section {i}")
            if len(paf_section.points) == 0:
                rospy.logerr(f"[global planner] section {i} has no points")

        # rospy.logwarn(groups)

        try:
            from paf_messages.msg import PafTopDownViewPointSet

            pts1 = PafTopDownViewPointSet()
            pts1.label = "signals_global"
            pts1.points = self.signal_positions
            pts1.color = (0, 255, 0)
            rospy.Publisher("/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1).publish(pts1)
        except rospy.exceptions.ROSException:
            pass

        return msg


# Point2D[] points
# float32[] speed_limits
# PafTrafficSignal[] signals
# target_lanes[]
# int64 target_lanes_index_distance
# int32 target_lanes_left_shift
# float32 distance_from_last_section
# float32 target_lanes_distance
