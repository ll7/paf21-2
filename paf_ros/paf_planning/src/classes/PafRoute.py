import numpy as np
from typing import List, Tuple

from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.traffic_sign import (
    SupportedTrafficSignCountry as Country,
    TrafficLightState,
    TrafficSignIDGermany,
)
from commonroad.scenario.traffic_sign_interpreter import TrafficSigInterpreter
from commonroad_route_planner.route import Route as CommonroadRoute

import rospy
from paf_messages.msg import PafLaneletRoute, Point2D, PafTrafficSignal, PafRouteSection
from .Spline import calc_spline_course_from_point_list


class PafRoute:
    SPEED_KMH_TO_MS = 1 / 3.6

    def __init__(self, route: CommonroadRoute, target, traffic_sign_country: Country = Country.GERMANY):
        if hasattr(target, "x"):
            target = target.x, target.y
        self.target = Point2D(target[0], target[1])
        self._traffic_sign_interpreter = TrafficSigInterpreter(traffic_sign_country, route.scenario.lanelet_network)
        self.route = route
        self._adjacent_lanelets = self._calc_adjacent_lanelet_routes()
        self.graph = self._calc_lane_change_graph()

    def _calc_adjacent_lanelet_routes(self) -> List[Tuple[int, List[int], List[int]]]:
        """
        Calculates the drivable (adjacent) alternative lanelets on a route.
        :return: list of tuples (planned lanelet id, [list of drivable lanelets left, list of drivable lanelets right])
        """

        def get_right_lanelets_same_directions(_lanelet_id, lanelets=None):
            lanelet = self.route.scenario.lanelet_network.find_lanelet_by_id(_lanelet_id)
            if lanelets is None:
                lanelets = []
            if lanelet.adj_right_same_direction:
                lanelets.append(lanelet.adj_right)
                lanelets = get_right_lanelets_same_directions(lanelet.adj_right, lanelets)
            return lanelets

        def get_left_lanelets_same_directions(_lanelet_id, lanelets=None):
            lanelet = self.route.scenario.lanelet_network.find_lanelet_by_id(_lanelet_id)
            if lanelets is None:
                lanelets = []
            if lanelet.adj_left_same_direction:
                lanelets.append(lanelet.adj_left)
                lanelets = get_left_lanelets_same_directions(lanelet.adj_left, lanelets)
            return lanelets

        adj_route = []
        for lanelet_id in self.route.list_ids_lanelets:
            adj_left = get_left_lanelets_same_directions(lanelet_id)
            adj_right = get_right_lanelets_same_directions(lanelet_id)
            adj_route.append((lanelet_id, adj_left, adj_right))
        return adj_route

    def _calc_lane_change_graph(self):
        """
        Calculates the options to drive on for each lanelet
        :return: dict { lanelet_id1: [ allowed_other_lanelet_id, ...], ... }
        """

        def valid_successor_from_list(let: Lanelet, l_id_check: list):
            try:
                l_id_check.remove(let.lanelet_id)
            except ValueError:
                ...
            if let.lanelet_id in l_id_check:
                return let.lanelet_id
            for l_successor in let.successor:
                if l_successor in l_id_check:
                    return l_successor
            return None

        lane_change_graph = {}
        for (planned_id_0, l_ids_0, r_ids_0), (planned_id_1, l_ids_1, r_ids_1) in zip(
                self._adjacent_lanelets, self._adjacent_lanelets[1:]
        ):
            possible_0 = [planned_id_0] + l_ids_0 + r_ids_0
            possible_1 = [planned_id_1] + l_ids_1 + r_ids_1
            for l_id in possible_0:
                lanelet = self.route.scenario.lanelet_network.find_lanelet_by_id(l_id)
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
        route_ids = self.route.list_ids_lanelets
        out = f"start route from {route_ids[0]}\n"
        for lanelet_id, next_lanelet_id in zip(route_ids, route_ids[1:]):
            lanelet = self.route.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            l_set = frozenset([lanelet_id])
            speed_limit = self._traffic_sign_interpreter.speed_limit(l_set)
            speed_minimum = self._traffic_sign_interpreter.required_speed(l_set)
            speed_minimum = 0 if speed_minimum is None else speed_minimum

            for sign in lanelet.traffic_signs:
                sign = self.route.scenario.lanelet_network.find_traffic_sign_by_id(sign)
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
        def dist(a, b):
            x1, y1 = a
            x2, y2 = b
            return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        if len(route_pts) == 0:
            return -1
        return int(np.argmin([dist(x, object_position) for x in route_pts]))

    def _calc_lane_rightmost_leftmost_routes(self) -> Tuple[List[int], List[int]]:
        """
        Calculates alternatives to the current route
        :return: tuple (leftmost_route, rightmost_route)
        """

        def calc_extremum(is_left_extremum):
            l_id = self.route.list_ids_lanelets[0]
            target_id = self.route.list_ids_lanelets[-1]
            route_ = []

            def test_direction(l_dir):
                # don't go back left when forced right before
                return l_dir is not None and (len(route_) < 2 or route_[-2] != l_dir)

            counter = 0
            while l_id != target_id:
                if is_left_extremum:
                    a, b, c = self.graph[l_id]
                else:
                    c, b, a = self.graph[l_id]
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
                    while len([x for x in self.graph[l_id] if x is not None]) < 2:
                        assert len(route_) > 0
                        del route_[-1]
                        l_id = route_[-1]
                    if is_left_extremum:
                        a, b, c = self.graph[l_id]
                    else:
                        c, b, a = self.graph[l_id]

                    if test_direction(b):
                        step = b
                    elif test_direction(c):
                        step = c
                assert step is not None
                route_.append(step)
                l_id = step
                assert counter < 10000, "Unable to find exit of route graph"
            print(route_)
            return route_

        return calc_extremum(is_left_extremum=True), calc_extremum(is_left_extremum=False)

    def _extract_traffic_signals(self, lanelet_id, new_vertices):
        traffic_signals = []
        speed_limits = []

        lanelet = self.route.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        vertices = lanelet.center_vertices

        # lane merge events
        if len(lanelet.predecessor) > 1:
            merging_pt = vertices[0]
            paf_sign = PafTrafficSignal()
            paf_sign.type = "MERGE"
            idx = self._locate_obj_on_lanelet(vertices, list(merging_pt)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            paf_sign.value = 0
            speed_limits += [paf_sign]

        # all traffic signs
        for sign_id in lanelet.traffic_signs:
            paf_sign = PafTrafficSignal()
            sign = self.route.scenario.lanelet_network.find_traffic_sign_by_id(sign_id)
            paf_sign.type = sign.traffic_sign_elements[0].traffic_sign_element_id.value
            is_speed_limit = paf_sign.type == TrafficSignIDGermany.MAX_SPEED.value
            try:
                paf_sign.value = float(sign.traffic_sign_elements[0].additional_values[0])
                if is_speed_limit:
                    paf_sign.value *= self.SPEED_KMH_TO_MS
            except IndexError:
                paf_sign.value = -1.0
            idx = self._locate_obj_on_lanelet(vertices, list(sign.position)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            if is_speed_limit:
                speed_limits += [paf_sign]
            else:
                traffic_signals += [paf_sign]

        # all traffic lights
        for light_id in lanelet.traffic_lights:
            light = self.route.scenario.lanelet_network.find_traffic_light_by_id(light_id)
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
                paf_sign.value = -1.0
            idx = self._locate_obj_on_lanelet(vertices, list(light.position)) / len(vertices) * len(new_vertices)
            paf_sign.index = int(idx)
            traffic_signals += [paf_sign]
        traffic_signals = sorted(traffic_signals, key=lambda elem: elem.index)
        return traffic_signals, speed_limits

    def _get_lanelet_groups(self, l_list):
        out = {}

        def get_lanelet_blob(from_lanelet):
            other = from_lanelet
            blob = []
            anchor_l = 0  # leftmost lane to continue on
            anchor_r = -1  # rightmost lane to continue on
            shift_l = 0  # merging lanes from next segment on the left
            while other in self.graph:  # go to leftmost lanelet
                (left, straight, _) = self.graph[other]
                if left is None:
                    break
                other = left
            other2 = other
            while other2 in self.graph:  # add from left to right
                if other2 not in blob:
                    blob.append(other2)
                (_, straight, other2) = self.graph[other2]
                if other2 is None:
                    break
            if len(blob) == 0:
                blob = (other,)
                return hash(_blob), blob, (0, 0, 0)
            other3 = blob[0]
            while other3 in self.graph and self.graph[other3][1] is None:
                other3 = self.graph[other3][2]
                anchor_l += 1
                anchor_r += 1
            next_straight = self.graph[other3][1] if other3 in self.graph else None
            while other3 in self.graph and self.graph[other3][1] is not None:
                other3 = self.graph[other3][2]
                anchor_r += 1
            while next_straight in self.graph and self.graph[next_straight][0] is not None:
                shift_l += 1

            blob = tuple(blob)
            h = hash(blob)

            return h, blob, (shift_l, anchor_l, anchor_r)

        for lanelet in l_list:
            _h, _blob, _anchor = get_lanelet_blob(lanelet)
            if _anchor is not None and _h not in out:
                out[_h] = (_blob, _anchor)
        out = list(out.values())
        return out

    def get_paf_lanelet_matrix(self, groups, distance_m=5):
        n: LaneletNetwork = self.route.scenario.lanelet_network
        out = []
        for blob, anchor in groups:
            lanelets = [n.find_lanelet_by_id(lanelet) for lanelet in blob]
            lengths = [
                np.sum([np.abs(x - y) for x, y in zip(lanelet.center_vertices, lanelet.center_vertices[1:])])
                for lanelet in lanelets
            ]
            avg_len = np.average(lengths)
            num_pts = max(2, avg_len / distance_m)
            vertices = []
            for i, (lanelet, length) in enumerate(zip(lanelets, lengths)):
                pts = lanelet.center_vertices[::10]
                if len(pts) < 3:
                    pts = lanelet.center_vertices
                new_pts = calc_spline_course_from_point_list(pts, length / num_pts / 5)[:-1]
                vertices.append(np.array(new_pts[::5]))
            out.append((np.array(vertices), anchor, avg_len))
        return out

    def as_msg(self) -> PafLaneletRoute:
        msg = PafLaneletRoute()
        groups = self._get_lanelet_groups(self.route.list_ids_lanelets)
        distance_m = 5
        last_limits = None

        for i, ((lanelet_id_list, (lanes_l, anchor_l, anchor_r)), (lanes, _, length)) in enumerate(
                zip(groups, self.get_paf_lanelet_matrix(groups, distance_m=distance_m))
        ):
            if last_limits is None:
                last_limits = [-1 for _ in lanes]
            print(lanelet_id_list)

            signals_per_lane, speed_limits_per_lane = [], []
            for lanelet_id, vertices in zip(lanelet_id_list, lanes):
                signals, speed_limits = self._extract_traffic_signals(lanelet_id, vertices)
                signals_per_lane.append(signals)
                speed_limits_per_lane.append(speed_limits)

            for j, section_points in enumerate(zip(*lanes)):
                paf_section = PafRouteSection()
                paf_section.points = [Point2D(p[0], p[1]) for p in section_points]
                paf_section.speed_limits = [x for x in last_limits]
                paf_section.target_lanes_index_distance = -1
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
                            paf_section.speed_limits[lane_number] = signal.value  # -1 = unknown, 0 = merge, else x m/s
                            break
                        if signal.index > j:
                            break
                last_limits = [x for x in paf_section.speed_limits]
                # print(len(section_points))
                msg.sections.append(paf_section)

            if lanes_l != 0 or anchor_l != 0 or anchor_r != len(lanelet_id_list) - 1:
                # shift speed limit lanes
                print(lanes_l, anchor_l, anchor_r)
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
                print(last_limits_new)
                last_limits = last_limits_new

                # todo calculate target_lanes[], target_lanes_index_distance (since last lane change), target_lanes_left_shift (for this instance)

        msg.distance = self.route.path_length[-1]
        msg.target = self.target

        return msg

# Point2D[] points
# float32[] speed_limits
# PafTrafficSignal[] signals

# int32[] target_lanes
# int32 target_lanes_index_distance
# int32 target_lanes_left_shift
