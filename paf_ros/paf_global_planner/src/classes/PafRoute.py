import numpy as np
from typing import List, Tuple

from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.traffic_sign import (
    SupportedTrafficSignCountry as Country,
    TrafficSign,
    TrafficLight,
    TrafficLightState,
)
from commonroad.scenario.traffic_sign_interpreter import TrafficSigInterpreter
from commonroad_route_planner.route import Route as CommonroadRoute

from paf_messages.msg import PafLaneletRoute, PafRoutingGraphNode, Point2D, PafTrafficSignal


class PafRoute:
    def __init__(self, route: CommonroadRoute, traffic_sign_country: Country):
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

        out += f"Route completed at {route_ids[-1]}"
        return out

    @staticmethod
    def _locate_obj_on_lanelet(route_pts: List[List[float]], object_position: List[float]):
        def dist(a, b):
            x1, y1 = a
            x2, y2 = b
            return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        return np.argmin([dist(x, object_position) for x in route_pts])

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

    def _extract_traffic_signals(self, path_pts, route_lanelet_ids):
        traffic_signals = []
        relevant_traffic_signs = []
        relevant_traffic_lights = []
        for lanelet_id in route_lanelet_ids:
            lanelet = self.route.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            relevant_traffic_lights += list(lanelet.traffic_lights)
            relevant_traffic_signs += list(lanelet.traffic_signs)

        sign: TrafficSign
        for sign in self.route.scenario.lanelet_network.traffic_signs:
            if sign.traffic_sign_id not in relevant_traffic_signs:
                continue
            paf_sign = PafTrafficSignal()
            paf_sign.type = sign.traffic_sign_elements[0].traffic_sign_element_id.name
            try:
                paf_sign.value = sign.traffic_sign_elements[0].additional_values[0]
            except IndexError:
                paf_sign.value = -1
            paf_sign.index = self._locate_obj_on_lanelet(path_pts, list(sign.position))
            traffic_signals.append(paf_sign)

        light: TrafficLight
        for light in self.route.scenario.lanelet_network.traffic_lights:
            if light.traffic_light_id not in relevant_traffic_lights:
                continue
            paf_sign = PafTrafficSignal()
            paf_sign.type = "LIGHT"
            paf_sign.value = 0
            red_value = 0
            for phase in light.cycle:
                if phase.state == TrafficLightState.RED:
                    red_value += phase.duration
                else:
                    paf_sign.value += phase.duration
            try:
                paf_sign.value /= paf_sign.value + red_value
            except ZeroDivisionError:
                paf_sign.value = -1
            paf_sign.index = self._locate_obj_on_lanelet(path_pts, list(light.position))
            traffic_signals.append(paf_sign)

        return sorted(traffic_signals, key=lambda elem: elem.index)

    def as_msg(self, resolution=0):
        msg = PafLaneletRoute()
        msg.lanelet_ids = self.route.list_ids_lanelets
        msg.points = []
        if resolution == 0:
            every_nth = 1
        else:
            every_nth = int(np.round(len(self.route.path_length) / msg.length / resolution))
            every_nth = every_nth if every_nth != 0 else 1
        path_pts = self.route.reference_path[::every_nth]
        msg.distances = self.route.path_length[::every_nth]
        msg.traffic_signals = self._extract_traffic_signals(path_pts, msg.lanelet_ids)
        for x, y in path_pts:
            point = Point2D()
            point.x = x
            point.y = y
            msg.points.append(point)
        msg.graph = []
        for key, (l, s, r) in self.graph.items():
            node_msg = PafRoutingGraphNode()
            node_msg.start = key
            node_msg.left = -1 if l is None else l
            node_msg.straight = -1 if s is None else s
            node_msg.right = -1 if r is None else r
            msg.graph.append(node_msg)
        return msg
