#!/usr/bin/env python
from commonroad_route_planner.route import Route, RouteType

import rospy
import numpy as np

from typing import List
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Circle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_route_planner.route_planner import RoutePlanner

from paf_messages.msg import PafLaneletRoute
from paf_messages.srv import PafRoutingRequest

from classes.PafRoute import PafRoute


class GlobalPlanner:
    # NETWORKX_REVERSED (earlier lane change)
    # NETWORKX (later lane change)
    # PRIORITY_QUEUE (heuristic cost a-star-algorithm)
    BACKEND = RoutePlanner.Backend.NETWORKX_REVERSED

    def __init__(self):
        self.scenario: Scenario
        self.scenario, _ = CommonRoadFileReader("/home/julin/Downloads/Town03.xml").open()

        rospy.init_node("paf_global_planner", anonymous=True)
        rospy.Service("/paf_global_planner/routing_request", PafRoutingRequest, self._routing_provider)

    def _routing_provider(self, request: PafRoutingRequest) -> List[List[PafLaneletRoute]]:
        routes = self._routes_from_objective(request.start, request.start_yaw, request.target)
        return [[route.as_msg(request.resolution) for route in routes]]

    def _routes_from_objective(
        self,
        start_coordinates: List[float],
        start_orientation_rad: float,
        target_coordinates: List[float],
        target_orientation_rad: float = None,
        start_velocity: float = 0.0,
        target_circle_diameter: float = 4.0,
        target_orientation_allowed_error: float = 0.2,
    ) -> List[PafRoute]:
        """
        Creates a commonroad planning problem with the given parameters
        and calculates possible routes to the target region.
        :param start_coordinates: start coordinates [x,y]
        :param start_orientation_rad: start orientation in radians (yaw)
        :param target_coordinates: target coordinates [x,y]
        :param target_orientation_rad: target orientation in radians (yaw). Standard: None for any direction
        :param start_velocity: start velocity in m/s. Standard: 0
        :param target_circle_diameter: size of the target region (circle diameter)
        :param target_orientation_allowed_error: if target_orientation_rad is not None,
                                                    specify allowed margin of error here
        :return: list of Routes
        """
        planning_problem = self._get_planning_problem(
            start_coordinates,
            start_orientation_rad,
            target_coordinates,
            target_orientation_rad,
            start_velocity,
            target_circle_diameter,
            target_orientation_allowed_error,
        )
        route_planner = RoutePlanner(self.scenario, planning_problem, backend=self.BACKEND)

        routes, _ = route_planner.plan_routes().retrieve_all_routes()
        return [PafRoute(route) for route in routes]

    @staticmethod
    def _get_planning_problem(
        start_coordinates: List[float],
        start_orientation_rad: float,
        target_coordinates: List[float],
        target_orientation_rad: float = None,
        start_velocity: float = 0.0,
        target_circle_diameter: float = 4.0,
        target_orientation_allowed_error: float = 0.2,
    ) -> PlanningProblem:
        """
        Creates a commonroad planning problem with the given parameters
        :param start_coordinates: start coordinates [x,y]
        :param start_orientation_rad: start orientation in radians (yaw)
        :param target_coordinates: target coordinates [x,y]
        :param target_orientation_rad: target orientation in radians (yaw). Standard: None for any direction
        :param start_velocity: start velocity in m/s. Standard: 0
        :param target_circle_diameter: size of the target region (circle diameter)
        :param target_orientation_allowed_error: if target_orientation_rad is not None,
                                                    specify allowed margin of error here
        :return planning problem
        """
        assert len(start_coordinates) == 2
        assert len(target_coordinates) == 2

        if target_orientation_rad is None:
            target_orientation_rad = 0
            target_orientation_allowed_error = np.pi - 0.01

        initial_state = State(
            position=np.array(start_coordinates, dtype=float),
            velocity=float(start_velocity),
            orientation=float(start_orientation_rad),
            yaw_rate=Interval(0, 0.01),
            slip_angle=AngleInterval(0, 0.01),
            time_step=0,
        )

        target_state = State(
            position=Circle(radius=target_circle_diameter / 2, center=np.array(target_coordinates, dtype=float)),
            velocity=Interval(-180.0, 180.0),
            orientation=AngleInterval(
                target_orientation_rad - target_orientation_allowed_error,
                target_orientation_rad + target_orientation_allowed_error,
            ),
            time_step=Interval(0, int(1e10)),
        )

        return PlanningProblem(1, initial_state, GoalRegion([target_state]))

    def _route_from_ids(self, lanelet_ids: List[int]):
        return PafRoute(Route(self.scenario, None, lanelet_ids, RouteType.REGULAR))

    @staticmethod
    def start():
        rospy.spin()


if __name__ == "__main__":
    node = GlobalPlanner()
    node.start()

# rosservice call /paf_global_planner/routing_request "start:
# - -85.0
# - -75.0
# start_yaw: 1.56
# target:
# - -180.0
# - 180.0
# resolution: 0.0"
