#!/usr/bin/env python
from collections import deque

from commonroad_route_planner.route import Route

import rospy
import numpy as np

from typing import List, Tuple, Optional, Union, Any
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Circle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_route_planner.route_planner import RoutePlanner

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from paf_messages.msg import (
    PafLaneletRoute,
    PafRoutingRequest,
    PafTopDownViewPointSet,
    Point2D,
    PafLocalPath,
)
from classes.HelperFunctions import dist, find_closest_lanelet, find_lanelet_yaw
from classes.GlobalPath import GlobalPath
from classes.MapManager import MapManager
from classes.SpeedCalculator import SpeedCalculator
from paf_messages.srv import PafRoutingService, PafRoutingServiceResponse
from std_msgs.msg import Bool, Empty as Empty_msg
from std_srvs.srv import Empty as Empty_srv
from tf.transformations import euler_from_quaternion


class GlobalPlanner:
    """Planning Node, calculates CR-Routes. GlobalPath used to convert CR-Route to paf-message"""

    # allowed backends:
    # NETWORKX_REVERSED (earlier lane change)
    # NETWORKX (later lane change)
    # PRIORITY_QUEUE (heuristic cost a-star-algorithm)
    BACKEND = RoutePlanner.Backend.NETWORKX_REVERSED

    def __init__(self):
        """
        Setup node and init all subscribers, services and publishers needed.
        """
        self._scenario: Scenario = MapManager.get_current_scenario()
        self._position = [1e99, 1e99]
        self._yaw = 0
        self._waypoints = deque()
        self._standard_loop = MapManager.get_demo_route()
        self._rerouting_target = None
        self._last_target = None

        rospy.init_node("paf_global_planner", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        rospy.Subscriber("/paf/paf_local_planner/routing_request", PafRoutingRequest, self._routing_provider_single)
        rospy.Subscriber("/paf/paf_local_planner/reroute", Empty_msg, self._process_rerouting)
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_provider)
        rospy.Subscriber("/paf/paf_local_planner/rules_enabled", Bool, self._change_rules, queue_size=1)

        rospy.Service("/paf/paf_local_planner/reroute", PafRoutingService, self._reroute_provider)
        rospy.Service(
            "/paf/paf_local_planner/routing_request_standard_loop",
            PafRoutingService,
            self._routing_provider_standard_loop,
        )
        rospy.Service("/paf/paf_local_planner/routing_request_random", PafRoutingService, self._routing_provider_random)

        self._routing_pub = rospy.Publisher("/paf/paf_global_planner/routing_response", PafLaneletRoute, queue_size=1)
        self._tdv_routing_pub = rospy.Publisher(
            "/paf/paf_global_planner/routing_response_tdv", PafLaneletRoute, queue_size=1
        )
        self._target_on_map_pub = rospy.Publisher(
            "/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1
        )
        self._start_score_pub = rospy.Publisher("/paf/paf_validation/score/start", Empty_msg, queue_size=1)

    def _change_rules(self, msg: Bool):
        """
        Change rules (driving mode) during runtime. Updates Speed Calculation rules and triggers rerouting.
        :param msg: Bool message to set rules_enabled to
        """
        rospy.set_param("rules_enabled", msg.data)
        self._scenario = MapManager.get_current_scenario()
        SpeedCalculator.set_limits(msg.data)
        rospy.logwarn(
            f"[global planner] Rules are now {'en' if msg.data else 'dis'}abled! "
            f"Speed limits will change after starting a new route."
        )

    def _process_rerouting(self, _: Empty_msg):
        self._routing_pub.publish(self._routing_provider_waypoints(reroute=True))

    def _reroute_provider(self, _: Empty_srv = None) -> PafRoutingServiceResponse:
        """
        Response to reroute service call
        :param _: empty msg
        :return: Paf Route
        """
        rospy.logwarn("[global planner] rerouting...")
        return self._routing_provider_waypoints(reroute=True)

    def _any_target_anywhere(self, p_home: Union[Point2D, Any], min_dist: float = 100) -> Optional[np.ndarray]:
        """
        Select a random target at least min_dist meters away from p_home. None if unsuccessful
        :param p_home: current position
        :param min_dist: minimum distance for random target
        :return:
        """
        lanelets = self._scenario.lanelet_network.lanelets
        lanelet_p = None
        counter = 0
        while counter < 100 and lanelet_p is None or dist(p_home, lanelet_p) < min_dist:
            counter += 1
            lanelet = np.random.choice(lanelets)
            lanelet_p = np.random.choice(range(len(lanelet.center_vertices)))
            lanelet_p = lanelet.center_vertices[lanelet_p]
        if lanelet_p is None:
            return None
        return lanelet_p

    def _find_closest_position_on_lanelet_network(self) -> Tuple[np.ndarray, float]:
        """
        Retrieve the closest lanelet_point on the closest lanelet relative to the current position and yaw angle
        :return: (lanelet_point,yaw)
        """
        pos = self._position[0], self._position[1]

        lanelets = find_closest_lanelet(self._scenario.lanelet_network, Point2D(*pos))

        angle_offsets = [
            np.abs(find_lanelet_yaw(self._scenario.lanelet_network, Point2D(*pos), let) - self._yaw) for let in lanelets
        ]
        lanelet_id = lanelets[np.argmin(angle_offsets)]
        lanelet = self._scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        idx = np.argmin([dist(a, pos) for a in lanelet.center_vertices])
        if idx == len(lanelet.center_vertices) - 1:
            idx -= 1
        position = lanelet.center_vertices[idx]
        return position, self._yaw

    def _routing_provider_standard_loop(self, _: Empty_srv) -> PafRoutingServiceResponse:
        """
        Get route to next waypoint in stack
        :param _: empty
        :return: routing response
        """
        initial_pose, waypoints = self._standard_loop
        if waypoints is None:
            return self._routing_provider_random()

        msg = None
        if len(self._waypoints) == 0:
            msg = PafLocalPath()
            msg.points = list(waypoints)
            rospy.logwarn("[global planner] restarting standard loop")
            rospy.Publisher("/carla/ego_vehicle/twist", Twist, queue_size=1).publish(Twist())
            rospy.Publisher("/carla/ego_vehicle/initialpose", PoseWithCovarianceStamped, queue_size=1).publish(
                initial_pose
            )
            rospy.sleep(3)
        return self._routing_provider_waypoints(msg)

    def _routing_provider_random(self, _: Empty_srv = None) -> PafRoutingServiceResponse:
        """
        Get route to random point on any lanelet
        :param _: empty
        :return: routing response
        """
        try:
            position, yaw = self._find_closest_position_on_lanelet_network()
        except IndexError:
            rospy.logerr_throttle(1, "[global planner] unable to find current lanelet")
            return PafRoutingServiceResponse()

        targets = [position]
        for i in range(1):
            target = self._any_target_anywhere(targets[-1])
            if target is not None:
                targets.append(target)
        msg = PafLocalPath()
        msg.points = [Point2D(t[0], t[1]) for t in targets[1:]]

        return self._routing_provider_waypoints(msg, position, yaw)

    def _routing_provider_single(self, msg: PafRoutingRequest = None, position=None, yaw=None):
        """
        Publish a route based on a routing request message call (from competition manager)
        :param msg: routing request message
        :param position: from position
        :param yaw: from yaw
        """
        msgs = PafLocalPath()
        msgs.points = [Point2D(msg.target[0], msg.target[1])]
        self._routing_pub.publish(self._routing_provider_waypoints(msgs, position, yaw))

    def _routing_provider_waypoints(
        self, msgs: PafLocalPath = None, position: Union[Point2D, Any] = None, yaw: float = None, reroute: bool = False
    ) -> Union[PafLaneletRoute, PafRoutingServiceResponse]:
        """
        Routing based on a list of waypoints
        :param msgs: list of waypoints points packed in local path message
        :param position: from position
        :param yaw: from yaw
        :param reroute: is a reroute request (true/false)
        :return: global path routing message
        """

        def failure():
            rospy.logerr_throttle(1, "[global planner] routing failed")
            return PafRoutingServiceResponse()

        if msgs is None:
            msgs = PafLocalPath()
            msgs.points = list(self._waypoints)
        else:
            self._waypoints.clear()
            for p in msgs.points:
                self._waypoints.append(p)

        if len(self._waypoints) >= 1:
            self._last_target = self._waypoints[0]
        elif reroute and self._last_target is not None:
            self._waypoints.append(self._last_target)  # fixed issue: target is on current lanelet while routing.

        if position is None or yaw is None:
            try:
                position, yaw = self._find_closest_position_on_lanelet_network()
            except Exception:
                rospy.logerr_throttle(1, "[global planner] unable to find current lanelet")
                return failure()
        if len(self._waypoints) == 0:
            if reroute:
                if not rospy.get_param("/validation"):
                    rospy.loginfo_throttle(
                        1,
                        "[global planner] waypoints are empty, no reroute possible, " "waiting for routing request...",
                    )
                    return failure()
                else:
                    rospy.logwarn_throttle(1, "[global planner] waypoints are empty, rerouting to random location...")
                    return self._routing_provider_random()
            else:
                rospy.loginfo_throttle(1, "[global planner] route planning waiting for route input...")
                return failure()
        rospy.loginfo_throttle(
            1, f"[global planner] waypoints in queue: " f"{[(int(p.x), int(p.y)) for p in self._waypoints]}"
        )
        draw_msg = PafTopDownViewPointSet()
        draw_msg.label = "planning_target"
        draw_msg.color = 153, 0, 153

        if reroute:
            target = self._rerouting_target
        else:
            target = self._waypoints.popleft()
            self._rerouting_target = target
        yaw = find_lanelet_yaw(self._scenario.lanelet_network, target)
        route: Route = self._route_from_objective(position, yaw, [target.x, target.y])
        draw_msg.points.append(target)
        if route is None:
            rospy.logerr(
                f"[global planner] routing from {list(position)} to {[target.x, target.y]} failed",
            )
            self._waypoints.appendleft(target)
            return failure()

        lanelet_ids = route.list_ids_lanelets
        route_paf = GlobalPath(self._scenario.lanelet_network, lanelet_ids, target).as_msg()
        self._target_on_map_pub.publish(draw_msg)
        self._start_score_pub.publish(Empty_msg())
        self._tdv_routing_pub.publish(route_paf)
        return route_paf

    def _odometry_provider(self, odometry: Odometry):
        """
        odometry subscriber callback
        """
        pose = odometry.pose.pose
        self._position = pose.position.x, pose.position.y
        _, _, self._yaw = euler_from_quaternion(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        )
        if self._yaw < 0:
            self._yaw += np.pi * 2

    def _route_from_objective(
        self,
        start_coordinates: np.ndarray,
        start_orientation_rad: float,
        target_coordinates: List[float],
        target_orientation_rad: float = None,
        start_velocity: float = 0.0,
        target_circle_diameter: float = 4.0,
        target_orientation_allowed_error: float = 0.2,
    ) -> Route:
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
        :return: selected route
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
        route_planner = RoutePlanner(self._scenario, planning_problem, backend=self.BACKEND)
        routes, _ = route_planner.plan_routes().retrieve_all_routes()

        return self._choose_route(routes)

    @staticmethod
    def _choose_route(routes: List[Route]) -> Optional[Route]:
        """
        Select tha route based on a list of CR-routes
        :param routes:
        :return: selected route
        """
        if len(routes) == 0:
            return None
        lengths = [x.path_length[-1] for x in routes]
        idx = np.argmin(lengths)
        route = routes[idx]
        rospy.logwarn(f"[global planner] found {len(routes)} routes with lengths {[round(x) for x in lengths]}")
        return route

    @staticmethod
    def _get_planning_problem(
        start_coordinates: np.ndarray,
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

    @staticmethod
    def start():
        """
        Start ros node
        """
        rospy.spin()


if __name__ == "__main__":
    node = GlobalPlanner()
    node.start()
