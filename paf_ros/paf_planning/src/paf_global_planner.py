#!/usr/bin/env python
from commonroad_route_planner.route import Route, RouteType

import rospy
import numpy as np

from typing import List
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Circle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_route_planner.route_planner import RoutePlanner

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLaneletRoute, PafRoutingRequest, PafTopDownViewPointSet, Point2D

from classes.PafRoute import PafRoute
from classes.MapManager import MapManager
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion


class GlobalPlanner:
    # NETWORKX_REVERSED (earlier lane change)
    # NETWORKX (later lane change)
    # PRIORITY_QUEUE (heuristic cost a-star-algorithm)
    BACKEND = RoutePlanner.Backend.NETWORKX_REVERSED
    UPDATE_HZ = 1

    def __init__(self):
        self._scenario: Scenario = MapManager.get_current_scenario()
        self._position = [1e99, 1e99]
        self._yaw = 0
        self._routing_target = None
        self._last_route = None

        rospy.init_node("paf_global_planner", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        rospy.Subscriber("/paf/paf_local_planner/routing_request", PafRoutingRequest, self._routing_provider)
        rospy.Subscriber("/paf/paf_local_planner/routing_request_random", Empty, self._routing_provider_random)
        rospy.Subscriber("/paf/paf_starter/teleport", Pose, self._teleport)
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_provider)

        rospy.Subscriber("/paf/paf_local_planner/reroute", Empty, self._reroute_provider)

        self._routing_pub = rospy.Publisher("/paf/paf_global_planner/routing_response", PafLaneletRoute, queue_size=1)
        self._teleport_pub = rospy.Publisher(f"/carla/{role_name}/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self._target_on_map_pub = rospy.Publisher(
            "/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1
        )

    def _reroute_provider(self, _: Empty = None):
        rospy.loginfo("[global planner] rerouting...")
        self._routing_provider()

    @staticmethod
    def dist(a, b):
        x1, y1 = a
        x2, y2 = b
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _any_target_anywhere(self, p_home):
        lanelets = self._scenario.lanelet_network.lanelets
        lanelet_p = None
        counter = 0
        min_dist = 100
        while counter < 100 and lanelet_p is None or self.dist(p_home, lanelet_p) < min_dist:
            counter += 1
            lanelet = np.random.choice(lanelets)
            lanelet_p = np.random.choice(range(len(lanelet.center_vertices)))
            lanelet_p = lanelet.center_vertices[lanelet_p]
        if lanelet_p is None:
            return None
        return lanelet_p

    def _find_closest_position_on_lanelet_network(self) -> np.ndarray:
        lanelet_id = self._find_closest_lanelet()[0]
        lanelet = self._scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        position = lanelet.center_vertices[np.argmin([self.dist(a, self._position) for a in lanelet.center_vertices])]
        return position

    def _routing_provider_random(self, _: Empty):
        msg = PafRoutingRequest()
        try:
            position = self._find_closest_position_on_lanelet_network()
        except IndexError:
            rospy.logerr_throttle(1, "[global planner] unable to find current lanelet")
            return
        msg.target = self._any_target_anywhere(position)
        self._routing_provider(msg)

    def _routing_provider(self, msg: PafRoutingRequest = None):
        if msg is None:
            msg = self._routing_target
        else:
            self._routing_target = msg

        try:
            position = self._find_closest_position_on_lanelet_network()
        except IndexError:
            rospy.logerr_throttle(1, "[global planner] unable to find current lanelet")
            return

        resolution = msg.resolution if msg is not None else 0
        route = None
        if msg is None:
            rospy.logwarn_throttle(1, "[global planner] route planning failed, trying to find any route for now...")
            target = self._any_target_anywhere(position)
            if target is None:
                rospy.logerr_throttle(1, "[global planner] unable to create random target")
                return
        else:
            target = msg.target

        routes = self._routes_from_objective(position, self._yaw, target, return_shortest_only=True)
        if len(routes) > 0:
            rospy.loginfo_throttle(1, f"[global planner] publishing route to target {target}")
            route = routes[0].as_msg(resolution)
        elif len(routes) == 0:
            rospy.logerr_throttle(1, f"[global planner] unable to route to target {target}")
            return

        self._last_route = route
        self._routing_pub.publish(self._last_route)

        draw_msg = PafTopDownViewPointSet()
        draw_msg.label = "planning_target"
        draw_msg.points = [Point2D(target[0], target[1])]
        draw_msg.color = 153, 0, 153
        # self._target_on_map_pub.publish(draw_msg)

    def _find_closest_lanelet(self, p=None):
        if p is None:
            p = self._position
        p = np.array(p, dtype=float)
        lanelets = self._scenario.lanelet_network.find_lanelet_by_position([p])[0]
        if len(lanelets) > 0:
            return lanelets
        for radius in range(3, 100, 3):
            shape = Circle(radius=radius, center=p)
            lanelets = self._scenario.lanelet_network.find_lanelet_by_shape(shape)
            if len(lanelets) > 0:
                break
        return lanelets

    def _teleport(self, msg: Pose):
        msg_out = PoseWithCovarianceStamped()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.pose = msg
        self._teleport_pub.publish(msg_out)

    def _odometry_provider(self, odometry: Odometry):
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

    def _routes_from_objective(
        self,
        start_coordinates: np.ndarray,
        start_orientation_rad: float,
        target_coordinates: List[float],
        target_orientation_rad: float = None,
        start_velocity: float = 0.0,
        target_circle_diameter: float = 4.0,
        target_orientation_allowed_error: float = 0.2,
        return_shortest_only=False,
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
        route_planner = RoutePlanner(self._scenario, planning_problem, backend=self.BACKEND)
        routes, _ = route_planner.plan_routes().retrieve_all_routes()
        if return_shortest_only:
            if len(routes) == 0:
                return []
            idx = np.argmin([x.path_length[-1] for x in routes])
            route = routes[idx]
            return [PafRoute(route)]
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
        return PafRoute(Route(self._scenario, None, lanelet_ids, RouteType.REGULAR))

    def start(self):
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            if self._last_route is None:
                self._reroute_provider()
            else:
                self._routing_pub.publish(self._last_route)
            rate.sleep()


if __name__ == "__main__":
    node = GlobalPlanner()
    node.start()
