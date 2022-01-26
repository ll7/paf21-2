#!/usr/bin/env python
import time
from collections import deque

from commonroad_route_planner.route import Route

import rospy
import numpy as np

from typing import List, Tuple
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Circle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_route_planner.route_planner import RoutePlanner

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
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
from std_msgs.msg import Empty, Bool
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
        self._waypoints = deque()
        self._lanelet_ids_route = []
        self._last_route = None
        self._standard_loop = MapManager.get_demo_route()
        self._rerouting_target = None

        rospy.init_node("paf_global_planner", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        rospy.Subscriber("/paf/paf_local_planner/routing_request", PafRoutingRequest, self._routing_provider_single)
        rospy.Subscriber(
            "/paf/paf_local_planner/routing_request_waypoints", PafLocalPath, self._routing_provider_waypoints()
        )
        rospy.Subscriber("/paf/paf_local_planner/routing_request_random", Empty, self._routing_provider_random)
        rospy.Subscriber(
            "/paf/paf_local_planner/routing_request_standard_loop", Empty, self._routing_provider_standard_loop
        )
        rospy.Subscriber("/paf/paf_starter/teleport", Pose, self._teleport)
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_provider)
        rospy.Subscriber("/paf/paf_local_planner/reroute", Empty, self._reroute_provider)
        rospy.Subscriber("/paf/paf_local_planner/rules_enabled", Bool, self._change_rules, queue_size=1)

        self._routing_pub = rospy.Publisher("/paf/paf_global_planner/routing_response", PafLaneletRoute, queue_size=1)
        self._teleport_pub = rospy.Publisher(f"/carla/{role_name}/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self._target_on_map_pub = rospy.Publisher(
            "/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1
        )
        self._start_score_pub = rospy.Publisher("/paf/paf_validation/score/start", Empty, queue_size=1)

    def _change_rules(self, msg: Bool):
        rospy.set_param("rules_enabled", msg.data)
        self._scenario = MapManager.get_current_scenario()
        SpeedCalculator.set_limits(msg.data)
        rospy.logwarn(
            f"[global planner] Rules are now {'en' if msg.data else 'dis'}abled! "
            f"Speed limits will change after starting a new route."
        )

    def _reroute_provider(self, _: Empty = None):
        rospy.loginfo("[global planner] rerouting...")
        self._routing_provider_waypoints(reroute=True)

    def _any_target_anywhere(self, p_home) -> np.ndarray:
        # return np.array([229., -100.])
        lanelets = self._scenario.lanelet_network.lanelets
        lanelet_p = None
        counter = 0
        min_dist = 100
        while counter < 100 and lanelet_p is None or dist(p_home, lanelet_p) < min_dist:
            counter += 1
            lanelet = np.random.choice(lanelets)
            lanelet_p = np.random.choice(range(len(lanelet.center_vertices)))
            lanelet_p = lanelet.center_vertices[lanelet_p]
        if lanelet_p is None:
            return None
        return lanelet_p

    def _find_closest_position_on_lanelet_network(self) -> Tuple[np.ndarray, float]:
        lanelet_id = find_closest_lanelet(
            self._scenario.lanelet_network, Point2D(self._position[0], self._position[1])
        )[0]
        lanelet = self._scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        idx = np.argmin([dist(a, self._position) for a in lanelet.center_vertices])
        if idx == len(lanelet.center_vertices) - 1:
            idx -= 1
        position = lanelet.center_vertices[idx]
        return position, self._yaw

    def _routing_provider_standard_loop(self, _: Empty):
        t0 = time.perf_counter()
        waypoints, initial_pose = self._standard_loop
        if waypoints is None:
            self._routing_provider_random()
            return

        position, yaw = (initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y), 0

        msg = None
        if len(self._waypoints) == 0:
            msg = PafLocalPath()
            msg.points = list(waypoints)
            rospy.logwarn("[global planner] restarting standard loop")
            rospy.Publisher("/carla/ego_vehicle/twist", Twist, queue_size=1).publish(Twist())
            rospy.Publisher("/carla/ego_vehicle/initialpose", PoseWithCovarianceStamped, queue_size=1).publish(
                initial_pose
            )
            rospy.sleep(1)
        success = self._routing_provider_waypoints(msg, position, yaw)
        t0 = np.round(time.perf_counter() - t0, 2)
        if success:
            rospy.loginfo_throttle(10, f"[global planner] success planning standard loop ({t0}s)")
        else:
            rospy.logerr_throttle(10, f"[global planner] failed planning standard loop ({t0}s)")

    def _routing_provider_random(self, _: Empty = None):
        t0 = time.perf_counter()
        self._last_route = None
        try:
            position, yaw = self._find_closest_position_on_lanelet_network()
        except IndexError:
            rospy.logerr_throttle(1, "[global planner] unable to find current lanelet")
            return

        targets = [position]
        for i in range(1):
            target = self._any_target_anywhere(targets[-1])
            if target is not None:
                targets.append(target)
        msg = PafLocalPath()
        msg.points = [Point2D(t[0], t[1]) for t in targets[1:]]
        self._routing_provider_waypoints(msg, position, yaw)

        t0 = np.round(time.perf_counter() - t0, 2)
        rospy.loginfo_throttle(10, f"[global planner] success ({t0}s)")

    def _routing_provider_single(self, msg: PafRoutingRequest = None, position=None, yaw=None):
        msgs = PafLocalPath()
        msgs.points = [Point2D(msg.target[0], msg.target[1])]
        self._routing_provider_waypoints(msgs, position, yaw)

    def _routing_provider_waypoints(self, msgs: PafLocalPath = None, position=None, yaw=None, reroute=False):
        if msgs is None:
            msgs = PafLocalPath()
            msgs.points = list(self._waypoints)
        else:
            self._waypoints.clear()
            for p in msgs.points:
                self._waypoints.append(p)

        if position is None or yaw is None:
            try:
                position, yaw = self._find_closest_position_on_lanelet_network()
            except IndexError:
                rospy.logerr_throttle(1, "[global planner] unable to find current lanelet")
                return False
        if len(self._waypoints) == 0:
            if reroute:
                rospy.logwarn_throttle(1, "[global planner] waypoints are empty, rerouting to random location...")
                return self._routing_provider_random()
            else:
                rospy.logwarn_throttle(1, "[global planner] route planning waiting for route input...")
                return False
        rospy.logwarn_throttle(
            1, f"[global planner] waypoints in queue: " f"{[(int(p.x), int(p.y)) for p in self._waypoints]}"
        )
        draw_msg = PafTopDownViewPointSet()
        draw_msg.label = "planning_target"
        draw_msg.color = 153, 0, 153

        self._routing_pub.publish(PafLaneletRoute())

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
            return False

        lanelet_ids = route.list_ids_lanelets
        route_paf = GlobalPath(self._scenario.lanelet_network, lanelet_ids, target).as_msg()
        self._lanelet_ids_route = lanelet_ids
        self._last_route = route_paf
        self._routing_pub.publish(route_paf)
        self._target_on_map_pub.publish(draw_msg)

        self._start_score_pub.publish(Empty())
        return True

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
    def _choose_route(routes: List[Route]):
        if len(routes) == 0:
            return None
        idx = np.argmin([x.path_length[-1] for x in routes])
        route = routes[idx]
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

    def _route_from_ids(self, lanelet_ids: List[int]):
        return GlobalPath(
            self._scenario.lanelet_network,
            lanelet_ids,
            self._scenario.lanelet_network.find_lanelet_by_id(lanelet_ids[-1]).center_vertices[-1],
        )

    @staticmethod
    def start():
        rospy.spin()


if __name__ == "__main__":
    node = GlobalPlanner()
    node.start()
