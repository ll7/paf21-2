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

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLaneletRoute, PafRoutingRequest

from classes.PafRoute import PafRoute
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion


class GlobalPlanner:
    # NETWORKX_REVERSED (earlier lane change)
    # NETWORKX (later lane change)
    # PRIORITY_QUEUE (heuristic cost a-star-algorithm)
    BACKEND = RoutePlanner.Backend.NETWORKX_REVERSED

    def __init__(self):
        self.scenario: Scenario
        self.scenario, _ = CommonRoadFileReader("/home/julin/Downloads/Town03.xml").open()
        self._position = [1e99, 1e99]
        self._yaw = 0
        self._routing_target = None

        rospy.init_node("paf_global_planner", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        rospy.Subscriber("/paf/paf_local_planner/routing_request", PafRoutingRequest, self._routing_provider)
        rospy.Subscriber("/paf/paf_starter/teleport", Pose, self._teleport)
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_provider)

        rospy.Service("/paf/paf_local_planner/reroute", Empty, self._reroute_provider)

        self._routing_pub = rospy.Publisher("/paf/paf_global_planner/routing_response", PafLaneletRoute, queue_size=1)
        self._teleport_pub = rospy.Publisher(f"/carla/{role_name}/initialpose", PoseWithCovarianceStamped, queue_size=1)

    def _reroute_provider(self, _: Empty):
        rospy.loginfo("[global planner] rerouting...")
        self._routing_provider()

    def _routing_provider(self, msg: PafRoutingRequest = None):
        if msg is None:
            msg = self._routing_target
        else:
            self._routing_target = msg

        if msg is not None:
            routes = self._routes_from_objective(self._position, self._yaw, msg.target, return_shortest_only=True)
            if len(routes) > 0:
                rospy.loginfo_throttle(1, f"[global planner] publishing route to target {msg.target}")
                self._routing_pub.publish(routes[0].as_msg(msg.resolution))
                return
        resolution = msg.resolution if msg is not None else 0
        rospy.logwarn_throttle(
            1, "[global planner] route planning failed, " "trying to find any straight route for now..."
        )
        try:
            lanelet_id = self._find_closest_lanelet()[0]
        except IndexError:
            rospy.logerr_throttle(1, "[global planner] unable to find current lanelet")
            return
        ids = [lanelet_id]
        for i in range(10):
            lanelet = self.scenario.lanelet_network.find_lanelet_by_id(ids[-1])
            choices = lanelet.successor
            if len(choices) == 0:
                rospy.logerr_throttle(1, "[global planner] unable to find successor lanelet")
                return
            ids.append(np.random.choice(choices))
        rospy.loginfo_throttle(1, "[global planner] publishing route to a target straight ahead")
        self._routing_pub.publish(self._route_from_ids(ids).as_msg(resolution))

    def _find_closest_lanelet(self, p=None):
        if p is None:
            p = self._position
        p = np.array(p, dtype=float)
        lanelets = self.scenario.lanelet_network.find_lanelet_by_position([p])[0]
        if len(lanelets) > 0:
            return lanelets
        shape = Circle(radius=5, center=p)
        lanelets = self.scenario.lanelet_network.find_lanelet_by_shape(shape)
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
        start_coordinates: List[float],
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
        route_planner = RoutePlanner(self.scenario, planning_problem, backend=self.BACKEND)
        routes, _ = route_planner.plan_routes().retrieve_all_routes()
        if return_shortest_only:
            routes = sorted(routes, key=lambda x: x.reference_path[-1])
            return [PafRoute(routes[0])]
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

# rosservice call /paf/paf_global_planner/routing_request "start:
# - -85.0
# - -75.0
# start_yaw: 1.56
# target:
# - -180.0
# - 180.0
# resolution: 0.0"
