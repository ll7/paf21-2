#!/usr/bin/env python
from typing import List, Tuple, Union, Optional, Any
from commonroad.geometry.shape import Circle
from commonroad.scenario.lanelet import Lanelet

import rospy
from classes.MapManager import MapManager
from nav_msgs.msg import Odometry
from paf_messages.msg import (
    PafObstacleList,
    PafObstacle,
    PafTopDownViewPointSet,
    Point2D,
    PafObstacleFollowInfo,
    PafLocalPath,
)
from classes.HelperFunctions import (
    closest_index_of_point_list,
    xy_to_pts,
    get_angle_between_vectors,
    dist,
    k_closest_indices_of_point_in_list,
)
import numpy as np

from paf_messages.srv import (
    PafLaneInfoService,
    PafLaneInfoServiceRequest,
    PafLaneInfoServiceResponse,
)
from tf.transformations import euler_from_quaternion


class ObstaclePlanner:
    """handling of path prediction of obstacles, calculates and publishes follow information for acting"""

    rules_enabled = MapManager.get_rules_enabled()
    network = MapManager.get_current_scenario().lanelet_network
    ON_LANELET_DISTANCE = 2
    ON_LANELET_DISTANCE_PEDESTRIANS = 1.5
    SHOW_TRACE_PTS = True
    SHOW_FOLLOW_PTS = True
    SAFETY_DISTANCE = 3

    def __init__(self):
        """
        Create Node, Subscriber and Publisher
        """
        rospy.init_node("obstacle_planner_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        rospy.logwarn(f"[obstacle planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")
        rospy.Subscriber("/paf/paf_perception/obstacles", PafObstacleList, self._process_obstacles, queue_size=1)
        rospy.Subscriber("/paf/paf_local_planner/path", PafLocalPath, self._process_local_path, queue_size=1)
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        self._follow_info_pub = rospy.Publisher(
            "/paf/paf_obstacle_planner/following_info", PafObstacleFollowInfo, queue_size=1
        )

        rospy.Service("/paf/paf_obstacle_planner/lane_info_service", PafLaneInfoService, self._lane_info_service_called)

        self.follow_trace_points = True  # self.rules_enabled

        self.vehicle_traces = []
        self.biker_and_peds_on_road_traces = []
        self.closest_follow_front = self.get_follow_info(), self.get_follow_info()
        self._closest_follow_front_pts = [], []
        self._last_local_path = PafLocalPath()
        self._position = Point2D(0, 0)
        self._current_yaw = 0

    def _lane_info_service_called(self, msg: PafLaneInfoServiceRequest):
        """
        Callback for lane info service (local planner calls this service).
        Calculates a follow info object for the given path.
        :param msg: request
        :return: follow info for given path
        """
        acc = 10
        if dist(*msg.path_to_check.points[:2]) > 0.5:
            acc = 1
        info, _, _ = self._get_local_path_follow_info(
            msg.path_to_check, max_dist_to_lane=None, acc=acc, check_traces=True
        )
        resp = PafLaneInfoServiceResponse()
        resp.follow_info = info
        return resp

    def _process_obstacles(self, msg: PafObstacleList):
        """
        Callback for incoming obstacle lists. Calculates path prediction for incoming obstacles
        :param msg: message with list of obstacles and obstacle type
        """
        if msg.type == "Pedestrians":
            process_fun = self.process_pedestrian
            self.biker_and_peds_on_road_traces = []
        elif msg.type == "Vehicles":
            process_fun = self.process_vehicle
            self.vehicle_traces = []
        else:
            raise TypeError(f"unknown obstacle type: {msg.type}")

        close = self.get_follow_info()
        close_pts = []
        o: PafObstacle
        for o in msg.obstacles:
            process_fun(o)

            _close, _close_pts = self.process_very_close_obstacles(o, close, msg.type == "Vehicles")

            if _close is not None:
                close, close_pts = _close, _close_pts
        p, v = self.closest_follow_front
        _p, _v = self._closest_follow_front_pts
        if msg.type == "Pedestrians":
            self.closest_follow_front = close, v
            self._closest_follow_front_pts = close_pts, _v
        elif msg.type == "Vehicles":
            self.closest_follow_front = p, close
            self._closest_follow_front_pts = _p, close_pts

    def process_very_close_obstacles(
        self, msg: PafObstacle, close: PafObstacleFollowInfo, is_vehicle: bool
    ) -> Tuple[Optional[PafObstacleFollowInfo], Optional[List[Point2D]]]:
        """
        Test if obstacle is close relative to the ego vehicle and calculate a follow info if this applies
        :param msg: obstacle message
        :param close: follow info to modify (previous search)
        :param is_vehicle: vehicle or pedestrian switch
        :return: (followinfo, [list of points])
        """
        lim_angle_front = np.deg2rad(10)
        lim_angle_side = np.pi / 2
        lim_dist_front = 10
        lim_dist_sides = self.ON_LANELET_DISTANCE * 1
        pts = [msg.bound_1, msg.bound_2, msg.closest]
        angles = [self._angle_rel_to_ego_yaw(p) for p in pts]
        idx = np.argmin([np.abs(a) for a in angles])

        pt = pts[idx]
        angle = angles[idx]
        distance = dist(pt, self._position)

        is_sides = distance < lim_dist_sides and -lim_angle_side <= angle <= lim_angle_side
        is_front = distance < lim_dist_front and -lim_angle_front <= angle <= lim_angle_front
        if not (is_front or is_sides):
            return None, None

        if close.no_target or close.distance > distance:
            close = self.get_follow_info(msg, is_vehicle)
            rospy.loginfo_throttle(1, f"found close obstacle d={distance:.1f}, a={np.rad2deg(angle):.1f}")
            return close, [pt]

        return None, None

    def process_pedestrian(self, msg: PafObstacle):
        """
        Callback for any incoming pedestrian obstacle. Calculate path prediction and add to self
        :param msg: obstacle found
        """
        forward, backward, current_lanelet = self.trace_obstacle_with_lanelet_network(msg)
        if forward is None:  # not on lanelet network -> ignore
            return
        if self.pedestrian_on_lanelet(msg):
            self.biker_and_peds_on_road_traces.append((msg, forward, backward, current_lanelet))

    def pedestrian_on_lanelet(self, ped: PafObstacle) -> bool:
        """
        Test if a pedestrian is close to the local path (within ON_LANELET_DISTANCE_PEDESTRIANS)
        :param ped: pedestrian
        :return: truth
        """
        idx, distance = closest_index_of_point_list(self._last_local_path.points, ped.closest)
        if self.ON_LANELET_DISTANCE_PEDESTRIANS >= distance > 0:
            return True
        return False

    def process_vehicle(self, msg: PafObstacle):
        """
        Callback for any incoming vehicle. Calculate path prediction and add to self
        :param msg: obstacle found
        """
        forward, backward, current_lanelet = self.trace_obstacle_with_lanelet_network(msg)
        if forward is None:  # not on lanelet network -> ignore
            return
        self.vehicle_traces.append((msg, forward, backward, current_lanelet))

    def _get_local_path_follow_info(
        self, msg: PafLocalPath, max_dist_to_lane=None, acc=10, check_traces=True
    ) -> Tuple[PafObstacleFollowInfo, int, List[Point2D]]:
        """
        Give follow info information based on a local path message
        :param msg: local path message
        :param max_dist_to_lane: maximum distance to the local path
        :param acc: accuracy of index search on local path (higher is lower accuracy)
        :param check_traces: true if check path prediction points as well
        :return: (follow info object, index-position of obstacle on local path, points found for debugging)
        """
        vehicle_follow_info = self.get_follow_info()
        info_pts = []
        local_path_index, distance_to_lanelet_network = closest_index_of_point_list(msg.points, self._position)
        if max_dist_to_lane is None or distance_to_lanelet_network <= max_dist_to_lane:
            vehicle_follow_info, info_pts = self._calculate_follow_vehicle_with_path(msg.points, local_path_index, acc)
        if check_traces and self.follow_trace_points:
            ped_trace_points, veh_trace_points = self.get_relevant_trace_points(msg.points, local_path_index, acc)
            trace_follow_info, info2_pts = self._calculate_follow_info_with_trace_points(
                ped_trace_points, veh_trace_points, msg.points, local_path_index, acc
            )
            if trace_follow_info is not None and (
                trace_follow_info.distance < vehicle_follow_info.distance or vehicle_follow_info.no_target
            ):
                vehicle_follow_info = trace_follow_info
                info_pts = info2_pts

        return vehicle_follow_info, local_path_index, info_pts

    def _process_local_path(self, msg: PafLocalPath):
        """
        Callback function for received local path message (from local planner)
        :param msg: lp message
        """
        vehicle_follow_info, local_path_index, info_pts = self._get_local_path_follow_info(
            msg, max_dist_to_lane=2.5, acc=10
        )
        for follow_lp, pts in zip(self.closest_follow_front, self._closest_follow_front_pts):
            if (not follow_lp.no_target) and (
                vehicle_follow_info.no_target or follow_lp.distance < vehicle_follow_info.distance
            ):
                vehicle_follow_info = follow_lp
                info_pts = pts
        vehicle_follow_info.distance -= self.SAFETY_DISTANCE
        self._follow_info_pub.publish(vehicle_follow_info)

        rospy.loginfo_throttle(
            1,
            f"[obstacle planner] following={not vehicle_follow_info.no_target} v={vehicle_follow_info.speed} "
            f"d={vehicle_follow_info.distance:.1f}",
        )

        if self.SHOW_FOLLOW_PTS:
            self._draw_path_pts(xy_to_pts(info_pts), "relevant_obs_pts")

    def get_relevant_trace_points(
        self, path: List[Point2D], path_idx: int, acc: int
    ) -> Tuple[List[Point2D], List[Point2D]]:
        """
        Get trace points for vehicles and pedestrians (path prediction)
        :param path: given path
        :param path_idx: path index of ego vehicle
        :param acc: index search accuracy (higher is lower accuracy)
        :return: (ped_traces, veh_traces)
        """

        def n_predecessors(_lanelet: Lanelet, n):
            if _lanelet is None:
                return []
            lanelets = _lanelet.predecessor
            if n <= 1:
                return lanelets
            recursive_search = []
            for let in lanelets:
                recursive_search.append(let)
                let = self.network.find_lanelet_by_id(let)
                recursive_search += n_predecessors(let, n - 1)
            return recursive_search

        lanelet, _ = self.obstacle_lanelet(pos=self._position)

        def process(origin):
            _trace_pts = []
            path2 = []
            for pred in n_predecessors(lanelet, 3):
                path2 += xy_to_pts(self.network.find_lanelet_by_id(pred).center_vertices)
                path2 += path
            for _, forward, backward, _ in origin:
                idx, distance = closest_index_of_point_list(path2[path_idx : path_idx + 500], forward[0], acc)
                if idx < 0:
                    continue
                if distance <= self.ON_LANELET_DISTANCE:
                    continue  # following vehicles not relevant to tracing

                to_add = []
                d = self.ON_LANELET_DISTANCE
                for pt in forward[1:]:
                    if dist(self._position, pt) < d or dist(self._position, backward[0]) < d:
                        break
                    to_add.append(pt)
                _trace_pts += to_add  # + backward
            return _trace_pts

        ped_traces = process(self.biker_and_peds_on_road_traces)
        veh_traces = process(self.vehicle_traces)
        trace_pts = ped_traces + veh_traces

        if self.SHOW_TRACE_PTS:
            self._draw_path_pts(xy_to_pts(trace_pts), "relevant_trace_pts", (255, 51, 204))

        return ped_traces, veh_traces

    def _calculate_follow_info_with_trace_points(
        self,
        ped_trace_points: List[Point2D],
        veh_trace_points: List[Point2D],
        path_pts: List[Point2D],
        local_path_index: int,
        acc: int,
    ) -> Tuple[Optional[PafObstacleFollowInfo], List[Point2D]]:
        """
        Calculate the closest relevant trace point available
        :param ped_trace_points: list of pedestrian trace points (path prediction)
        :param veh_trace_points: list of vehicle trace points (path prediction)
        :param path_pts: list of points (local path)
        :param local_path_index: path index of ego vehicle
        :param acc: index search accuracy (higher is lower accuracy)
        :return: (follow info, points following)
        """
        msgs = []
        distances = []
        local_path_index += 4
        pts = []
        relevant_path = path_pts[local_path_index:]

        def process(trace_pts, is_vehicle):
            for i, pt in enumerate(relevant_path):
                indices, _distances = k_closest_indices_of_point_in_list(2, trace_pts, pt, acc)
                i += local_path_index
                pt = path_pts[max(i - 20, 0)]
                for _idx, distance in zip(indices, _distances):
                    trace_pt = trace_pts[_idx]
                    if distance < self.ON_LANELET_DISTANCE:
                        msg = PafObstacleFollowInfo()
                        msg.speed = 0
                        msg.distance = dist(pt, self._position)
                        msg.no_target = False
                        msg.is_vehicle = is_vehicle
                        msgs.append(msg)
                        distances.append(msg.distance)
                        pts.append(trace_pt)
                        break

        process(ped_trace_points, False)
        process(veh_trace_points, True)

        if len(msgs) == 0:
            return None, []

        idx = np.argmin(distances)
        return msgs[idx], [pts[idx]]

    def _angle_rel_to_ego_yaw(self, ref_pt: np.ndarray) -> float:
        """
        Calculate the angle between the curren yaw of the vehicle and a reference point based steering vector in rads
        :param ref_pt: reference point. steering angle = (ref_pt-position)
        :return: radians [-pi,+pi]
        """
        angle = get_angle_between_vectors(ref_pt - np.array([self._position.x, self._position.y])) - self._current_yaw
        if angle > np.pi:
            angle -= np.pi * 2
        if angle <= -np.pi:
            angle += np.pi * 2
        return angle

    def _calculate_follow_vehicle_with_path(
        self, path_pts, ego_vehicle_index, acc
    ) -> Tuple[PafObstacleFollowInfo, List[Any]]:
        """
        Calculate the closest follow info based on the obstacle positions and the local path
        :param path_pts: points on LP
        :param ego_vehicle_index: index of ego vehicle on LP
        :param acc: index search accuracy (higher is lower accuracy)
        :return: (follow info, follow info points)
        """
        obs_on_path = []
        path_indices = []
        debug_pts = []

        limit_angle = np.deg2rad(45)
        relevant_path_pts = path_pts[ego_vehicle_index : ego_vehicle_index + 500][::acc]
        relevant_obstacles = [
            (fwd_trace[0], obs) for obs, fwd_trace, _, _ in self.vehicle_traces + self.biker_and_peds_on_road_traces
        ]

        obstacle: PafObstacle
        for ref_pt, obstacle in relevant_obstacles:
            idx, distance_to_path = closest_index_of_point_list(relevant_path_pts, ref_pt)
            if idx == -1:
                continue
            path_pt = relevant_path_pts[idx]
            distance_to_obstacle = dist(ref_pt, self._position)
            if distance_to_obstacle < 1:
                continue
            if distance_to_path > self.ON_LANELET_DISTANCE and distance_to_obstacle > self.ON_LANELET_DISTANCE:
                if distance_to_path > 1.5 * self.ON_LANELET_DISTANCE:
                    continue  # too far away from path
                path_lanelets = self.network.find_lanelet_by_shape(
                    Circle(self.ON_LANELET_DISTANCE * 0.75, np.array([path_pt.x, path_pt.y], dtype=float))
                )

                if len(path_lanelets) <= 1:
                    continue  # is parallel lane
                rel_angle = self._angle_rel_to_ego_yaw(ref_pt)
                if not (-limit_angle <= rel_angle <= limit_angle):
                    # not within view cone of ego vehicle (distance limit still applies)
                    continue
                rospy.logwarn_throttle(
                    4,
                    f"[obstacle info] including vehicle distance_to_path={distance_to_path} "
                    f"num_lanelets={len(path_lanelets)}, within_angle=False",
                )
                # else: is lane change (on_path_dist = 2 * on_lanelet_dist)
            debug_pts += [ref_pt, path_pt]
            obs_on_path.append(obstacle)
            path_indices.append(idx)

        if len(obs_on_path) == 0:
            info = self.get_follow_info()
            debug_pts = []
        else:
            i = np.argmin(path_indices)
            obs = obs_on_path[i]
            info = self.get_follow_info(obs, is_vehicle=True)
            debug_pts = debug_pts[2 * i : 2 * (i + 1)]

        return info, debug_pts

    def _odometry_updated(self, odometry: Odometry):
        """Odometry Update Callback"""
        _, _, self._current_yaw = euler_from_quaternion(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        )
        if self._current_yaw < 0:
            self._current_yaw += np.pi * 2
        self._position = odometry.pose.pose.position

    @staticmethod
    def get_follow_info(following: PafObstacle = None, is_vehicle: bool = True) -> PafObstacleFollowInfo:
        """
        Return follow info based on obstacle message or return an "empty" follow if following is None
        :param following: optional obstacle message
        :param is_vehicle: vehicle or pedestrian switch
        :return: follow info object
        """
        if following is None:
            return PafObstacleFollowInfo(0, 0, True, is_vehicle)
        distance = following.distance
        return PafObstacleFollowInfo(following.speed, distance, False, is_vehicle)

    def trace_obstacle_with_lanelet_network(
        self, msg: PafObstacle, unknown_trace_length: float = 4, min_trace_length: float = 3
    ) -> Tuple[Optional[List[Point2D]], Optional[List[Point2D]], Optional[Lanelet]]:
        """
        Calculate a trace forward and backward trace for a given obstacle
        :param msg: obstacle message
        :param unknown_trace_length: length of trace if speed is unknown (currently speed is always ignored) (meter)
        :param min_trace_length: length of trace should not be smaller than this (meter)
        :return: (forward trace, backward trace, lanelet object of obstacle)
        """
        chosen_lanelet, idx_start = self.obstacle_lanelet(msg)
        if idx_start < 0:
            return None, None, None
        # experimental result: better performance with fixed, small trace lengths not based on speed
        # trace_meters = trace_seconds * msg.speed if msg.speed_known else unknown_trace_length
        trace_meters = unknown_trace_length
        trace_meters = max(trace_meters, min_trace_length)
        trace_fwd, trace_bwd = self.trace_lanelet(
            chosen_lanelet, trace_meters, idx_start, accuracy_m=1, offset_backwards_m=1
        )
        return trace_fwd, trace_bwd, chosen_lanelet

    def trace_lanelet(
        self,
        start_lanelet: Union[int, Lanelet],
        forwards_m: float,
        start_index: int,
        accuracy_m: float,
        offset_backwards_m: float,
    ):
        """
        Lanelet tracing forward and backward from given index (recursive)
        :param start_lanelet: lanelet object or id
        :param forwards_m: meters to trace forward
        :param start_index: index to start at
        :param accuracy_m: index search accuracy (higher is lower accuracy) (meter)
        :param offset_backwards_m: meters to trace backwards
        :return: list of trace points
        """

        def __trace_lanelet(
            _start_lanelet: Union[int, Lanelet],
            _meters: float,
            _start_index: int,
            _accuracy_m: float,
            _trace_forward: bool,
        ) -> Tuple[List[Point2D], List[Tuple[List[Point2D], Any]], List[Point2D]]:
            """
            Recursive part of the outer function
            :param _start_lanelet: lanelet object or id
            :param _meters: meters to trace
            :param _start_index: index to start at
            :param _accuracy_m: index search accuracy (meter)
            :param _trace_forward: trace forward or backward
            :return: (points found this depth, points found on recursive calls (nested), all points found)
            """
            if type(_start_lanelet) is int:
                _start_lanelet = self.network.find_lanelet_by_id(_start_lanelet)
            points = []
            all_points = []
            other_traces = []
            current_distance = 0  # len(points) / dbp
            accuracy = round(_accuracy_m / 0.125)
            _start_index = max(0, _start_index)
            if _trace_forward:
                vertices = _start_lanelet.center_vertices[_start_index:][::accuracy]
            elif _start_index == 0:
                vertices = _start_lanelet.center_vertices[::accuracy][::-1]
            else:
                vertices = _start_lanelet.center_vertices[: _start_index + 1][::accuracy][::-1]
            points.append(vertices[0])
            all_points.append(vertices[0])
            for p1, p2 in zip(vertices, vertices[1:]):
                if current_distance >= _meters:
                    break
                points.append(p2)
                current_distance += dist(p1, p2)
            all_points += points
            if current_distance < _meters:
                remaining = _meters - current_distance
                if _trace_forward:
                    successors = _start_lanelet.successor
                else:
                    successors = _start_lanelet.predecessor
                for successor in successors:
                    _points, _other_traces, _all_points = __trace_lanelet(
                        successor, remaining, 0, _accuracy_m, _trace_forward
                    )
                    other_traces.append((_points, _other_traces))
                    all_points += _all_points
            return points, other_traces, all_points

        _, _, forward_trace = __trace_lanelet(start_lanelet, forwards_m, start_index, accuracy_m, True)
        _, _, backward_trace = __trace_lanelet(start_lanelet, offset_backwards_m, start_index, accuracy_m, False)
        return forward_trace, backward_trace

    def obstacle_lanelet(self, msg: PafObstacle = None, pos: Point2D = None) -> Tuple[Optional[Lanelet], int]:
        """
        Determine on which lanelet the obstacle is on
        :param msg: obstacle
        :param pos: optional alternative position to the closest obstacle point
        :return: (chosen lanelet, obstacle index on lanelet vertices)
        """
        if pos is not None:
            pts = [np.array([pos.x, pos.y], dtype=float)]
        else:
            # , msg.bound_1, msg.bound_2]]
            pts = [np.array(pt, dtype=float) for pt in [msg.closest]]
        result = self.network.find_lanelet_by_position(pts)
        lanelets = []
        for res in result:
            lanelets += res

        if len(lanelets) == 0:
            lanelets += self.network.find_lanelet_by_shape(Circle(5, pts[0]))
        if len(lanelets) == 0:
            return None, -1
        ref_pt = tuple(np.average(pts, axis=0))
        chosen_lanelet = np.bincount(lanelets).argmax()
        chosen_lanelet = self.network.find_lanelet_by_id(chosen_lanelet)
        idx_start, _ = closest_index_of_point_list(list(chosen_lanelet.center_vertices), ref_pt)
        return chosen_lanelet, idx_start

    @staticmethod
    def _draw_path_pts(
        points: List[Point2D],
        lbl: str = "obs_pts",
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
    def start():
        """
        Start ROS node
        """
        rospy.spin()


if __name__ == "__main__":
    ObstaclePlanner().start()
