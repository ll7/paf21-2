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

from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


class ObstaclePlanner:
    rules_enabled = MapManager.get_rules_enabled()
    network = MapManager.get_current_scenario().lanelet_network
    ON_LANELET_DISTANCE = 2
    SHOW_TRACE_PTS = True
    SHOW_FOLLOW_PTS = True

    def __init__(self):
        rospy.init_node("obstacle_planner_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        rospy.logwarn(f"[obstacle planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")
        rospy.Subscriber("/paf/paf_perception/obstacles", PafObstacleList, self._process_obstacles, queue_size=1)
        rospy.Subscriber("/paf/paf_local_planner/path", PafLocalPath, self._process_local_path, queue_size=1)
        rospy.Subscriber(
            "/paf/paf_local_planner/follow_trace_points", Bool, self._process_follow_trace_points, queue_size=1
        )
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        self._follow_info_pub = rospy.Publisher(
            "/paf/paf_obstacle_planner/following_info", PafObstacleFollowInfo, queue_size=1
        )
        self._obstacle_traces_pub = rospy.Publisher(
            "/paf/paf_obstacle_planner/obstacle_traces", PafObstacleFollowInfo, queue_size=1
        )

        self.follow_trace_points = True
        self.vehicle_traces = []
        self.biker_and_peds_on_road_traces = []
        self._last_local_path = PafLocalPath()
        self._position = Point2D(0, 0)

    def _process_follow_trace_points(self, msg: Bool):
        self.follow_trace_points = msg.data

    def _process_obstacles(self, msg: PafObstacleList):
        if msg.type == "Pedestrians":
            process_fun = self.process_pedestrian
            self.biker_and_peds_on_road_traces = []
        elif msg.type == "Vehicles":
            process_fun = self.process_vehicle
            self.vehicle_traces = []
        else:
            raise TypeError(f"unknown obstacle type: {msg.type}")

        o: PafObstacle
        for o in msg.obstacles:
            process_fun(o)

    def process_pedestrian(self, msg: PafObstacle):
        forward, backward, current_lanelet = self.trace_obstacle_with_lanelet_network(msg)
        if forward is None:  # not on lanelet network (pedestrians) -> ignore
            return
        if self.pedestrian_on_lanelet(msg):
            self.biker_and_peds_on_road_traces.append((msg, forward, backward, current_lanelet))

    def pedestrian_on_lanelet(self, ped: PafObstacle):
        idx, distance = closest_index_of_point_list(self._last_local_path.points, ped.closest)
        if distance <= self.ON_LANELET_DISTANCE:
            return True
        return False

    def process_vehicle(self, msg: PafObstacle):
        forward, backward, current_lanelet = self.trace_obstacle_with_lanelet_network(msg)
        if forward is None:
            return
        self.vehicle_traces.append((msg, forward, backward, current_lanelet))

    def angle_difference(self, lanelet_id, velocity_vector, ref_pt):
        lanelet = self.network.find_lanelet_by_id(lanelet_id)
        idx_start, _ = closest_index_of_point_list(list(lanelet.center_vertices), ref_pt)
        idx_start = max(idx_start, len(lanelet.center_vertices) - 2)
        p1, p2 = lanelet.center_vertices[idx_start : idx_start + 2]
        angle_diff = get_angle_between_vectors(p2 - p1, velocity_vector)
        return lanelet, idx_start, angle_diff

    def _process_local_path(self, msg: PafLocalPath):
        self._last_local_path = msg
        vehicle_follow_info = self.get_follow_info()
        info_pts = []
        local_path_index, distance_to_lanelet_network = closest_index_of_point_list(msg.points, self._position)
        if distance_to_lanelet_network <= 2.5:
            vehicle_follow_info, info_pts = self._calculate_follow_vehicle_with_path(msg.points, local_path_index)
        if self.follow_trace_points:
            points = self.get_relevant_trace_points(msg.points, local_path_index)
            trace_follow_info, info2_pts = self._calculate_follow_vehicle_with_trace_points(
                points, msg.points, local_path_index
            )
            if trace_follow_info is not None and (
                trace_follow_info.distance < vehicle_follow_info.distance or vehicle_follow_info.no_target
            ):
                rospy.loginfo_throttle(5, "[obstacle planner] following trace")
                vehicle_follow_info = trace_follow_info
                info_pts = info2_pts
        rospy.loginfo_throttle(
            5,
            f"[obstacle planner] following={not vehicle_follow_info.no_target} v={vehicle_follow_info.speed} "
            f"d={vehicle_follow_info.distance:.1f}",
        )
        if self.SHOW_FOLLOW_PTS:
            self._draw_path_pts(xy_to_pts(info_pts), "relevant_obs_pts")
        self._follow_info_pub.publish(vehicle_follow_info)

    def get_relevant_trace_points(self, path: List[Point2D], path_idx: int):
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
        path2 = []
        trace_pts = []
        for pred in n_predecessors(lanelet, 3):
            path2 += xy_to_pts(self.network.find_lanelet_by_id(pred).center_vertices)
            path2 += path
        for _, forward, backward, _ in self.vehicle_traces + self.biker_and_peds_on_road_traces:
            idx, distance = closest_index_of_point_list(path2[path_idx : path_idx + 500], forward[0], 10)
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
            trace_pts += to_add  # + backward
        if self.SHOW_TRACE_PTS:
            self._draw_path_pts(xy_to_pts(trace_pts), "relevant_trace_pts", (255, 51, 204))
        return trace_pts

    def _calculate_follow_vehicle_with_trace_points(self, trace_pts, path_pts, local_path_index):
        msgs = []
        distances = []
        local_path_index += 4
        pts = []
        relevant_path = path_pts[local_path_index::10]
        for i, pt in enumerate(relevant_path):
            indices, distances = k_closest_indices_of_point_in_list(2, trace_pts, pt)
            if i > 4:
                pt = relevant_path[i - 4]
            for idx, distance in zip(indices, distances):
                trace_pt = trace_pts[idx]
                # angle = self._angle_rel_to_ego_yaw(trace_pt)
                # if -np.pi / 2 <= angle <= np.pi / 2:
                #     continue  # only consider traces on the sides
                if distance < self.ON_LANELET_DISTANCE:
                    msg = PafObstacleFollowInfo()
                    msg.speed = 0
                    msg.distance = dist(pt, self._position)
                    msg.no_target = False
                    msgs.append(msg)
                    distances.append(msg.distance)
                    pts.append(trace_pt)
                    break
        if len(msgs) == 0:
            return None, []

        idx = np.argmin(distances)
        return msgs[idx], [pts[idx]]

    def _angle_rel_to_ego_yaw(self, ref_pt: np.ndarray):
        return get_angle_between_vectors(ref_pt - np.array([self._position.x, self._position.y])) - np.pi

    def _pt_within_angle_range(self, ref_pt, limit_angle=np.deg2rad(45)):
        rel_angle = self._angle_rel_to_ego_yaw(ref_pt)
        return -limit_angle <= rel_angle <= limit_angle

    def _calculate_follow_vehicle_with_path(
        self, path_pts, ego_vehicle_index
    ) -> Tuple[PafObstacleFollowInfo, List[Any]]:
        obs_on_path = []
        path_indices = []
        debug_pts = []

        relevant_path_pts = path_pts[ego_vehicle_index : ego_vehicle_index + 500][::10]
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
                if not self._pt_within_angle_range(ref_pt):
                    continue  # not within view cone of ego vehicle (distance limit still applies)
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
            info = self.get_follow_info(obs)
            debug_pts = debug_pts[2 * i : 2 * (i + 1)]

        return info, debug_pts

    def _odometry_updated(self, odometry: Odometry):
        """Odometry Update Callback"""
        _, self._current_pitch, self._current_yaw = euler_from_quaternion(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        )
        self._position = odometry.pose.pose.position

    @staticmethod
    def get_follow_info(following: PafObstacle = None):
        if following is None:
            return PafObstacleFollowInfo(0, 0, True)
        return PafObstacleFollowInfo(following.speed, following.distance - 3, False)

    def trace_obstacle_with_lanelet_network(
        self, msg: PafObstacle, trace_seconds=2, unknown_trace_length=4, min_trace_length=3
    ):
        chosen_lanelet, idx_start = self.obstacle_lanelet(msg)
        if idx_start < 0:
            return None, None, None
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
        _, _, forward_trace = self.__trace_lanelet(start_lanelet, forwards_m, start_index, accuracy_m, True)
        _, _, backward_trace = self.__trace_lanelet(start_lanelet, offset_backwards_m, start_index, accuracy_m, False)
        return forward_trace, backward_trace

    def __trace_lanelet(
        self,
        start_lanelet: Union[int, Lanelet],
        meters: float,
        start_index: int,
        accuracy_m: float,
        trace_forward: bool,
    ):
        if type(start_lanelet) is int:
            start_lanelet = self.network.find_lanelet_by_id(start_lanelet)
        points = []
        all_points = []
        other_traces = []
        current_distance = 0  # len(points) / dbp
        accuracy = round(accuracy_m / 0.125)
        start_index = max(0, start_index)
        if trace_forward:
            vertices = start_lanelet.center_vertices[start_index:][::accuracy]
        elif start_index == 0:
            vertices = start_lanelet.center_vertices[::accuracy][::-1]
        else:
            vertices = start_lanelet.center_vertices[: start_index + 1][::accuracy][::-1]
        points.append(vertices[0])
        all_points.append(vertices[0])
        for p1, p2 in zip(vertices, vertices[1:]):
            if current_distance >= meters:
                break
            points.append(p2)
            current_distance += dist(p1, p2)
        all_points += points
        if current_distance < meters:
            remaining = meters - current_distance
            if trace_forward:
                successors = start_lanelet.successor
            else:
                successors = start_lanelet.predecessor
            for successor in successors:
                _points, _other_traces, _all_points = self.__trace_lanelet(
                    successor, remaining, 0, accuracy_m, trace_forward
                )
                other_traces.append((_points, _other_traces))
                all_points += _all_points
        return points, other_traces, all_points

    def obstacle_lanelet(self, msg: PafObstacle = None, pos: Point2D = None) -> Tuple[Optional[Lanelet], int]:
        if pos is not None:
            pts = [np.array([pos.x, pos.y], dtype=float)]
        else:
            pts = [np.array(pt, dtype=float) for pt in [msg.closest]]  # , msg.bound_1, msg.bound_2]]
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

    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._last_local_path is None:
                continue
            self._process_local_path(self._last_local_path)
            rate.sleep()


if __name__ == "__main__":
    ObstaclePlanner().start()
