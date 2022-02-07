#!/usr/bin/env python
from typing import List, Tuple, Union
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
from classes.HelperFunctions import closest_index_of_point_list, xy_to_pts, get_angle_between_vectors, dist
import numpy as np

from tf.transformations import euler_from_quaternion


class ObstaclePlanner:
    rules_enabled = MapManager.get_rules_enabled()
    network = MapManager.get_current_scenario().lanelet_network
    ON_LANELET_DISTANCE = 2

    def __init__(self):
        rospy.init_node("obstacle_planner_node", anonymous=True)
        # role_name = rospy.get_param("~role_name", "ego_vehicle")
        rospy.logwarn(f"[obstacle planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")
        rospy.Subscriber("/paf/paf_perception/obstacles", PafObstacleList, self._process_obstacles, queue_size=1)
        rospy.Subscriber("/paf/paf_local_planner/path", PafLocalPath, self._process_local_path, queue_size=1)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        self._follow_info_pub = rospy.Publisher(
            "/paf/paf_obstacle_planner/following_info", PafObstacleFollowInfo, queue_size=1
        )

        self.debug_pts_ped = []
        self.debug_pts_veh = []
        self.vehicle_traces = []
        self.biker_and_peds_on_road_traces = []
        self._last_local_path = PafLocalPath()
        self._position = Point2D(0, 0)
        # rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        # self._current_speed, self._current_pitch, self._current_yaw = 0, 0, 0
        # self._current_pose = Pose()

    def _process_obstacles(self, msg: PafObstacleList):

        # rospy.loginfo_throttle(
        #     0.5, f"[obstacle planner] processing {len(msg.obstacles)} obstacles of type '{msg.type}'"
        # )

        if msg.type == "Pedestrians":
            process_fun = self.process_pedestrian
            self.biker_and_peds_on_road_traces = []
            self.debug_pts_ped = []
        elif msg.type == "Vehicles":
            process_fun = self.process_vehicle
            self.debug_pts_veh = []
            self.vehicle_traces = []
        else:
            raise TypeError(f"unknown obstacle type: {msg.type}")

        o: PafObstacle
        for o in msg.obstacles:
            process_fun(o)
        # self._draw_path_pts(self.debug_pts_ped if msg.type == "Pedestrians" else self.debug_pts_veh, msg.type, color)

    def process_pedestrian(self, msg: PafObstacle):
        forward, backward = self.trace_obstacle_with_lanelet_network(msg)
        if forward is None:  # not on lanelet network (pedestrians) -> ignore
            return
        if self.pedestrian_on_lanelet(msg):
            self.biker_and_peds_on_road_traces.append((msg, forward, backward))

    def pedestrian_on_lanelet(self, ped: PafObstacle):
        idx, distance = closest_index_of_point_list(self._last_local_path.points, ped.closest)
        if distance <= self.ON_LANELET_DISTANCE:
            return True
        return False

    def process_vehicle(self, msg: PafObstacle):
        forward, backward = self.trace_obstacle_with_lanelet_network(msg)
        if forward is None:
            return
        self.debug_pts_veh += xy_to_pts(forward + backward)
        self.vehicle_traces.append((msg, forward, backward))

    def angle_difference(self, lanelet_id, velocity_vector, ref_pt):
        lanelet = self.network.find_lanelet_by_id(lanelet_id)
        idx_start, _ = closest_index_of_point_list(list(lanelet.center_vertices), ref_pt)
        idx_start = max(idx_start, len(lanelet.center_vertices) - 2)
        p1, p2 = lanelet.center_vertices[idx_start : idx_start + 2]
        angle_diff = get_angle_between_vectors(p2 - p1, velocity_vector)
        return lanelet, idx_start, angle_diff

    def _process_local_path(self, msg: PafLocalPath):
        self._last_local_path = msg
        acc = 10
        max_distance = self.ON_LANELET_DISTANCE
        obs_on_path = []
        path_indices = []
        debug_pts = []
        start_idx, _ = closest_index_of_point_list(msg.points, self._position)
        # start_idx -= 5
        start_idx = max(0, start_idx)
        obstacle: PafObstacle
        for obstacle, forward_trace, backward_trace in self.vehicle_traces + self.biker_and_peds_on_road_traces:
            if forward_trace is None:
                ref_pt = obstacle.closest
            else:
                ref_pt = forward_trace[0]
            idx, distance_to_path = closest_index_of_point_list(msg.points[start_idx : start_idx + 500], ref_pt, acc)
            if idx == -1 or idx in path_indices:
                # rospy.loginfo_throttle(.33, f"continue1 {idx} {idx in path_indices}")
                continue
            idx = max(idx + start_idx - int(acc / 2), 0)
            distance_to_obstacle = dist(ref_pt, self._position)
            if distance_to_path > max_distance and distance_to_obstacle > max_distance:
                # if distance_to_path < 5:
                #     rospy.loginfo_throttle(.33, f"continue2 d_pt={distance_to_path:.1f}
                #     d_ob={distance_to_obstacle:.1f} max={max_distance:.1f} ")
                continue
            debug_pts += [ref_pt, msg.points[idx]]
            obs_on_path.append(obstacle)
            path_indices.append(idx)

        if len(obs_on_path) == 0:
            info = self.get_follow_info()
            debug_pts = []
        else:
            i = np.argmin(path_indices)
            obs = obs_on_path[i]
            info = self.get_follow_info(obs)
            debug_pts = xy_to_pts(debug_pts[2 * i : 2 * (i + 1)])
        self._draw_path_pts(debug_pts, "relevant_obs_pts")
        # rospy.loginfo_throttle(1, f"{info.distance:.2f} {info.speed*3.6:.2f} {len(self.vehicle_traces)}")
        self._follow_info_pub.publish(info)

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

    def trace_obstacle_with_lanelet_network(self, msg: PafObstacle, trace_seconds=2, unknown_trace_length=2):
        chosen_lanelet, idx_start = self.obstacle_lanelet(msg)
        if idx_start < 0:
            return None, None
        trace_meters = trace_seconds * msg.speed if msg.speed_known else unknown_trace_length
        return self.trace_lanelet(chosen_lanelet, trace_meters, idx_start, accuracy_m=1, offset_backwards_m=1)

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

    def obstacle_lanelet(self, msg: PafObstacle):
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

        # try:
        #     if msg.speed == 0:
        #         raise ValueError
        #     angles = []
        #     indices = []
        #     lanelets_cr = []
        #     for let in lanelets:
        #         lanelet, idx_start, angle_diff = self.angle_difference(let, msg.velocity_vector, ref_pt)
        #
        #         lanelets_cr.append(lanelet)
        #         indices.append(idx_start)
        #         angles.append(np.abs(angle_diff))
        #
        #     idx = np.argmin(angles)
        #     chosen_lanelet = lanelets_cr[idx]
        #     idx_start = indices[idx]
        # except ValueError:
        #     ...
        chosen_lanelet = np.bincount(lanelets).argmax()
        chosen_lanelet = self.network.find_lanelet_by_id(chosen_lanelet)
        idx_start, _ = closest_index_of_point_list(list(chosen_lanelet.center_vertices), ref_pt)
        return chosen_lanelet, idx_start

    # def _odometry_updated(self, odometry: Odometry):
    #     """Odometry Update Callback"""
    #
    #     # calculate current speed (m/s) from twist
    #     self._current_speed = np.sqrt(
    #         odometry.twist.twist.linear.x ** 2 +
    #         odometry.twist.twist.linear.y ** 2 + odometry.twist.twist.linear.z ** 2
    #     )
    #     _, self._current_pitch, self._current_yaw = euler_from_quaternion(
    #         [
    #             odometry.pose.pose.orientation.x,
    #             odometry.pose.pose.orientation.y,
    #             odometry.pose.pose.orientation.z,
    #             odometry.pose.pose.orientation.w,
    #         ]
    #     )
    #     self._current_pose = odometry.pose.pose

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
