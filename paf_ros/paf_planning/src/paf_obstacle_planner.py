#!/usr/bin/env python
from typing import List, Tuple, Union

from commonroad.geometry.shape import Circle
from commonroad.scenario.lanelet import Lanelet

import rospy
from classes.MapManager import MapManager
from paf_messages.msg import PafObstacleList, PafObstacle, PafTopDownViewPointSet, Point2D
from classes.HelperFunctions import closest_index_of_point_list, xy_to_pts, get_angle_between_vectors, dist
import numpy as np


class ObstaclePlanner:
    rules_enabled = MapManager.get_rules_enabled()
    network = MapManager.get_current_scenario().lanelet_network

    def __init__(self):
        rospy.init_node("obstacle_planner_node", anonymous=True)
        # role_name = rospy.get_param("~role_name", "ego_vehicle")
        rospy.logwarn(f"[obstacle planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")
        rospy.Subscriber("/paf/paf_perception/obstacles", PafObstacleList, self._process_obstacles, queue_size=1)
        self.debug_pts_ped = []
        self.debug_pts_veh = []
        # rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        # self._current_speed, self._current_pitch, self._current_yaw = 0, 0, 0
        # self._current_pose = Pose()

    def _process_obstacles(self, msg: PafObstacleList):

        rospy.loginfo_throttle(
            0.5, f"[obstacle planner] processing {len(msg.obstacles)} obstacles of type '{msg.type}'"
        )

        if msg.type == "Pedestrians":
            process_fun = self.process_pedestrian
            self.debug_pts_ped = []
            color = (0, 45, 123)
        elif msg.type == "Vehicles":
            process_fun = self.process_vehicle
            self.debug_pts_veh = []
            color = (45, 123, 0)
        else:
            raise TypeError(f"unknown obstacle type: {msg.type}")

        o: PafObstacle
        for o in msg.obstacles:
            process_fun(o)
        self._draw_path_pts(self.debug_pts_ped if msg.type == "Pedestrians" else self.debug_pts_veh, msg.type, color)

    def process_pedestrian(self, msg: PafObstacle):
        return

    def process_vehicle(self, msg: PafObstacle):
        chosen_lanelet, idx_start = self.obstacle_lanelet(msg)
        if idx_start < 0:
            return
        trace_meters = 5
        pts = self.trace_lanelet(chosen_lanelet, trace_meters, idx_start, accuracy_m=1, offset_backwards_m=10)
        self.debug_pts_veh += xy_to_pts(pts)  # [chosen_lanelet.center_vertices[idx_start]])

    def angle_difference(self, lanelet_id, velocity_vector, ref_pt):
        lanelet = self.network.find_lanelet_by_id(lanelet_id)
        idx_start, _ = closest_index_of_point_list(list(lanelet.center_vertices), ref_pt)
        idx_start = max(idx_start, len(lanelet.center_vertices) - 2)
        p1, p2 = lanelet.center_vertices[idx_start : idx_start + 2]
        angle_diff = get_angle_between_vectors(p2 - p1, velocity_vector)
        return lanelet, idx_start, angle_diff

    def trace_lanelet(
        self,
        start_lanelet: Union[int, Lanelet],
        meters: float,
        start_index: int,
        accuracy_m: float = 1,
        offset_backwards_m: float = 0,
    ):
        if type(start_lanelet) is int:
            start_lanelet = self.network.find_lanelet_by_id(start_lanelet)
        # dbp = .125
        points = []  # list(start_lanelet.center_vertices[start_index:start_index + int(meters / dbp)])
        current_distance = 0  # len(points) / dbp
        start_index = max(0, start_index)
        vertices = start_lanelet.center_vertices[start_index:][:: round(accuracy_m / 0.125)]
        for p1, p2 in zip(vertices, vertices[1:]):
            if current_distance >= meters:
                break
            points.append(p2)
            current_distance += dist(p1, p2)
        if current_distance < meters:
            remaining = meters - current_distance
            successors = start_lanelet.successor
            for successor in successors:
                points += self.trace_lanelet(successor, remaining, 0)
        return points

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

    @staticmethod
    def start():
        rospy.spin()


if __name__ == "__main__":
    ObstaclePlanner().start()
