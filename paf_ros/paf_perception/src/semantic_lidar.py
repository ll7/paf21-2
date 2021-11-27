#!/usr/bin/env python
from time import perf_counter

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from paf_perception.msg import PafObstacleList, PafObstacle
from tf.transformations import euler_from_quaternion


class SemanticLidarNode(object):
    TAGS = {int(k): v for k, v in rospy.get_param("semantic_tags").items()}
    MIN_DIST_IS_SELF = 0  # not needed if z >= 2.4
    MIN_OBJ_DIFF_DIST = 5
    SENSOR_X_OFFSET = -1
    SKIP_NORMALIZING_BOUND_DIST = False
    LOG_FPS_SECS = 60

    def __init__(self):
        rospy.init_node("semantic_lidar", anonymous=True)
        role_name = rospy.get_param("role_name")
        lidar_topic = f"/carla/{role_name}/semantic_lidar/lidar1/point_cloud"
        odometry_topic = f"/carla/{role_name}/odometry"
        rospy.logwarn(lidar_topic)
        rospy.logwarn(odometry_topic)
        rospy.Subscriber(odometry_topic, Odometry, self._process_odometry, queue_size=1)
        rospy.Subscriber(lidar_topic, PointCloud2, self._process_lidar_semantic, queue_size=1)
        self._obstacle_publisher = rospy.Publisher(rospy.get_param("obstacles_topic"), PafObstacleList, queue_size=1)
        self.ignored_indices = [0, 6, 7] + list(range(13, 23))
        self.xy_position = None
        self.z_orientation = None
        self.sin, self.cos = None, None
        self.frame_time_lidar = perf_counter()
        self.frame_time_odo = perf_counter()

    def _process_odometry(self, msg: Odometry):
        """
        Odometry topic callback
        :param msg:
        """
        t0 = perf_counter()
        self.xy_position = np.array([msg.pose.pose.position.x + self.SENSOR_X_OFFSET, -msg.pose.pose.position.y])
        current_pose = msg.pose.pose
        quaternion = (
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.z_orientation = yaw
        # rotation of result in opposite direction to get world coords
        self.cos = np.cos(-self.z_orientation)
        self.sin = np.sin(-self.z_orientation)
        time = perf_counter()
        rospy.logwarn_throttle(self.LOG_FPS_SECS, f"[semantic lidar] max odo fps={1 / (time - t0)}")
        rospy.logwarn_throttle(
            self.LOG_FPS_SECS, f"[semantic lidar] current odo fps={1 / (time - self.frame_time_odo)}"
        )
        self.frame_time_odo = time

    def _transform_to_relative_world_pos(self, leftwards: float, backwards: float) -> tuple:
        """
        Rotates the lidar coordinates to the correct orientation relative to ego_vehicle
        :param leftwards: lidar point variable leftwards
        :param backwards: lidar point variable backwards
        :return: x,y in world coordinates relative to ego_vehicle x,y
        """
        x = leftwards
        y = -backwards
        x, y = x * self.cos - y * self.sin, y * self.cos + x * self.sin
        return x, y

    def _process_lidar_semantic(self, msg: PointCloud2):
        """
        Callback for semantic lidar topic
        :param msg:
        """
        if self.xy_position is None:
            return
        t0 = perf_counter()
        points = pc2.read_points(msg, skip_nans=True)
        sorted_points = self._process_lidar_points_by_tag_and_idx(points)
        bounds_by_tag = self._process_sorted_points_calculate_bounds(sorted_points)
        self._publish_object_information(bounds_by_tag)
        time = perf_counter()
        rospy.logwarn_throttle(self.LOG_FPS_SECS, f"[semantic lidar] max lidar fps={1 / (time - t0)}")
        rospy.logwarn_throttle(
            self.LOG_FPS_SECS, f"[semantic lidar] current lidar fps={1 / (time - self.frame_time_lidar)}"
        )
        self.frame_time_lidar = time

    def _process_sorted_points_calculate_bounds(self, sorted_points: dict) -> dict:
        """
        Calculates bound1, bound2 and closest point (each in x,y,d coords)
        :param sorted_points: format { obj_index : { "tag" : tag, "pts": [[x,y,d],...] , ...}
        :return: bounds in format { tag : [ [bound1, bound2, closest], ...] , ...}
        """
        bounds_by_tag = {}
        for obj_idx, pts_and_tag in sorted_points.items():
            tag = pts_and_tag["tag"]
            if tag not in bounds_by_tag:
                bounds_by_tag[tag] = []
            for i, pts in enumerate(pts_and_tag["pts"]):
                a_max_x, a_max_y, a_max_d = np.argmax(pts, axis=0)
                a_min_x, a_min_y, a_min_d = np.argmin(pts, axis=0)

                # bounds: select two most distant points of interest (poi)
                poi = [pts[a_max_x], pts[a_min_x], pts[a_max_y], pts[a_min_y]]
                bound_1, bound_2 = self._get_vectors_with_maximum_angle(poi)

                # closest: select closest point to sensor
                closest = np.array((pts[a_min_d][:2]))
                if self.SKIP_NORMALIZING_BOUND_DIST:
                    # use original bound points
                    bound_2 = np.array(bound_2[:2])
                    bound_1 = np.array(bound_1[:2])
                else:
                    # shorten bound vectors to len(closest)
                    d_min = pts[a_min_d][-1]
                    bound_1 = np.array(bound_1[:2]) / bound_1[2] * d_min
                    bound_2 = np.array(bound_2[:2]) / bound_2[2] * d_min
                poi = [bound_1, bound_2, closest]
                poi = [p + self.xy_position for p in poi]
                bounds_by_tag[tag].append(poi)
        return bounds_by_tag

    def _process_lidar_points_by_tag_and_idx(self, points: list) -> dict:
        """
        Ordering and clustering points by index, tag and max allowed distance
        :param points: lidar points in format
            [[leftwards, backwards, downwards, cos_inc_angle, object_idx, object_tag],...]
        :return: sorted points in format { obj_index : { "tag" : tag, "pts": [[x,y,d],...] , ...}
        """
        objects = {}
        for point in points:
            leftwards, backwards, downwards, cos_inc_angle, object_idx, object_tag = point
            x, y = self._transform_to_relative_world_pos(leftwards, backwards)
            if object_tag in self.ignored_indices:
                continue
            object_tag = self.TAGS[object_tag]
            if object_idx == 0:
                continue
            else:
                d = self._dist((x, y))
                if d < self.MIN_DIST_IS_SELF:
                    continue
                if object_idx not in objects:
                    objects[object_idx] = {"pts": [], "tag": object_tag}
                for idx, objects_with_idx in enumerate(objects[object_idx]["pts"]):
                    _x, _y, _ = objects_with_idx[0]
                    _d = self._dist((x, y), (_x, _y))
                    if _d < self.MIN_OBJ_DIFF_DIST:
                        objects[object_idx]["pts"][idx].append([x, y, d])
                        break
                else:
                    objects[object_idx]["pts"].append([[x, y, d]])
        return objects

    def _publish_object_information(self, bounds_by_tag: dict):
        """
        publishes ObstacleList topic
        :param bounds_by_tag: format { tag : [ [bound1, bound2, closest], ...] , ...}
        """
        header = Header()
        header.stamp = rospy.Time.now()
        for tag, values in bounds_by_tag.items():
            obstacles = PafObstacleList()
            obstacles.type = tag
            obstacles.header = header
            obstacles.obstacles = []
            for i, (bound_1, bound_2, closest) in enumerate(values):
                obstacle = PafObstacle()
                obstacle.bound_1 = bound_1
                obstacle.bound_2 = bound_2
                obstacle.closest = closest
                obstacles.obstacles.append(obstacle)
            self._obstacle_publisher.publish(obstacles)

    @staticmethod
    def _get_vectors_with_maximum_angle(poi_list: list) -> list:
        """
        calculates the two vectors with maximum 2d-angular distance
        :param poi_list: list of (x,y,d) vectors
        :return: the two vectors with maximum 2d-angular distance
        """
        angle_max = -1
        pts_max = poi_list[:2]
        for j, p in enumerate(poi_list):
            k = j + 1
            for q in poi_list[k:]:
                angle = SemanticLidarNode._angle(p, q)
                if angle > angle_max:
                    angle_max = angle
                    pts_max = [p, q]
        return pts_max

    @staticmethod
    def _angle(p1: list, p2: list) -> float:
        """
        Angle between two vectors (x,y,d)
        :param p1: vector 1
        :param p2: vector 2
        :return: angle in rads
        """
        x1, y1, d1 = p1
        x2, y2, d2 = p2
        return np.arccos((x1 * x2 + y1 * y2) / (d1 * d2))

    @staticmethod
    def _dist(p1: tuple, p2: tuple = (0, 0)) -> float:
        """
        Euclidean distance between two 2d points (or origin)
        :param p1: first point (x,y)
        :param p2: second point (defaults to zero)
        :return:
        """
        x1, y1 = p1
        x2, y2 = p2
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    @staticmethod
    def start():
        """
        starts ROS node
        """
        rospy.spin()


if __name__ == "__main__":
    node = SemanticLidarNode()
    node.start()
