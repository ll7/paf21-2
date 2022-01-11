#!/usr/bin/env python
from time import perf_counter

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from paf_messages.msg import PafObstacleList, PafObstacle
from tf.transformations import euler_from_quaternion


class SemanticLidarNode(object):
    TAGS = {int(k): v for k, v in rospy.get_param("semantic_tags").items()}
    MIN_DIST_IS_SELF = 0  # not needed if z >= 2.4
    MIN_OBJ_DIFF_DIST = 5
    SENSOR_X_OFFSET = -0.3  # x value of lidar1 in paf_perception/sensors.json
    SKIP_NORMALIZING_BOUND_DIST = False
    LOG_FPS_SECS = 60

    def __init__(self):
        rospy.init_node("semantic_lidar", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
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

        self.previous_obstacles = {}
        self.previous_time = rospy.Time.now().to_time()

    @staticmethod
    def _get_max_ai_speed(position: list) -> float:
        """
        Retrieves max speed from position by asking the MapManager
        :param position: x,y coordinates of the map
        :return: speed value in m/s
        """
        max_ai_speed = 150 / 3.6  # in m/s todo: use lanelet speed limit instead
        return max_ai_speed

    def _process_odometry(self, msg: Odometry):
        """
        Odometry topic callback
        :param msg:
        """
        t0 = rospy.Time.now().to_time()
        self.xy_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        current_pose = msg.pose.pose
        quaternion = (
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.z_orientation = -yaw
        # rotation of result in opposite direction to get world coords
        self.cos = np.cos(-self.z_orientation)
        self.sin = np.sin(-self.z_orientation)
        time = rospy.Time.now().to_time()
        delta = time - t0
        if delta > 0:
            rospy.loginfo_throttle(self.LOG_FPS_SECS, f"[semantic lidar] max odo fps={1 / delta}")
        delta = time - self.frame_time_odo
        if delta > 0:
            rospy.loginfo_throttle(self.LOG_FPS_SECS, f"[semantic lidar] current odo fps={1 / delta}")
        self.frame_time_odo = time

    def _transform_to_relative_world_pos(self, leftwards: float, backwards: float) -> tuple:
        """
        Rotates the lidar coordinates to the correct orientation relative to ego_vehicle
        :param leftwards: lidar point variable leftwards
        :param backwards: lidar point variable backwards
        :return: x,y in world coordinates relative to ego_vehicle x,y
        """
        x = leftwards + self.SENSOR_X_OFFSET
        y = backwards
        x, y = x * self.cos - y * self.sin, y * self.cos + x * self.sin
        return x, y

    def _process_lidar_semantic(self, msg: PointCloud2):
        """
        Callback for semantic lidar topic
        :param msg:
        """
        if self.xy_position is None:
            return
        t0 = rospy.Time.now().to_time()
        points = pc2.read_points(msg, skip_nans=True)
        sorted_points = self._process_lidar_points_by_tag_and_idx(points)
        pois_by_tag = self._process_sorted_points(sorted_points)
        self._publish_object_information(pois_by_tag)
        time = rospy.Time.now().to_time()
        delta = time - t0
        if delta > 0:
            rospy.loginfo_throttle(self.LOG_FPS_SECS, f"[semantic lidar] max lidar fps={1 / delta}")
        delta = time - self.frame_time_lidar
        if delta > 0:
            rospy.loginfo_throttle(self.LOG_FPS_SECS, f"[semantic lidar] current lidar fps={1 / delta}")
        self.frame_time_lidar = time

    def _process_sorted_points(self, sorted_points: dict) -> dict:
        """
        Calculates bound1, bound2, closest point (each in x,y,d coords) and speed (in m/s)

        :param sorted_points: format { obj_index : { "tag" : tag, "pts": [[x,y,d],...] , ...}
        :return: pois in format { tag : [ [bound1, bound2, closest, speed, int_id], ...] , ...} , speed (in m/s)
        """
        poi_by_tag = {}
        previous_obstacles = self.previous_obstacles
        self.previous_obstacles = {}
        current_time = rospy.Time.now().to_time()

        for obj_idx, pts_and_tag in sorted_points.items():
            tag = pts_and_tag["tag"]
            if tag not in poi_by_tag:
                poi_by_tag[tag] = []
            for i, pts in enumerate(pts_and_tag["pts"]):
                poi = self._calculate_bounds(pts)

                obj_id_string = f"{tag}-{obj_idx}-{i}"
                obj_id_int = self._create_int_id(tag, obj_idx)
                speed = self._get_obj_speed(previous_obstacles, poi, obj_id_string, obj_idx, current_time)
                if speed is None and len(previous_obstacles) > 0:
                    pass
                distance = self._dist(poi[2], self.xy_position)
                poi.append(distance)
                poi.append(speed)
                poi.append(obj_id_int)
                poi_by_tag[tag].append(poi)
                self.previous_obstacles[obj_id_string] = poi

        self.previous_time = current_time
        return poi_by_tag

    def _create_int_id(self, tag, obj_idx):
        """[create an integer id from a string tag and an idex"""
        if tag == "Vehicles":
            tag_int = 1
        elif tag == "Pedestrians":
            tag_int = 2
        else:
            tag_int = 3

        obj_id_int = obj_idx * 10 + tag_int
        return obj_id_int

    def _calculate_bounds(self, pts) -> list:
        """calculate bound1, bound2, closest_point in (x,y,d)-coordinates of an object by given points
        Args: points
        Returns:
            bounds in format [bound1, bound2, closest]
        """
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
        # add vehicle position to change to global coordinates
        poi = [p + self.xy_position for p in poi]

        return poi

    def _get_obj_speed(
        self, previous_obstacles: dict, obstacle_bounds: list, obj_id: str, obj_idx: int, current_time: float
    ):
        def speed_calc(xy):
            if xy is None:
                return None
            time_delta = current_time - self.previous_time
            distance_delta = self._dist(cur_closest, xy)
            if time_delta > 0:
                _speed = distance_delta / time_delta
                return _speed if _speed <= self._get_max_ai_speed(xy) else None

        cur_closest = obstacle_bounds[2]
        if obj_idx != 0 and obj_id in previous_obstacles:
            prev_closest = previous_obstacles[obj_id][2]
            speed = speed_calc(prev_closest)
            if speed is not None:
                return speed
        min_point, min_dist = None, None
        for poi in previous_obstacles.values():
            prev_closest = poi[2]
            dist = self._dist(cur_closest, prev_closest)
            if min_point is None or dist < min_dist:
                min_point, min_dist = prev_closest, dist
        return speed_calc(min_point)

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

    def _publish_object_information(self, pois_by_tag: dict):
        """
        publishes ObstacleList topic
        :param bounds_by_tag: format { tag : [ [bound1, bound2, closest, speed], ...] , ...}
        """
        header = Header()
        header.stamp = rospy.Time.now()
        for tag, values in pois_by_tag.items():
            obstacles = PafObstacleList()
            obstacles.type = tag
            obstacles.header = header
            obstacles.obstacles = []
            for i, (bound_1, bound_2, closest, distance, speed, id) in enumerate(values):
                obstacle = PafObstacle()
                obstacle.bound_1 = tuple(bound_1)
                obstacle.bound_2 = tuple(bound_2)
                obstacle.closest = tuple(closest)
                obstacle.distance = distance
                obstacle.speed_known = speed is not None
                if obstacle.speed_known:
                    obstacle.speed = speed
                obstacle.id = id

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
        bottom = d1 * d2
        top = x1 * x2 + y1 * y2
        if bottom == 0:
            fraction = 0
        else:
            fraction = top / bottom
        return np.arccos(np.clip(-1, 1, fraction))

    @staticmethod
    def _dist(p1: tuple, p2: tuple = (0, 0)) -> float:
        """
        Euclidean distance between two 2d points (or origin)
        :param p1: first point (x,y)
        :param p2: second point (defaults to zero)
        :return:
        """
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        return float(np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))

    @staticmethod
    def start():
        """
        starts ROS node
        """
        rospy.spin()


if __name__ == "__main__":
    node = SemanticLidarNode()
    node.start()
