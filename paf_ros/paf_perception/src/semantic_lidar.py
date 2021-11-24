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
    MAGIC_X_OFFSET = 4

    def __init__(self):
        rospy.init_node("semantic_lidar", anonymous=True)
        role_name = rospy.get_param("role_name")
        topic1 = f"/carla/{role_name}/semantic_lidar/lidar1/point_cloud"
        topic2 = f"/carla/{role_name}/odometry"
        topic3 = rospy.get_param("obstacles_topic")
        rospy.Subscriber(topic1, PointCloud2, self.process_lidar_semantic)
        rospy.Subscriber(topic2, Odometry, self.process_odometry)
        rospy.Publisher(topic3, PafObstacleList, queue_size=1)
        self.ignored_indices = [0, 6, 7] + list(range(13, 23))
        self.xy_position = None
        self.z_orientation = None
        self.sin, self.cos = None, None

    def process_odometry(self, msg: Odometry):
        self.xy_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        current_pose = msg.pose.pose
        quaternion = (
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.z_orientation = yaw
        self.cos = np.cos(-self.z_orientation)
        self.sin = np.sin(-self.z_orientation)
        rospy.logwarn_throttle(10, f"yaw={np.rad2deg(yaw)}°")

    def transform_to_relative_world_pos(self, leftwards, backwards):
        x = leftwards - self.MAGIC_X_OFFSET
        y = -backwards
        x, y = x * self.cos - y * self.sin, y * self.cos + x * self.sin
        return x, y

    def process_lidar_semantic(self, msg: PointCloud2):
        if self.xy_position is None:
            return
        t0 = perf_counter()
        points = pc2.read_points(msg, skip_nans=True)
        point_count = 0
        objects = {}
        # static_objects = {}
        for point in points:
            leftwards, backwards, downwards, cos_inc_angle, object_idx, object_tag = point
            x, y = self.transform_to_relative_world_pos(leftwards, backwards)
            point_count += 1
            if object_tag in self.ignored_indices:
                continue
            object_tag = self.TAGS[object_tag]
            if object_idx == 0:
                continue
                # if object_tag not in static_objects:
                #     static_objects[object_tag] = []
                # static_objects[object_tag].append((x, y, z))
            else:
                object_idx = str(object_idx)
                d = np.sqrt(x ** 2 + y ** 2)
                if d < self.MIN_DIST_IS_SELF:
                    continue
                if object_idx not in objects:
                    objects[object_idx] = {"objects": [], "tag": object_tag}
                    # objects[object_idx] = {"xyd": [], "x": [], "y": [], "d": [], "tag": object_tag}
                for idx, objects_with_idx in enumerate(objects[object_idx]["objects"]):
                    rand_idx = 0
                    _x, _y = objects_with_idx["x"][rand_idx], objects_with_idx["y"][rand_idx]
                    _d = np.sqrt((x - _x) ** 2 + (y - _y) ** 2)
                    if _d < self.MIN_OBJ_DIFF_DIST:
                        objects[object_idx]["objects"][idx]["x"].append(x)
                        objects[object_idx]["objects"][idx]["y"].append(y)
                        objects[object_idx]["objects"][idx]["d"].append(d)
                        break
                else:
                    objects[object_idx]["objects"].append({"x": [x], "y": [y], "d": [d]})
        objects_min_max = {}
        for obj_idx, obj_and_tag in objects.items():
            tag = obj_and_tag["tag"]
            obj: dict
            for i, obj in enumerate(obj_and_tag["objects"]):
                a_max_x, a_max_y = np.argmax(obj["x"]), np.argmax(obj["y"])
                a_min_x, a_min_y = np.argmin(obj["x"]), np.argmin(obj["y"])
                a_min_d = np.argmin(obj["d"])
                d_min = obj["d"][a_min_d]
                poi = [
                    (obj["x"][a_max_x], obj["y"][a_max_x], obj["d"][a_max_x]),
                    (obj["x"][a_min_x], obj["y"][a_min_x], obj["d"][a_min_x]),
                    (obj["x"][a_max_y], obj["y"][a_max_y], obj["d"][a_max_y]),
                    (obj["x"][a_min_y], obj["y"][a_min_y], obj["d"][a_min_y]),
                ]

                poi = sorted(poi, key=lambda _x: np.sqrt(_x[0] ** 2 + _x[1] ** 2), reverse=True)
                bound_1, bound_2 = poi[:2]
                closest = np.array((obj["x"][a_min_d], obj["y"][a_min_d]))
                bound_1 = np.array(bound_1[:2]) / bound_1[2] * d_min + self.xy_position
                bound_2 = np.array(bound_2[:2]) / bound_2[2] * d_min + self.xy_position
                # bound_2 = np.array(bound_2[:2]) + self.xy_position
                # bound_1 = np.array(bound_1[:2]) + self.xy_position
                closest += self.xy_position
                poi = [bound_1, bound_2, closest]
                if tag not in objects_min_max:
                    objects_min_max[tag] = {}
                objects_min_max[tag][f"{obj_idx}_{i}"] = poi
        rospy.logwarn_throttle(10, f"fps={1 / (perf_counter() - t0)}")
        self.publish_dynamic_object_information(objects_min_max)

    def publish_dynamic_object_information(self, objects_min_max):
        header = Header()
        header.stamp = rospy.Time.now()
        for tag, values in objects_min_max.items():
            obstacles = PafObstacleList()
            obstacles.type = tag
            obstacles.header = header
            obstacles.obstacles = []
            for i, (bound_1, bound_2, closest) in values.items():
                obstacle = PafObstacle()
                obstacle.bound_1 = bound_1
                obstacle.bound_2 = bound_2
                obstacle.closest = closest
                obstacles.obstacles.append(obstacle)
            self._obstacle_publisher.publish(obstacles)

    @staticmethod
    def start(rate=100):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    node = SemanticLidarNode()
    node.start()
