#!/usr/bin/env python
from nav_msgs.msg import Odometry

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np


class SemanticLidarNode(object):
    TAGS = {
        0: "Unlabeled",
        1: "Building",
        2: "Fence",
        3: "Other",
        4: "Pedestrians",
        5: "Pole",
        6: "RoadLine",
        7: "Road",
        8: "SideWalk",
        9: "Vegetation",
        10: "Vehicles",
        11: "Wall",
        12: "TrafficSign",
        13: "Sky",
        14: "Ground",
        15: "Bridge",
        16: "RailTrack",
        17: "GuardRail",
        18: "TrafficLight",
        19: "Static",
        20: "Dynamic",
        21: "Water",
        22: "Terrain",
    }

    MIN_DIST = 2.4

    def __init__(self):
        rospy.init_node("semantic_lidar", anonymous=True)
        topic1 = "/carla/ego_vehicle/semantic_lidar/lidar1/point_cloud"
        topic2 = "/carla/ego_vehicle/odometry"
        self._lidar_subscriber = rospy.Subscriber(topic1, PointCloud2, self.process_lidar_semantic)
        self._odometry_subcriber = rospy.Subscriber(topic2, Odometry, self.process_odometry)
        self.ignored_indices = [0, 6, 7] + list(range(13, 23))
        self.position = None

    def process_odometry(self, msg: Odometry):
        self.position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def process_lidar_semantic(self, msg: PointCloud2):
        if self.position is None:
            return
        points = pc2.read_points(msg, skip_nans=True)
        point_count = 0
        objects = {}
        # static_objects = {}
        for point in points:
            x, y, z, cos_inc_angle, object_idx, object_tag = point
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
                d = np.sqrt(x ** 2 + y ** 2)
                if d < self.MIN_DIST:
                    continue
                if object_idx not in objects:
                    objects[object_idx] = {"x": [], "y": [], "d": [], "tag": object_tag}
                objects[object_idx]["x"].append(x)
                objects[object_idx]["y"].append(y)
                objects[object_idx]["d"].append(d)
        objects_min_max = {}
        for k, obj in objects.items():
            a_max_x, a_max_y = np.argmax(obj["x"]), np.argmax(obj["y"])
            a_min_x, a_min_y = np.argmin(obj["x"]), np.argmin(obj["y"])
            a_min_d = np.argmin(obj["d"])
            d_min = obj["d"][a_min_d]
            # rospy.logwarn(np.max(obj["d"]))
            poi = [
                (obj["x"][a_max_x], obj["y"][a_max_x], obj["d"][a_max_x]),
                (obj["x"][a_min_x], obj["y"][a_min_x], obj["d"][a_min_x]),
                (obj["x"][a_max_y], obj["y"][a_max_y], obj["d"][a_max_y]),
                (obj["x"][a_min_y], obj["y"][a_min_y], obj["d"][a_min_y]),
            ]

            poi = sorted(poi, key=lambda _x: np.sqrt(_x[0] ** 2 + _x[1] ** 2), reverse=True)
            bound_1, bound_2 = poi[:2]
            bound_1 = np.array(bound_1[:2]) / bound_1[2] * d_min + self.position
            bound_2 = np.array(bound_2[:2]) / bound_2[2] * d_min + self.position
            closest = np.array((obj["x"][a_min_d], obj["y"][a_min_d])) + self.position
            poi = (tuple(bound_1), tuple(bound_2), tuple(closest))
            objects_min_max[f"{obj['tag']}_{k}"] = poi

        self.publish_dynamic_object_information(objects_min_max)

    def publish_dynamic_object_information(self, objects_min_max):
        for k, v in objects_min_max.items():
            rospy.logwarn(f"{k}: {v}")
        pass  # todo

    def start(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.logwarn("testetse")
            rate.sleep()


if __name__ == "__main__":
    node = SemanticLidarNode()
    node.start()
