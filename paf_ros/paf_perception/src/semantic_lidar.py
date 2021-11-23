#!/usr/bin/env python
# from time import perf_counter

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

    def __init__(self):
        rospy.init_node("semantic_lidar", anonymous=True)
        topic = "/carla/ego_vehicle/semantic_lidar/lidar1/point_cloud"
        self._lidar_subscriber = rospy.Subscriber(topic, PointCloud2, self.process_lidar_semantic)
        topic = "/carla/ego_vehicle/lidar/lidar2/point_cloud"
        self._lidar_subscriber2 = rospy.Subscriber(topic, PointCloud2, self.process_lidar_standard)
        self.ignored_indices = [0, 6, 7] + list(range(13, 23))

    def process_lidar_standard(self, msg: PointCloud2):
        points = pc2.read_points(msg)
        counter = 0
        for point in points:
            counter += 1
            d, r, dynamic, intensity = point
        print("aue")

    def process_lidar_semantic(self, msg: PointCloud2):
        points = pc2.read_points(msg, skip_nans=True)
        # t0 = perf_counter()
        point_count = 0
        objects = {}
        static_objects = {}
        for point in points:
            x, y, z, cos_inc_angle, object_idx, object_tag = point
            point_count += 1
            if object_tag in self.ignored_indices:
                continue
            object_tag = self.TAGS[object_tag]
            if object_idx == 0:
                if object_tag not in static_objects:
                    static_objects[object_tag] = []
                static_objects[object_tag].append((x, y, z))
            else:
                if object_idx not in objects:
                    objects[object_idx] = {"x": [], "y": [], "tag": object_tag}
                objects[object_idx]["x"].append(x)
                objects[object_idx]["y"].append(y)
        # t_perf = perf_counter() - t0
        # t_perf_per_point = t_perf / point_count
        # temp = [len(x["x"]) for x in objects.values()]
        objects_min_max = {}
        for k, obj in objects.items():
            objects_min_max[f"{obj['tag']}_{k}"] = {
                "max": (np.max(obj["x"]), np.max(obj["y"])),
                "min": (np.min(obj["x"]), np.min(obj["y"])),
            }

        pass

    def start(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.logwarn("testetse")
            rate.sleep()


if __name__ == "__main__":
    node = SemanticLidarNode()
    node.start()
