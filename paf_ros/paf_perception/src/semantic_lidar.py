#!/usr/bin/env python
from time import perf_counter

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from paf_perception.msg import PafObstacleList, PafObstacle


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
        topic3 = "/paf/paf_perception/obstacles"
        self._lidar_subscriber = rospy.Subscriber(topic1, PointCloud2, self.process_lidar_semantic)
        self._odometry_subcriber = rospy.Subscriber(topic2, Odometry, self.process_odometry)
        self._obstacle_publisher = rospy.Publisher(topic3, PafObstacleList, queue_size=1)
        self.ignored_indices = [0, 6, 7] + list(range(13, 23))
        self.xy_position = None

    def process_odometry(self, msg: Odometry):
        self.xy_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def process_lidar_semantic(self, msg: PointCloud2):
        if self.xy_position is None:
            return
        t0 = perf_counter()
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
                    objects[object_idx] = {"xyd": [], "x": [], "y": [], "d": [], "tag": object_tag}
                for _x, _y, _ in objects[object_idx]["xyd"]:
                    _d = np.sqrt((x - _x) ** 2 + (y - _y) ** 2)
                    if _d > 3:
                        break
                else:
                    # objects[object_idx]["xyd"].append((x, y, d))
                    objects[object_idx]["x"].append(x)
                    objects[object_idx]["y"].append(y)
                    objects[object_idx]["d"].append(d)
        objects_min_max = {}
        for k, obj in objects.items():
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

            bound_1 = np.array(bound_1[:2]) / bound_1[2] * d_min + self.xy_position
            bound_2 = np.array(bound_2[:2]) / bound_2[2] * d_min + self.xy_position
            # bound_2 = np.array(bound_2[:2]) + self.xy_position
            # bound_1 = np.array(bound_1[:2]) + self.xy_position

            # sorted_obj = sorted(obj["xyd"], key=lambda _x: _x[2])
            # bounds = []
            # for point in sorted_obj[:3]:
            #     x, y, d = point
            #     bounds.append(np.array([x, y]) + self.xy_position)
            # while len(bounds) < 3:
            #     bounds.append(bounds[0])

            closest = np.array((obj["x"][a_min_d], obj["y"][a_min_d])) + self.xy_position
            bounds = [bound_1, bound_2, closest]

            poi = [x for x in bounds]
            if obj["tag"] not in objects_min_max:
                objects_min_max[obj["tag"]] = {}
            objects_min_max[obj["tag"]][k] = poi
        rospy.logwarn_throttle(10, f"fps={1/(perf_counter() - t0)}")
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
