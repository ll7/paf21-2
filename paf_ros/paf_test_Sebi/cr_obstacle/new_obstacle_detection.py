#!/usr/bin/env python
from time import perf_counter
import math
import numpy as np
import matplotlib.pyplot as plt
import rospy


from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Pose, PoseStamped
# from paf_perception.msg import PafObstacleList, PafObstacle
from paf_messages.msg import PafObstacleList, PafObstacle
from argparse import Namespace

# import functions to read xml file +CommonRoad Drivability Checker
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_dc.pycrcc as pycrcc

file_path = "/home/imech154/paf21-2/paf_ros/paf_test_Sebi/CR_Test.xml"

# read in the scenario and the lanelet set
scenario, _ = CommonRoadFileReader(file_path).open()
lanelet_network = scenario.lanelet_network

class ObstacleContainer:
    def __init__(self):
        self.pedestrians = []
        self.vehicles = []


class ObstacleDetectionNode(object):
    """
    detects potential obstacles in front of the car (collision) along the way
    """
    ROADWAY_WIDTH = 2  # car width = 1.85m + buffer
    # minimum distance the car is able to react to an obstacle in front
    MIN_DISTANCE_TO_OBSTACLE = 3
    LOG_FPS_SECS = 60
    

    def __init__(self):

        rospy.init_node("obstacle_detection", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        odometry_topic = f"/carla/{role_name}/odometry"
        obstacle_topic = rospy.get_param("obstacles_topic")
        detected_obstacle_topic = f"/paf/paf_perception/detected_obstacles"
        rospy.logwarn(odometry_topic)
        rospy.logwarn(obstacle_topic)
        rospy.Subscriber(odometry_topic, Odometry, self._odometry_updated)
        rospy.Subscriber(obstacle_topic, PafObstacleList,
                         self._obstacle_list_updated)
        self.detected_obstacle_publisher = rospy.Publisher(
            detected_obstacle_topic,
            PafObstacleList,
            queue_size=1
        )

        self.current_pose = Pose()  # car_position
        self.target_position = None

        self.obstacles = ObstacleContainer()

        self.detected_obstacle = None

    def _obstacle_list_updated(self, msg: PafObstacleList):
        """
        Callback for PAFObstacle topic
        :param bounds_by_tag: format { tag : [ [bound1, bound2, closest, speed], ...] , ...}
        """
        rospy.logwarn("Nachricht wird empfangen")

        if msg.type == "Pedestrians":
            self.obstacles.pedestrians = msg.obstacles
        if msg.type == "Vehicles":
            self.obstacles.vehicles = msg.obstacles
        else:
            rospy.logwarn_once(f"obstacle type '{msg.type}' is unknown to obstacle detection")
    
        
        # if msg.type == "Pedestrians" or msg.type == "Vehicles":
        #     self.obstacles[msg.type] = msg.obstacles
        #     self._process_obstacle_detection()
        # else:
        #     rospy.logwarn_once(f"obstacle type '{msg.type}' is unknown to obstacle detection")

    @staticmethod
    def _update_obstacles(msg: PafObstacleList):
        obs: PafObstacle
        ret = []
        for obs in msg.obstacles:
            obs_pts = []
            for x, y in [obs.bound_1, obs.bound_2, obs.closest]:
                obs_pts.append(Namespace(**{"x": x, "y": y}))
            ret.append(obs_pts)
        return ret

    def _odometry_updated(self, odometry: Odometry):
        """
        Odometry topic callback
        :param msg: 
        """
        t0 = perf_counter()
        # calculate current speed (km/h) from twist
        self._current_speed = (
            math.sqrt(
                odometry.twist.twist.linear.x ** 2
                + odometry.twist.twist.linear.y ** 2
                + odometry.twist.twist.linear.z ** 2
            )
            * 3.6
        )
        self._current_pose = odometry.pose.pose
        # invert y-coordinate of odometry, because odometry sensor returns wrong values
        self._current_pose.position.y = -self._current_pose.position.y
    
    def _process_obstacle_detection(self, tag,  obstacle_list):

        car_position = self.current_pose
        target_position = self.target_position
        obstaclelist = self.obstacle_list
        dangerrous_obs_list = self._check_roadway(car_position, obstaclelist)
        self.detected_obstacle = dangerrous_obs_list
        return dangerrous_obs_list

    def _check_roadway(self, car_position, obstaclelist):
        """
        detects potential obstacles in front of the car (collision) in the same lanelet
        Step1: 
        Step2:
        """
        lanelet_ID = self._define_danger_zone(car_position)
        observated_obstacles = self._check_obstacles_in_danger_zone(
            obstaclelist, lanelet_ID)
        min_dist = self.MIN_DISTANCE_TO_OBSTACLE
        obs_list = self._check_obstacles_in_range(observated_obstacles, min_dist)
        return obs_list

    def _define_danger_zone(car_position, target_position):
        laneletId = lanelet_network.find_lanelet_by_position(car_position)
        return laneletId

    def _check_obstacles_in_danger_zone(self, obstaclelist, laneletID):
        observated_obstacles = []
        for obstacle in obstaclelist:
            bound1, bound2, closest= obstacle[:3]
            pos_list = np.array([bound1, bound2, closest])
            if laneletID in lanelet_network.find_lanelet_by_position(pos_list):
                observated_obstacles.append(obstacle)
        return observated_obstacles

    def _check_obstacles_in_range(self, observated_obstacles, min_dist):
        obs_list = []
        for obstacle in observated_obstacles:
            bound1, bound2, closest= obstacle[:3]
            pos_list = np.array([bound1, bound2, closest])
            for xy in pos_list:
                dist = self._dist(xy)
                if min_dist > dist:
                    obs_list.append(obstacle)
                    break
        return obs_list

    # def _check_obstacles_in_range(self, obstacleList, danger_zone):
    #     observated_obstacles = []
    #     for obstacle in obstaclelist:
    #         bound1, bound2, closest= obstacle[:3]
    #         pos_list = np.array([bound1, bound2, closest])
    #         for xy in pos_list:
    #             dist = self._dist(xy)
    #             if self.MIN_DISTANCE_TO_OBSTACLE > dist:
    #                 observated_obstacles.append(pos_list)
    #                 break
    #     return observated_obstacles

    def _check_risk_of_collision(self, observated_obstacles):
        risk_of_collision = False
        for obstacles in observated_obstacles:
            dist = self._dist(obstacles)
            if self.MIN_DISTANCE_TO_OBSTACLE > dist:
                risk_of_collision = True
                break
        return risk_of_collision

    def publish_detected_obstacle(self):
        """ros publisher functionality"""

        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            path_msg = self.create_ros_msg()
            rospy.loginfo(f"this list of obstacles are in front {self.detected_obstacle}")
            self.detected_obstacle_publisher.publish(self.detected_obstacle)
            rate.sleep()

    @staticmethod
    def _unit_vector(p1: list) -> float:
        """
        calculates the unit_vector 
        :param p1: vector 1 (x,y)
        :return: unit_vector
        """
        length = np.linalg.norm(p1)
        unit_vector = p1 / length
        return unit_vector

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
    node = ObstacleDetectionNode()
    node.start()
