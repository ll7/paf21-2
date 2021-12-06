#!/usr/bin/env python
from time import perf_counter
import math
import numpy as np
import matplotlib.pyplot as plt
import rospy


from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Pose, PoseStamped
from paf_perception.msg import PafObstacleList, PafObstacle

# import functions to read xml file +CommonRoad Drivability Checker
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_dc.pycrcc as pycrcc

file_path = "/home/imech154/paf21-2/paf_ros/paf_test_Sebi/CR_Test.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

class ObstacleDetectionNode(object):
    """
    detects potential obstacles in front of the car (collision) along the way
    """
    ROADWAY_WIDTH = 2  # car width = 1.85m + buffer
    # minimum distance the car is able to react to an obstacle in front
    MIN_DISTANCE_TO_OBSTACLE = 3
    LOG_FPS_SECS = 60
    

    def __init__(self):

        self.xy_position = None
        self.z_orientation = None
        self.current_pose = Pose()  # car_position
        self.target_position = None
        self.obstacle_list = []
        self.risk = False

    
    def _process_obstacle_detection(self):

        car_position = self.current_pose
        target_position = self.target_position
        obstaclelist = self.obstacle_list
        risk_of_collision = self._check_roadway(car_position, target_position, obstaclelist)
        self.risk = risk_of_collision

    def _check_roadway(self, car_position, target_position, obstaclelist):
        """
        detects potential obstacles in front of the car (collision) along the way
        Step1: 
        Step2:
        """
        danger_zone = self._define_danger_zone(car_position, target_position)
        observated_obstacles = self._check_obstacles_in_danger_zone(
            obstaclelist, danger_zone)
        risk_of_collision = self._check_risk_of_collision(
            self, observated_obstacles)
        return risk_of_collision

    def _define_danger_zone(self, car_position, target_position):
        length_vector = np.substract(car_position, target_position)
        v1 = length_vector * np.array([-1, 0])
        v2 = length_vector * np.array([0, -1])
        width_vector1 = self.ROADWAY_WIDTH * self._unit_vector(v1)
        width_vector2 = self.ROADWAY_WIDTH * self._unit_vector(v2)
        corner_point3 = length_vector + width_vector1
        corner_point4 = length_vector + width_vector2
        corner_points = np.array[(
            width_vector1, width_vector2, corner_point3, corner_point4)]
        return corner_points

    def _check_obstacles_in_danger_zone(self, obstaclelist, danger_zone):
        observated_obstacles = []
        x1, y1 = np.minimum(danger_zone[0], danger_zone[3])
        x2, y2 = np.maximum(danger_zone[0], danger_zone[3])
        for obstacle in obstaclelist:
            x, y, _ = obstacle
            poi = (x, y)
            if x1 < x < x2 and y1 < y < y2:
                observated_obstacles.append(poi)
        return observated_obstacles

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
            rospy.loginfo(f"boolean is Obstacle detected {self.risk}")
            self.detected_obstacle_publisher.publish(self.risk)
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

