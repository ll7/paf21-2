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
from tf.transformations import euler_from_quaternion
# import functions to read xml file +CommonRoad Drivability Checker
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_dc.pycrcc as pycrcc

#file_path = "/home/imech154/paf21-2/paf_ros/paf_test_Sebi/CR_Test.xml"
file_path = "/home/imech154/paf21-2/paf_ros/paf_test_Sebi/Town03.xml"


# read in the scenario and the lanelet set
scenario, _ = CommonRoadFileReader(file_path).open()
lanelet_network = scenario.lanelet_network

# class ObstacleContainer:
#     def __init__(self):
#         self.pedestrians = []
#         self.vehicles = []

class ObstacleDetectionNode(object):
    """
    detects potential obstacles in front of the car (collision) along the way
    """
    ROADWAY_WIDTH = 2  # car width = 1.85m + buffer
    # minimum distance the car is able to react to an obstacle in front
    MIN_DISTANCE_TO_OBSTACLE = 50
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

        self._current_pose = Pose()  # car_position
        self.target_position = None 

        self.x_orientation = 0
        self.y_orientation = 0
        self.z_orientation = 0

        # self.obstacles = ObstacleContainer()

        self.detected_obstacle = None

    def _obstacle_list_updated(self, msg: PafObstacleList):
        """
        Callback for PAFObstacle topic
        :param bounds_by_tag: format { tag : [ [bound1, bound2, closest, speed], ...] , ...}
        """
        rospy.logwarn("Nachricht wird empfangen")

        # if msg.type == "Pedestrians":
        #     self.obstacles.pedestrians = msg.obstacles
        # elif msg.type == "Vehicles":
        #     self.obstacles.vehicles = msg.obstacles
        # else:
        #     rospy.logwarn_once(f"obstacle type '{msg.type}' is unknown to obstacle detection")
    
        self._process_obstacle_detection(
            msg.type,
            msg.obstacles
        )
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

        quaternion = (
            self._current_pose.orientation.x,
            self._current_pose.orientation.y,
            self._current_pose.orientation.z,
            self._current_pose.orientation.w,
        )
        _, _, yaw = euler_from_quaternion(quaternion)

        self.z_orientation = yaw
        # rotation of result in opposite direction to get world coords
        self.x_orientation = np.cos(-self.z_orientation)
        self.y_orientation = np.sin(-self.z_orientation)

        # TODO: Remove this
        self.target_position = self._current_pose
    
    def _process_obstacle_detection(self, tag,  obstacles):

        dangerous_obs_list = self._check_roadway(
            obstacles
        )
        
        if len(dangerous_obs_list) > 0:
            self._publish_dangerous_obstacle_list(tag, dangerous_obs_list)

        # self.detected_obstacle_publisher.publish(self.detected_obstacle)

    def _publish_dangerous_obstacle_list(self, tag, found_obstacles):
        header = Header()
        header.stamp = rospy.Time.now()

        obstacle_list = PafObstacleList()
        obstacle_list.type = tag
        obstacle_list.header = header
        obstacle_list.obstacles = found_obstacles

        self.detected_obstacle_publisher.publish(obstacle_list)

    def _check_roadway(self, obstacles):
        """
        detects potential obstacles in front of the car (collision) in the same lanelet
        Step1: 
        Step2:
        """
        lanelet_ID = self._define_danger_zone()
        observated_obstacles = self._check_obstacles_in_danger_zone(
            obstacles, lanelet_ID)
        min_dist = self.MIN_DISTANCE_TO_OBSTACLE
        obs_list = self._check_obstacles_in_range(observated_obstacles, min_dist)
        obs_list = self._check_obstacles_in_front(obs_list)
        
        return obs_list

    def _define_danger_zone(self):
        # cur_pos = np.array([
        #     self._current_pose.position.x,
        #     self._current_pose.position.y,
        # ])
        cur_pos = np.array([
            -50.0,
            0.0
        ])
        laneletId = (
            lanelet_network.find_lanelet_by_position([cur_pos]),
            # TODO: target_pos
            lanelet_network.find_lanelet_by_position([cur_pos])
        )
        return laneletId

    def _check_obstacles_in_danger_zone(self, obstacles, laneletID):
        observated_obstacles = []
        for obstacle in obstacles:
            observated_obstacles.append(obstacle)
            # x1, y1 = obstacle.bound_1
            # x2, y2 = obstacle.bound_2
            # x3, y3 = obstacle.closest

            # for x in lanelet_network.find_lanelet_by_position([
            #     np.array([x1, y1]),
            #     np.array([x2, y2]),
            #     np.array([x3, y3]),
            # ]):
            #     if laneletID[0] in x or laneletID[1] in x:
            #         observated_obstacles.append(obstacle)
            #         break
        return observated_obstacles

    def _check_obstacles_in_front(self, observated_obstacles):
        obs_list = []
        for obstacle in observated_obstacles:
            closest = list(obstacle.closest)
            car_x = self._current_pose.position.x
            car_y = self._current_pose.position.y
            vect_car_to_obstacle = np.array(closest) - np.array([car_x, car_y]) 
            orientation_vect = np.array([self.x_orientation, self.y_orientation])
            angle = self._angle(vect_car_to_obstacle, orientation_vect)
            if 1.57 > angle: #1,57rad -> 90degrees
                obs_list.append(obstacle)
                break
        return obs_list

    def _check_obstacles_in_range(self, observated_obstacles, min_dist):
        obs_list = []
        for obstacle in observated_obstacles:
            bound1 = obstacle.bound_1
            bound2 = obstacle.bound_2
            closest = obstacle.closest
            pos_list = np.array([bound1, bound2, closest])
            for xy in pos_list:
                car_x = self._current_pose.position.x
                car_y = self._current_pose.position.y

                dist = self._dist(xy, (car_x, car_y))
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


        #rospy.loginfo(f"this list of obstacles are in front {self.detected_obstacle}")
        #self.detected_obstacle_publisher.publish(self.detected_obstacle)
        #rate.sleep()

    @staticmethod
    def _unit_vector(p1: np.ndarray) -> float:
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

    def _angle(self, p1: np.ndarray, p2: np.ndarray) -> float:
        """
        Angle between two vectors (x,y,d)
        :param p1: vector 1
        :param p2: vector 2
        :return: angle in rads
        """
        v1_u = self._unit_vector(p1)
        v2_u = self._unit_vector(p2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    @staticmethod
    def start():
        """
        starts ROS node
        """
        rospy.spin()


if __name__ == "__main__":
    node = ObstacleDetectionNode()
    node.start()
