#!/usr/bin/env python


import math
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt
import numpy as np
from numpy.core.arrayprint import BoolFormat
# for orientation of the ego vehicle
from tf.transformations import euler_from_quaternion
import rospy
import time

from threading import Thread


from nav_msgs.msg import Odometry


# from paf_perception.msg import PafObstacleList, PafObstacle
from paf_messages.msg import PafObstacleList, PafObstacle

# import functions to read xml file +CommonRoad Drivability Checker
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape
from commonroad.scenario.obstacle import DynamicObstacle, Obstacle, StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State

from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile

from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

from commonroad.scenario.trajectory import State, Trajectory

from rospy.timer import Rate


# generate path of the file to be opened
file_path = "/home/imech154/paf21-2/maps/Rules/Town03.xml"
EGO_VEHICLE_SHAPE = Rectangle(width=2.0, length=4.5)

class CRDriveabilityChecker(object):
    def __init__(self):

        role_name = rospy.get_param("~role_name", "ego_vehicle")
        odometry_topic = f"/carla/{role_name}/odometry"
        obstacle_topic = rospy.get_param("obstacles_topic")

        self.paf_obstacles_pedestrians = None
        self.paf_obstacles_vehicles = None

        self.ego_vehicle = None
        self.cr_obstacles_pedestrians = []
        self.cr_obstacles_vehicles = []

        rospy.logwarn(odometry_topic)
        rospy.logwarn(obstacle_topic)
        rospy.Subscriber(odometry_topic, Odometry, self._odometry_updated)
        rospy.Subscriber(obstacle_topic, PafObstacleList,
                         self._obstacle_list_updated)

        # read in the scenario and planning problem set
        self.scenario, self.planning_problem_set = CommonRoadFileReader(
            file_path).open()

        self.ego_vehicle_id = self.scenario.generate_object_id()

        #collision checker
        self.collision_checker = create_collision_checker(self.scenario)


    def _odometry_updated(self, odo: Odometry):
        """
        Odometry update Callback
        Args:
            odo (Odometry): The Odometry
        """
        # calculate current speed (km/h) from twist
        self._current_speed = math.sqrt(
            odo.twist.twist.linear.x ** 2 +
            odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2
        )

        self.current_pose = odo.pose.pose

        # calculate current orientation in z axis (rad)
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.current_orientation = yaw

        # update pose of ego vehicle in cr-scenario
        self._update_ego_vehicle_to_CRScenario()


    def _obstacle_list_updated(self, msg: PafObstacleList):
        # clear obstacles

        # add all obstacle from obstacle list to commonroad-scenario
        """
        Update obstacle mask
        :param msg: msg from Obstacle topic
        """
        if msg.type == "Pedestrians":
            # clear pedestrians
            for cr_obstacle in self.cr_obstacles_pedestrians:
                self.scenario.remove_obstacle(cr_obstacle)
            self.cr_obstacles_pedestrians = []

            self.paf_obstacles_pedestrians = msg.obstacles
            self._add_all_pedestrians_to_cr(self.paf_obstacles_pedestrians)
        elif msg.type == "Vehicles":
            # clear vehicles
            for cr_obstacle in self.cr_obstacles_vehicles:
                    self.scenario.remove_obstacle(cr_obstacle)
            self.cr_obstacles_vehicles = []

            #add new obstacles
            self._add_all_vehicles_to_cr(msg.obstacles)
        else:
            rospy.logwarn_once(
                f"obstacle type '{msg.type}' is unknown to node")

        self._overwrite_file()

        if self.check_collision(self.ego_vehicle.prediction):
            self.on_predicted_collision()
        else:
            rospy.loginfo('no incoming collision detected')

    def _update_obstacles(self, msg: PafObstacleList) -> Obstacle:
        """get the obstacle information from the Obstacle topic without the type"""
        return msg.obstacles

    def _update_ego_vehicle_to_CRScenario(self):
        """create a new ego vehicel graphical representation"""
        if self.ego_vehicle is not None:
            self.scenario.remove_obstacle(self.ego_vehicle)

        id = self.ego_vehicle_id
        type = ObstacleType.PARKED_VEHICLE
        shape = EGO_VEHICLE_SHAPE
        position = [self.current_pose.position.x, self.current_pose.position.y]
        orientation = self.current_orientation

        #dummy velocity
        initial_state = State(position=position, velocity=20,
                              orientation=orientation, time_step=0)
        predicted_trajectory = self._generate_trajectory_prediction_ego_vehicle(initial_state, EGO_VEHICLE_SHAPE)

        self.ego_vehicle = DynamicObstacle(id, type, shape, initial_state, predicted_trajectory)
        self.scenario.add_objects(self.ego_vehicle)







    def _add_all_pedestrians_to_cr(self, pedestrians: PafObstacleList):
        for obstacle in pedestrians:
            id = self.scenario.generate_object_id()
            self._add_pedestrian(id, obstacle)

    def _add_all_vehicles_to_cr(self, vehicles: PafObstacleList):
        for obstacle in vehicles:
            id = self.scenario.generate_object_id()
            self._add_vehicle(id, obstacle)

    def _create_obstacle(
        self, id, obstacle: PafObstacle, shape: Shape, type: ObstacleType, position: np.ndarray
    ) -> Obstacle:
        initial_state = State(position=np.array(
            [0.0, 0.0]), velocity=0, orientation=0, time_step=0)

        predicted_trajectory = self._generate_trajectory_prediction_obstacle (initial_state, shape)

        dynamic_obstacle = DynamicObstacle(id, type, shape, initial_state, predicted_trajectory)
        return dynamic_obstacle

    def _generate_trajectory_prediction_ego_vehicle(self, intialState: State, shape: Shape)-> TrajectoryPrediction:
        state_list = []
        for i in range(1, 21):
            # compute new position
            new_position = np.array([intialState.position[0] + self.scenario.dt * i * 20, self.current_pose.position.y])
            # create new state
            new_state = State(position = new_position, velocity = 20,orientation = self.current_orientation, time_step = i)
            # add new state to state_list
            state_list.append(new_state)
        dynamic_obstacle_trajectory = Trajectory(1, state_list)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, shape)
        return dynamic_obstacle_prediction

    def _generate_trajectory_prediction_obstacle(self, initial_state:State, shape: Shape)-> TrajectoryPrediction:
        state_list = []
        for i in range(1, 21):
            # compute new position
            new_position = np.array([initial_state.position[0] + self.scenario.dt * i * 0, 0])
            # create new state
            new_state = State(position = new_position, velocity = 0, orientation = 0.02, time_step = i)
            # add new state to state_list
            state_list.append(new_state)
        dynamic_obstacle_trajectory = Trajectory(1, state_list)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, shape)
        return dynamic_obstacle_prediction


    def _add_pedestrian(self, id, obstacle: PafObstacle):
        shape = Circle(0.35, np.array(obstacle.closest))
        position = np.array(obstacle.closest)
        type = ObstacleType.PEDESTRIAN
        cr_obstacle = self._create_obstacle(
            id, obstacle, shape, type, position)
        self.scenario.add_objects(cr_obstacle)
        self.cr_obstacles_pedestrians.append(cr_obstacle)

    def _add_vehicle(self, id, obstacle: PafObstacle):
        vertices = np.array(
            [obstacle.closest, obstacle.bound_1, obstacle.bound_2])
        # add dummy implementation, obstacles should have three different vertices
        new_vertices = vertices + np.array([[0.0, 0.0], [0.00001, 0.0], [0.0000, 0.00001]])
        shape = Polygon(vertices=new_vertices)
        # actual position should be fixed, dummy implementation
        position = np.array(obstacle.closest)
        type = ObstacleType.PARKED_VEHICLE

        cr_obstacle = self._create_obstacle(
            id, obstacle, shape, type, position)

        # create collision object
        create_collision_object(cr_obstacle)

        self.scenario.add_objects(cr_obstacle)
        self.cr_obstacles_vehicles.append(cr_obstacle)

    def check_collision(self, predicted_trajectory: Trajectory)-> bool:

        # create a collision object using the trajectory prediction of the ego vehicle
        co = create_collision_object(predicted_trajectory)
        cc = self.collision_checker.collide(co)
        return cc

    def on_predicted_collision(self):
        rospy.logwarn('Incoming collision between the trajectory of the ego vehicle and objects in the environment: ')

    def _overwrite_file(self):
        author = ""
        affiliation = ""
        source = ""
        tags = {}

        # write new scenario
        fw = CommonRoadFileWriter(
            self.scenario, self.planning_problem_set, author, affiliation, source, tags)
        fw.write_to_file(
            "/home/imech154/paf21-2/maps/Rules/Town03_modnew8.xml", OverwriteExistingFile.ALWAYS)

    def start(self):
        rospy.init_node("CRDrivabilityChecker", anonymous=True)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()



if __name__ == "__main__":
    checker = CRDriveabilityChecker()
    checker.start()
