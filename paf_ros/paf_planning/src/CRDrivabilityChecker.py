#!/usr/bin/env python


import math
from typing import List

import numpy as np
import rospy

# import functions to read xml file +CommonRoad Drivability Checker
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, Obstacle, ObstacleType
from commonroad.scenario.trajectory import State, Trajectory
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (
    create_collision_checker,
    create_collision_object,
)
from commonroad_dc.collision.trajectory_queries.trajectory_queries import trajectories_collision_dynamic_obstacles
from nav_msgs.msg import Odometry

# from paf_perception.msg import PafObstacleList, PafObstacle
from paf_messages.msg import PafObstacle, PafObstacleList

# for orientation of the ego vehicle
from tf.transformations import euler_from_quaternion

# from commonroad_dc.collision.trajectory_queries


# generate path of the file to be opened
file_path = "/home/imech154/paf21-2/maps/Rules/Town03.xml"
save_path = "/home/imech154/paf21-2/maps/debug/Town03_modnew1.xml"
EGO_VEHICLE_SHAPE = Rectangle(width=2.0, length=4.5)
LOOK_AHEAD_STEPS = 10

PERCEPTS_PER_COMPUTATION = 10

POLYGON_POINT_OFFSET = [1, 1]


class CRDriveabilityChecker(object):
    def __init__(self):

        role_name = rospy.get_param("~role_name", "ego_vehicle")
        odometry_topic = f"/carla/{role_name}/odometry"
        obstacle_topic = rospy.get_param("obstacles_topic")

        self.counter = 1
        self.paf_obstacles_pedestrians = []
        self.paf_obstacles_vehicles = []

        self.ego_vehicle = None
        self.cr_obstacles_pedestrians = []
        self.cr_obstacles_vehicles = []

        self._collsion_objects_vehicles = []
        self.current_pose = None
        self.drive_direction = None

        rospy.logwarn(odometry_topic)
        rospy.logwarn(obstacle_topic)
        rospy.Subscriber(odometry_topic, Odometry, self._odometry_updated)
        rospy.Subscriber(obstacle_topic, PafObstacleList, self._obstacle_list_updated)

        # read in the scenario and planning problem set
        self.scenario, self.planning_problem_set = CommonRoadFileReader(file_path).open()

        self.ego_vehicle_id = self.scenario.generate_object_id()

        # collision checker
        self.collision_checker = create_collision_checker(self.scenario)

    def _odometry_updated(self, odo: Odometry):
        """
        Odometry update Callback
        Args:
            odo (Odometry): The Odometry
        """
        # calculate current speed (km/h) from twist
        self._current_speed = math.sqrt(
            odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2
        )

        if self.current_pose is not None:
            last_position = self.current_pose.position
            # pose update
            self.current_pose = odo.pose.pose
            self.drive_direction = np.array(
                [self.current_pose.position.x - last_position.x, self.current_pose.position.y - last_position.y]
            )
            length = math.sqrt(
                self.drive_direction[0] * self.drive_direction[0] + self.drive_direction[1] * self.drive_direction[1]
            )
            if length != 0:
                self.drive_direction = self.drive_direction / length
        else:
            self.drive_direction = np.array([0, 0])
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
        if self.counter % PERCEPTS_PER_COMPUTATION == 0:
            self._update_obstacles(msg)
            self.counter = 0
        self.counter += 1

    def _update_obstacles(self, msg: PafObstacleList):
        # clear obstacles
        paf_obstacle_list = msg.obstacles
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
            for paf_obstacle in paf_obstacle_list:
                id = (
                    paf_obstacle.id + 1000
                )  # add 3 zeros to identify obstacle and avoid clashes with ids from cr-scenario
                index = self._obstacle_contained_in_cr_scenario(id, self.cr_obstacles_vehicles)

                if index != -1:
                    # obstacle already contained
                    points = np.array([paf_obstacle.closest, paf_obstacle.bound_1, paf_obstacle.bound_2])
                    speed = paf_obstacle.speed
                    self._update_cr_obstacle(self.cr_obstacles_vehicles, index, points, speed)
                else:
                    self._add_vehicle(id, paf_obstacle)

            for cr_obstacle in self.cr_obstacles_vehicles:
                if (
                    self._obstacle_contained_in_old_obstacles(cr_obstacle.obstacle_id, self.paf_obstacles_vehicles)
                    != -1
                ):
                    self.cr_obstacles_vehicles.remove(cr_obstacle)
                    self.scenario.remove_obstacle(cr_obstacle)

        else:
            rospy.logwarn_once(f"obstacle type '{msg.type}' is unknown to node")

        self._overwrite_file()

        potential_collisions = self.check_collision(self.ego_vehicle.prediction)
        if potential_collisions.__contains__(-1):
            rospy.loginfo(f"no incoming collision detected {potential_collisions}")
        else:
            self.on_predicted_collision(potential_collisions[0])

    def _obstacle_contained_in_cr_scenario(self, obstacle_id, obstacle_list: List):
        for obstacle in obstacle_list:
            if obstacle_id == obstacle.obstacle_id:
                return obstacle_list.index(obstacle)
        return -1

    def _obstacle_contained_in_old_obstacles(self, obstacle_id, obstacle_list: PafObstacleList):
        for obstacle in obstacle_list:
            if obstacle_id == obstacle.id:
                return obstacle_list.index(obstacle)
        return -1

    def _update_cr_obstacle(self, list: List, index_in_list, points, speed):
        cr_obstacle = list[index_in_list]
        cr_obstacle.initial_state = State(position=np.array([0.0, 0.0]), velocity=speed, orientation=0, time_step=0)

        # add dummy implementation, obstacles should have three different vertices + visibility
        new_vertices = points + np.array(
            [[0.0, 0.0], [POLYGON_POINT_OFFSET[0], 0.0], [0.0000, POLYGON_POINT_OFFSET[1]]]
        )
        cr_obstacle.obstacle_shape = Polygon(vertices=new_vertices)

    def _update_ego_vehicle_to_CRScenario(self):
        """create a new ego vehicel graphical representation"""
        if self.ego_vehicle is not None:
            self.scenario.remove_obstacle(self.ego_vehicle)

        id = self.ego_vehicle_id
        type = ObstacleType.PARKED_VEHICLE
        shape = EGO_VEHICLE_SHAPE
        position = [self.current_pose.position.x, self.current_pose.position.y]
        orientation = self.current_orientation

        speed = self._current_speed
        drive_direction = self.drive_direction

        initial_state = State(position=position, velocity=speed, orientation=orientation, time_step=0)
        predicted_trajectory = self._generate_trajectory_prediction_ego_vehicle(
            drive_direction, speed, initial_state, EGO_VEHICLE_SHAPE
        )

        self.ego_vehicle = DynamicObstacle(id, type, shape, initial_state, predicted_trajectory)
        self.scenario.add_objects(self.ego_vehicle)

    def _add_all_pedestrians_to_cr(self, pedestrians: PafObstacleList):
        for obstacle in pedestrians:
            id = self.scenario.generate_object_id()
            self._add_pedestrian(id, obstacle)

    def _add_all_vehicles_to_cr(self, vehicles: PafObstacleList):
        for obstacle in vehicles:
            id = obstacle.id * 1000  # add 3 zeros to identify obstacle and avoid clashes with ids from cr-scenario
            self._add_vehicle(id, obstacle)

    def _create_obstacle(
        self, id, obstacle: PafObstacle, shape: Shape, type: ObstacleType, position: np.ndarray
    ) -> Obstacle:
        initial_state = State(position=np.array([0.0, 0.0]), velocity=obstacle.speed, orientation=0, time_step=0)

        predicted_trajectory = self._generate_trajectory_prediction_obstacle(
            initial_state, obstacle.velocity_vector, obstacle.speed, shape
        )

        dynamic_obstacle = DynamicObstacle(id, type, shape, initial_state, predicted_trajectory)
        return dynamic_obstacle

    def _generate_trajectory_prediction_ego_vehicle(
        self, drive_direction, speed, intialState: State, shape: Shape
    ) -> TrajectoryPrediction:
        state_list = []
        for i in range(1, LOOK_AHEAD_STEPS):
            # compute new position
            new_position = np.array(
                [
                    intialState.position[0] + self.scenario.dt * i * drive_direction[0] * speed,
                    intialState.position[1] + self.scenario.dt * i * drive_direction[1] * speed,
                ]
            )
            # create new state
            new_state = State(position=new_position, velocity=speed, orientation=self.current_orientation, time_step=i)
            # add new state to state_list
            state_list.append(new_state)
        dynamic_obstacle_trajectory = Trajectory(1, state_list)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, shape)
        return dynamic_obstacle_prediction

    def _generate_trajectory_prediction_obstacle(
        self, initial_state: State, velocity_vector, speed, shape: Shape
    ) -> TrajectoryPrediction:
        state_list = []

        for i in range(1, LOOK_AHEAD_STEPS):
            # compute new position
            new_position = np.array(
                [
                    initial_state.position[0] + self.scenario.dt * i * velocity_vector[0] * speed,
                    initial_state.position[1] + self.scenario.dt * i * velocity_vector[1] * speed,
                ]
            )

            new_state = State(position=new_position, velocity=speed, orientation=0.02, time_step=i)
            # add new state to state_list
            state_list.append(new_state)
        dynamic_obstacle_trajectory = Trajectory(1, state_list)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, shape)
        return dynamic_obstacle_prediction

    def _add_pedestrian(self, id, obstacle: PafObstacle):
        shape = Circle(0.35, np.array(obstacle.closest))
        position = np.array(obstacle.closest)
        type = ObstacleType.PEDESTRIAN
        cr_obstacle = self._create_obstacle(id, obstacle, shape, type, position)
        self.scenario.add_objects(cr_obstacle)
        self.cr_obstacles_pedestrians.append(cr_obstacle)

    def _add_vehicle(self, id, obstacle: PafObstacle):
        vertices = np.array([obstacle.closest, obstacle.bound_1, obstacle.bound_2])
        # add dummy implementation, obstacles should have three different vertices
        new_vertices = vertices + np.array(
            [[0.0, 0.0], [POLYGON_POINT_OFFSET[0], 0.0], [0.0000, POLYGON_POINT_OFFSET[1]]]
        )
        shape = Polygon(vertices=new_vertices)
        # actual position should be fixed, dummy implementation
        position = np.array(obstacle.closest)
        type = ObstacleType.PARKED_VEHICLE

        cr_obstacle = self._create_obstacle(id, obstacle, shape, type, position)

        # create collision object
        collision_object = create_collision_object(cr_obstacle)
        self._collsion_objects_vehicles.append(collision_object)

        self.scenario.add_objects(cr_obstacle)
        self.cr_obstacles_vehicles.append(cr_obstacle)

    def check_collision(self, predicted_trajectory: Trajectory) -> list:

        # create a collision object using the trajectory prediction of the ego vehicle
        collision_trajectory = create_collision_object(predicted_trajectory)
        collision_vehicles = self._collsion_objects_vehicles

        # cc = self.collision_checker.collide(co)
        cc = trajectories_collision_dynamic_obstacles([collision_trajectory], collision_vehicles, method="fcl")
        return cc

    def on_predicted_collision(self, timestamp):
        rospy.logwarn(
            f"""Incoming collision between the trajectory of the ego vehicle and objects in the environment at,
        timestamp: {timestamp}: """
        )

    def _overwrite_file(self):
        author = ""
        affiliation = ""
        source = ""
        tags = {}

        # write new scenario
        fw = CommonRoadFileWriter(self.scenario, self.planning_problem_set, author, affiliation, source, tags)
        fw.write_to_file(save_path, OverwriteExistingFile.ALWAYS)

    def start(self):
        rospy.init_node("CRDrivabilityChecker", anonymous=True)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    @staticmethod
    def orientation_vector_to_angle(vector) -> int:
        return np.arctan(vector[0] / vector[1])


if __name__ == "__main__":
    checker = CRDriveabilityChecker()
    checker.start()
