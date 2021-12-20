#!/usr/bin/env python


import math
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt
import numpy as np
#for orientation of the ego vehicle
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
        rospy.Subscriber(obstacle_topic, PafObstacleList, self._obstacle_list_updated)

        # read in the scenario and planning problem set
        self.scenario, self.planning_problem_set = CommonRoadFileReader(file_path).open()

        self.ego_vehicle_id = self.scenario.generate_object_id()

        self.done = 0

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

        self.current_pose = odo.pose.pose

        #calculate current orientation in z axis (rad)
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
        # self._overwrite_file()

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
            self.vehicles = []

            self.paf_obstacles_vehicles = msg.obstacles
            self._add_all_vehicles_to_cr(self.paf_obstacles_vehicles)
        else:
            rospy.logwarn_once(f"obstacle type '{msg.type}' is unknown to node")

        self._overwrite_file()

    def _update_obstacles(self, msg: PafObstacleList) -> Obstacle:
        """get the obstacle information from the Obstacle topic without the type"""
        return msg.obstacles

    def _update_ego_vehicle_to_CRScenario(self):
        """create a new ego vehicel graphical representation"""
        if self.ego_vehicle is not None:
            self.scenario.remove_obstacle(self.ego_vehicle)

        id = self.ego_vehicle_id
        type = ObstacleType.PARKED_VEHICLE
        shape = Rectangle(width=2.0, length=4.5)
        position = [self.current_pose.position.x, self.current_pose.position.y]
        orientation = self.current_orientation

        initial_state = State(position=position, velocity=5, orientation=orientation, time_step=0)

        self.ego_vehicle = DynamicObstacle(id, type, shape, initial_state)
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
        initial_state = State(position=np.array([0.0, 0.0]), velocity=0, orientation=0, time_step=0)
        return StaticObstacle(id, type, shape, initial_state)

    def _convert_obstacles_to_collision_objects(self, cr_obstacles):
        # convert each obstacle in the scenario to a collision object
        for obs in cr_obstacles:
            create_collision_object(obs)

    def _add_pedestrian(self, id, obstacle: PafObstacle):
        shape = Circle(0.35, np.array(obstacle.closest))
        position = np.array(obstacle.closest)
        type = ObstacleType.PEDESTRIAN
        cr_obstacle = self._create_obstacle(id, obstacle, shape, type, position)
        self.scenario.add_objects(cr_obstacle)
        self.cr_obstacles_pedestrians.append(cr_obstacle)

    def _add_vehicle(self, id, obstacle: PafObstacle):
        vertices = np.array([obstacle.closest, obstacle.bound_1, obstacle.bound_2])
        shape = Polygon(vertices=vertices)
        # actual position should be fixed, dummy implementation
        position = np.array(obstacle.closest)
        type = ObstacleType.PARKED_VEHICLE
        cr_obstacle = self._create_obstacle(id, obstacle, shape, type, position)

        # create collision object
        #create_collision_object(cr_obstacle)

        self.scenario.add_objects(cr_obstacle)
        self.cr_obstacles_vehicles.append(cr_obstacle)

    def _overwrite_file(self):
        author = ""
        affiliation = ""
        source = ""
        tags = {}

        # write new scenario
        fw = CommonRoadFileWriter(self.scenario, self.planning_problem_set, author, affiliation, source, tags)
        fw.write_to_file("/home/imech154/paf21-2/maps/Rules/Town03_modnew5.xml", OverwriteExistingFile.ALWAYS)

    def start(self):
        rospy.init_node("CRDrivabilityChecker", anonymous=True)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def main1(self):
        print("in main1")
        print(self.scenario)

        # time.sleep(10)
        # plt.figure(figsize=(25, 10))

        # self.scenario.draw(self.renderer)
        # self.planning_problem_set.draw(self.renderer)
        # self.renderer.render()

        # plt.show()

        # example trajectory

        # trajectory_list = []
        # start = [-9,207.5]
        # trajectory_list.append(start)
        # for i in range(0,20):
        #     trajectory_list.append([start[0]+i*5, start[1]])

        # #create collision checker
        # self.collision_checker = create_collision_checker(self.scenario)

        # state_list = list()
        # for k in range(0, len(trajectory_list)):
        #     state_list.append(State(**{'time_step':k,'position': trajectory_list[k], 'orientation': 0.0}))
        # trajectory = Trajectory(0, state_list)

        # # create the shape of the ego vehicle
        # shape = Rectangle(length=4.5, width=2.0)
        # # create a TrajectoryPrediction object consisting of the trajectory and the shape of the ego vehicle
        # traj_pred = TrajectoryPrediction(trajectory=trajectory, shape=shape)


if __name__ == "__main__":

    checker = CRDriveabilityChecker()

    checker.start()
