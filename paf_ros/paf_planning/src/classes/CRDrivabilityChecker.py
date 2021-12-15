#!/usr/bin/env python
import math
import numpy as np
import rospy


from nav_msgs.msg import Odometry

# from paf_perception.msg import PafObstacleList, PafObstacle
from paf_messages.msg import PafObstacleList, PafObstacle

# import functions to read xml file +CommonRoad Drivability Checker
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape
from commonroad.scenario.obstacle import Obstacle, StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State

from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile


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

        self._current_pose = odo.pose.pose
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
        position = [self._current_pose.position.x, self._current_pose.position.y]
        orientation = 0

        initial_state = State(position=position, velocity=0, orientation=orientation, time_step=0)

        self.ego_vehicle = StaticObstacle(id, type, shape, initial_state)
        self.scenario.add_objects(self.ego_vehicle)

    def _add_all_pedestrians_to_cr(self, pedestrians: PafObstacleList):
        for obstacle in pedestrians:
            id = self.scenario.generate_object_id()
            self._add_pedestrian(id, obstacle)

    def _add_all_vehicles_to_cr(self, vehicles: PafObstacleList):
        for obstacle in vehicles:
            id = self.scenario.generate_object_id()
            self._add_vehicle(id, obstacle)

    def _add_obstacle(self, id, obstacle: PafObstacle, shape: Shape, type: ObstacleType, position: np.ndarray):
        initial_state = State(position=np.array([0.0, 0.0]), velocity=0, orientation=0, time_step=0)
        obstacle = StaticObstacle(id, type, shape, initial_state)
        self.scenario.add_objects(obstacle)

    def _add_pedestrian(self, id, obstacle: PafObstacle):
        shape = Circle(0.35, np.array(obstacle.closest))
        position = np.array(obstacle.closest)
        type = ObstacleType.PEDESTRIAN
        self._add_obstacle(id, obstacle, shape, type, position)
        self.cr_obstacles_pedestrians.append(obstacle)

    def _add_vehicle(self, id, obstacle: PafObstacle):
        vertices = np.array([obstacle.closest, obstacle.bound_1, obstacle.bound_2])
        shape = Polygon(vertices=vertices)
        # actual position should be fixed, dummy implementation
        position = np.array(obstacle.closest)
        type = ObstacleType.PARKED_VEHICLE
        self._add_obstacle(id, obstacle, shape, type, position)
        self.cr_obstacles_vehicles.append(obstacle)

    # def _update_obstacles_to_CRScenario(self):

    def _overwrite_file(self):
        author = ""
        affiliation = ""
        source = ""
        tags = {}

        # write new scenario
        fw = CommonRoadFileWriter(self.scenario, self.planning_problem_set, author, affiliation, source, tags)
        fw.write_to_file("/home/imech154/paf21-2/maps/Rules/Town03_modnew4.xml", OverwriteExistingFile.ALWAYS)

    def start(self):
        rospy.init_node("CRDrivabilityChecker", anonymous=True)


if __name__ == "__main__":
    checker = CRDriveabilityChecker()
    checker.start()
    checker._overwrite_file()
