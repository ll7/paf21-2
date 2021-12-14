#!/usr/bin/env python
from time import perf_counter
import math
import numpy as np
import matplotlib.pyplot as plt
import rospy


from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
# from paf_perception.msg import PafObstacleList, PafObstacle
from paf_messages.msg import PafObstacleList, PafObstacle
from tf.transformations import euler_from_quaternion
# import functions to read xml file +CommonRoad Drivability Checker
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_dc.pycrcc as pycrcc
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State

from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile


# generate path of the file to be opened
file_path = "/home/imech154/paf21-2/maps/Rules/Town03.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()


class CRDriveabilityChecker(object):
    def __init__(self):

        role_name = rospy.get_param("~role_name", "ego_vehicle")
        odometry_topic = f"/carla/{role_name}/odometry"
        obstacle_topic = rospy.get_param("obstacles_topic")

        self.detected_obstacle = None
        self.obstacles_pedestrians = None
        self.obstacles_vehicles = None

        scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

        self.ego_vehice_id = scenario.generate_object_id()
        self.ego_vehicle = None

        rospy.logwarn(odometry_topic)
        rospy.logwarn(obstacle_topic)
        rospy.Subscriber(odometry_topic, Odometry, self._odometry_updated)
        rospy.Subscriber(obstacle_topic, PafObstacleList,
                         self._obstacle_list_updated)
        


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
            #update pose of ego vehicle in cr-scenario 
            self._update_ego_vehicle_to_CRScenario()
            self._overwrite_file()
        

    def _obstacle_list_updated(self, msg: PafObstacleList):
        # add all obstacle from obstacle list to commonroad-scenario
        """
            Update obstacle mask
            :param msg: msg from Obstacle topic
            """
        if msg.type == "Pedestrians":
            self.obstacles_pedestrians = self._update_obstacles(msg)
            self._add_all_pedestrians_to_cr(self.obstacles_pedestrians)
        elif msg.type == "Vehicles":
            self.obstacles_vehicles = self._update_obstacles(msg)
            self._add_all_vehicles_to_cr(self.obstacles_vehicles)
        else:
            rospy.logwarn_once(f"obstacle type '{msg.type}' is unknown to top_down_view node")

        


    def _update_ego_vehicle_to_CRScenario(self):
        """create a new ego vehicel graphical representation
        """
        if (self.ego_vehicle is not None):
            scenario.remove_obstacle(self.ego_vehicle)

        id = self.ego_vehice_id
        scenario.remove_obstacle
        type = ObstacleType.PARKED_VEHICLE
        shape = Rectangle(width = 2.0, length = 4.5)
        position = [self._current_pose.position.x, self._current_pose.position.y]
        orientation = 0

        initial_state = State(position = position,
                                       velocity = 0,
                                       orientation = orientation,
                                       
                                       time_step = 0)
        
        self.ego_vehicle = StaticObstacle(id, type, shape, initial_state)
        scenario.add_objects(self.ego_vehicle)

    def _add_all_pedestrians_to_cr(self, pedestrians: PafObstacleList):
        for obstacle in pedestrians:
            id = scenario.generate_object_id()
            self._add_pedestrian(id, obstacle)

    def _add_all_vehicles_to_cr(self, vehicles: PafObstacleList):
        for obstacle in vehicles:
            id = scenario.generate_object_id()
            self._add_vehicle(id, obstacle)



    def _add_obstactle(self, id, obstacle: PafObstacle, shape: Shape, type: ObstacleType) -> int:
        orientation = 0
        #actual position should be fixed, dummy implementation
        position = np.array[obstacle.closest]
        initial_state = State(position = position,
                                       velocity = 0,
                                       orientation = orientation,                                      
                                       time_step = 0)
        
        obstacle = StaticObstacle(id, type, shape, initial_state)
        scenario.add_objects(obstacle)
        return id


    def _add_pedestrian(self,id, obstacle: PafObstacle):
        shape = Polygon(np.array[obstacle.closest, obstacle.bound_1, obstacle.bound_2])
        type = ObstacleType.PEDESTRIAN
        self._add_obstactle(self, id, obstacle, shape, type)

    def _add_vehicle(self,id, obstacle: PafObstacle):
        shape = Polygon(np.array[obstacle.closest, obstacle.bound_1, obstacle.bound_2])
        type = ObstacleType.PARKED_VEHICLE
        self._add_obstactle(self, id, obstacle, shape, type)

        

    #def _update_obstacles_to_CRScenario(self):

    def _overwrite_file(self):
        author = ''
        affiliation = ''
        source = ''
        tags = {}

        # write new scenario
        fw = CommonRoadFileWriter(scenario, planning_problem_set, author, affiliation, source, tags)
        fw.write_to_file("/home/imech154/paf21-2/maps/Rules/Town03_mod.xml", OverwriteExistingFile.ALWAYS)


    def start(self):
        rospy.init_node("CRDrivabilityChecker", anonymous=True)


if __name__ == "__main__":
    checker = CRDriveabilityChecker()
    checker.start()
        

    


