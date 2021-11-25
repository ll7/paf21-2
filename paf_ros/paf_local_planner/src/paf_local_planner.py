#!/usr/bin/env python

import rospy
import math
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped

   

class LocalPlanner:
    """ class used for the local planner. Task: return a local path 
    """
    def __init__(self, role_name):

        self.role_name = role_name
        
        # mocked path - later replace by a calculated path
        self.path_array = [[199.0, -9.5], [210.0, -9.5], [219.0, -9.5], [224.4, -9.9],[227.8, -12.3], [
            230.1, -15.7], [231.0, -20.2], [231.1, -27.6], [231.2, -34.7], ]
        
        self._current_pose = Pose()

        # subscribe to carla odometry
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_updated)

        # create and start the publisher for the local path
        self.local_plan_publisher = rospy.Publisher('local_path_publisher', Path, queue_size=10)
        

    def get_current_path(self):
        """returns the current local path starting with the nearest point on the path to the vehicle's position

        Returns:
            [type]: [description]
        """
        print(f'closest point in path:{self.closest_point_in_path()}' )
        starting_id = self.path_array.index(self.closest_point_in_path())
        path_length = self.path_array.__len__
        current_path = self.path_array[starting_id:]

        return current_path

    def closest_point_in_path(self):
        """returns the closest point in the local path to the vehicle's current position 

        Returns:
            [type]: [description]
        """
        # initialize with a high value
        min_distance = 99999999999
        current_pos = [self._current_pose.position.x, self._current_pose.position.y]
        nearest_point = None
        for point in self.path_array:
            distance_x = point[0] - current_pos[0]
            distance_y = point[1] - current_pos[1]

            distance = math.sqrt(distance_x * distance_x +
                                 distance_y * distance_y)

            # print(f'distance to point on path {point} is {distance}')
            if(distance < min_distance):
                min_distance = distance
                nearest_point = point

        return nearest_point

    def testing(self):
        print("oh lol")
        self._current_pose.position.x = 230
        self._current_pose.position.y = -16
        closestPoint = self.closest_point_in_path()
        print(
            f'closes point to currentPosition "  {self._current_pose.position} + " is: " + {closestPoint}')


    def create_ros_msg(self):
        """create path message for ros

        Returns:
            [type]: [description]
        """
        path_msg = Path()
        path_msg.header.frame_id = "local path"

        current_path = self.get_current_path()

        for point in current_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        return path_msg


    def publish_local_path(self):
        """ros publisher functionality
        """
        

        
        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            path_msg = self.create_ros_msg()
            next_waypoint = path_msg.poses[0].pose.position
            rospy.loginfo(f'current position {self._current_pose.position}, next waypoint: {next_waypoint}' )
            self.local_plan_publisher.publish(path_msg)
            rate.sleep()

    def odometry_updated(self, odometry):
        """Odometry Update Callback
        """

       
        # calculate current speed (km/h) from twist
        self._current_speed = math.sqrt(odometry.twist.twist.linear.x ** 2 +
                                        odometry.twist.twist.linear.y ** 2 +
                                        odometry.twist.twist.linear.z ** 2) * 3.6
        self._current_pose = odometry.pose.pose
        
       


if __name__ == '__main__':

    rospy.init_node('local_path_publisher', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    lp = LocalPlanner(role_name)

    lp.publish_local_path()
    rospy.spin()
    print("starting")
    lp.testing()
