#!/usr/bin/env python

import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from paf_messages.msg import PafLocalPath


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    def __init__(self, _role_name):

        self.role_name = rospy.get_param("~role_name", "ego_vehicle")

        # mocked path - later replace by a calculated path
        self.path_array = [
            [199.0, 9.5],
            [210.0, 9.5],
            [219.0, 9.5],
            [224.4, 9.9],
            [227.8, 12.3],
            [230.1, 15.7],
            [231.0, 20.2],
            [231.1, 27.6],
            [231.2, 34.7],
        ]

        self._current_pose = Pose()
        self._current_speed = 0
        self._target_speed = 180

        # subscribe to carla odometry
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_updated)

        # create and start the publisher for the local path
        self.local_plan_publisher = rospy.Publisher(rospy.get_param("local_path_topic"), PafLocalPath, queue_size=1)

    def get_current_path(self):
        """returns the current local path starting with the nearest point on the path to the vehicle's position

        Returns:
            [type]: [description]
        """
        # print(f"closest point in path:{self.closest_point_in_path()}")
        starting_id = self.path_array.index(self.closest_point_in_path())
        current_path = self.path_array[starting_id:]

        return current_path

    def closest_point_in_path(self):
        """returns the closest point in the local path to the vehicle's current position

        Returns:
            [type]: [description]
        """
        # initialize with a high value
        min_distance = 1e10
        current_pos = [self._current_pose.position.x, self._current_pose.position.y]
        nearest_point = None
        for point in self.path_array:
            distance_x = point[0] - current_pos[0]
            distance_y = point[1] - current_pos[1]

            distance = math.sqrt(distance_x * distance_x + distance_y * distance_y)

            if distance < min_distance:
                min_distance = distance
                nearest_point = point

        return nearest_point

    def create_ros_msg(self):
        """create path message for ros

        Returns:
            [type]: [description]
        """
        path_msg = PafLocalPath()
        path_msg.header.frame_id = "local path"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.target_speed = self._target_speed
        current_path = self.get_current_path()

        for point in current_path:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0
            path_msg.poses.append(pose)

        return path_msg

    def publish_local_path(self):
        """ros publisher functionality"""

        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            path_msg = self.create_ros_msg()
            # next_waypoint = path_msg.poses[0].position
            # rospy.loginfo(f"current position {self._current_pose.position}, next waypoint: {next_waypoint}")
            self.local_plan_publisher.publish(path_msg)
            rate.sleep()

    def odometry_updated(self, odometry: Odometry):
        """Odometry Update Callback"""

        # calculate current speed (m/s) from twist
        self._current_speed = math.sqrt(
            odometry.twist.twist.linear.x ** 2 + odometry.twist.twist.linear.y ** 2 + odometry.twist.twist.linear.z ** 2
        )
        self._current_pose = odometry.pose.pose
        # invert y-coordinate of odometry, because odometry sensor returns wrong values
        self._current_pose.position.y = -self._current_pose.position.y

    def start(self):
        rospy.init_node("local_path_publisher", anonymous=True)
        self.publish_local_path()


if __name__ == "__main__":
    LocalPlanner().start()
