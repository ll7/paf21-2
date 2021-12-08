#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLocalPath, Point2D as Point




LOOK_AHEAD_POINTS = 10

class LocalPlanner:

    """class used for the local planner. Task: return a local path"""

    def __init__(self):

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
            [228.5, 47.7],
            [224.3,52.2],
            [220.8,53.6],
            [217.0,54.3],
            [199.8, 56.5],
            [179.2, 57.2],
            [139.3, 56.8],
            [120.6,56.5],

            [107.7,59.1],
            [97.1,56,9],
            [88.0,52.2],
            [78.4,48.8],
            [72.3,49],
            [66.8,52],
            [64.2,62.6],
            [68.9,69.7],
            [76.6,72.1],
            [82.3,71.7],
            [86.7, 69.6],
            [93.6,64.2],
            [100.3,62.6],
            [119.6, 62.3],
            [137.6,61.9],
            [160, 62.3],
            [178.9,62.3],
            [199.5, 62.3],
            [217.5,62.3],
            [228.3,61.9],
            [235.6,59],
            [243.5,49.4],
            [241.6,36.4],
            [242.3,13.2],
            [237.8,2.2],
            [223.2,-4.7],
            [206.7,-5.7],
            [187.4,-5.5],
            [162.8,-5.7],
            [141.2,-6],
            [120.6,-5.7],
            [100.3,-5.7],
            [80.3,-6],
            [60.4,-6],
            [39.4,-5.7],
            [29.6, -7.5],
            [19.6,-12.2],
            [11.6,-19.3],
            [0.5,-22.6],
            [-10.4,-20],
            [-19.1, -12.8],
            [-23.4,-1.8],
            [-22.4,7.5],
            [-16.9,16.7],
            [-10.7,21.2],
            [5.6,22.3],
            [13.8,18.1],
            [20.8,10.9],
            [28,7.1],
            [40.7,6.4],
            [60.0,7.7],
            [79.7, 8],
            [100.3, 7.4],
            [119.6, 8.4],
            [139.6, 8.7],
            [160,8.7],
            [180,8.4]

        ]

        # next Point in path
        self.currentPointIndex = 0

        self._current_pose = Pose()
        self._current_speed = 0
        self._target_speed = 180

        # subscribe to carla odometry
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_updated)

        # create and start the publisher for the local path
        self.local_plan_publisher = rospy.Publisher(rospy.get_param("local_path_topic"), PafLocalPath, queue_size=1)



    def get_current_path(self) -> Path:
        """returns the current local path starting with the nearest point on the path to the vehicle's position

        Returns:
            [type]: [description]
        """
        print(f"closest point in path:{self.closest_point_in_path()}")
        starting_id = self.currentPointIndex
        if(starting_id + LOOK_AHEAD_POINTS < len(self.path_array)-1):
            current_path = self.path_array[starting_id:starting_id+LOOK_AHEAD_POINTS]
        else:
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

    

    def create_ros_msg(self) -> Path:
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
            pt = Point()
            pt.x = point[0]
            pt.y = point[1]
            path_msg.points.append(pt)
            path_msg.target_speed = 30

        

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
