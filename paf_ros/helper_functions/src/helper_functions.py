from collections import deque
import rospy
import math
import numpy as np
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Path, Odometry


def main():
    rospy.init_node('helper_functions', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    helper_functions = HelperFunctions(role_name)

if __name__ == '__main__':    
    main()

def difference_between_points(point1, point2) -> Point:
    difference = Point()
    difference.x = point1.x - point2.x
    difference.y = point1.y - point2.y
    difference.z = point1.z - point2.z
    return difference
        
def distance_between_points(point1, point2) -> float:
    return ((point2.x - point1.x) ** 2 + (point2.y - point1.y) ** 2 + (point2.z - point1.z) ** 2) ** 0.5
    
def dot_product(vector1, vector2):
    return sum((a * b) for a, b in zip(vector1, vector2))
    
def length_vector(vector):
    return math.sqrt(dot_product(vector, vector))
    
def angle_between_vector(vector1, vector2):
    math.acos(dot_product(vector1, vector2) / (length_vector(vector1) * length_vector(vector2)))

def from_global_target_to_local_target (target, current_position) -> Point:
    #getCarPose
    return difference_between_points(target, current_position)

def from_local_target_to_global_target (target, current_position) -> Point:
    global_target = Point()
    global_target.x = current_position.x + target.x
    global_target.y = current_position.y + target.y
    global_target.z = current_position.z + target.z
    return global_target

def next_point_on_path (path, point) -> Point:
    distance = -1.0
    nearest_point = Point()
    for point_on_path in path:
        if distance == -1.0:
            distance = self.distance_between_points(point_on_path.pose.position, point)
            nearest_point = ppoint_on_path.pose.position

        if distance > self.distance_between_points(point_on_path.pose.position, point):
            distance = self.distance_between_points(point_on_path.pose.position, point)
            nearest_point = point_on_path.pose.position
    return nearest_point
    
def calc_path_yaw(Path, idx):
        # computes yaw of a path at index idx
        if idx >= len(Path.poses) - 1:
            #this should probably throw an exception
            return 0

        point_current = Path.poses[idx].pose.position
        point_next = Path.poses[idx+1].pose.position
        angle = math.atan2(point_next.y - point_current.y, point_next.x - point_current.x)
        return normalize_angle(angle)

def calc_egocar_yaw(currentPose):
    # compute Ego Car Yaw
    quaternion = (
        currentPose.orientation.x,
        currentPose.orientation.y,
        currentPose.orientation.z,
        currentPose.orientation.w
            )
    _, _, yaw = euler_from_quaternion(quaternion)
    return normalize_angle(yaw)

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

class HelperFunctions:
    def __init__(self, role_name):
        self._odometry_subscriber = rospy.Subscriber("/carla/{}/odometry".format(role_name), Odometry, self.get_odometry)
        self._current_pose = Pose()

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()
            self.print_odometry()

    def get_odometry(self, odo):
        self._current_pose = odo.pose.pose


    def print_odometry(self):
        rospy.loginfo("Positionsänderung zu: {}".format(self._current_pose.position))    

