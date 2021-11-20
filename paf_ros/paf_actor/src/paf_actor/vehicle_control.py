#!/usr/bin/env python

from collections import deque
import math
import numpy as np
import rospy

import random
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose, PoseStamped
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Float32

from paf_actor.pid_control import PIDLongitudinalController
from paf_actor.stanley_control import StanleyLateralController


class VehicleController(object):
    """
    Combination of all Controllers that perform lateral and longitudinal control on the ego vehicle.
    Currently PID controllers are used for distance and speed control
    A Stanley Controller is used for steering control
    """

    def __init__(self, role_name, target_speed):

        self._current_speed = 0.0  # Km/h
        self._current_pose = Pose()
        self._route = Path()
        self._target_speed = target_speed
        self._current_distance = 0
        self._target_distance = 10

        # speed controller parameters
        args_longitudinal = {'K_P': 0.25, 'K_D': 0.0, 'K_I': 0.1}
        # distance control parameters
        args_dist = {'K_P': 0.2, 'K_D': 0.0, 'K_I': 0.01}
        # Stanley control parameters
        args_lateral = {'k': 2.5, 'Kp': 1.0, 'L': 2.9,
                        'max_steer': 30.0, 'min_speed': 0.1}

        self._lon_controller = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller = StanleyLateralController(**args_lateral)
        self._dist_controller = PIDLongitudinalController(**args_dist)
        self._last_control_time = rospy.get_time()

#        self._route_subscriber = rospy.Subscriber(
#            f"/psaf/{role_name}/local_path", Path, self.path_updated)
#        self._target_speed_subscriber = rospy.Subscriber(
#            f"/psaf/{role_name}/target_speed", Float64, self.target_speed_updated)
        self._odometry_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/odometry", Odometry, self.odometry_updated)
#        self.radarsubscriber = rospy.Subscriber(
#            f"psaf/{role_name}/radar/distance", Float64, self.radar_updated)
        self.vehicle_control_publisher = rospy.Publisher(
            f"/carla/{role_name}/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1)

    def run_step(self):
        """
        This function should be called periodically to compute steering, throttle and brake commands

        Returns:
            carla_msgs_.msg.CarlaEgoVehicleControl: Ego Vehicle Control command
        """
        # allow for variable stepsize
        current_time = rospy.get_time()
        dt = current_time-self._last_control_time
        if dt == 0.0:
            dt = 0.000001

        # TODO: Remove this. Used for validation
        self._current_speed = 0
        self._current_distance = 5000
        self._target_speed = 180

        path = np.array([[-84, -136]])
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        self._route = path_msg

        # compute safety distance
        min_dist = 4
        if self._current_speed > min_dist*2:
            self._target_distance = self._current_speed * .55
        else:
            self._target_distance = min_dist

        # perform pid control step with distance and speed controllers
        lon = self._lon_controller.run_step(
            self._target_speed, self._current_speed, dt)
        dist = - \
            self._dist_controller.run_step(
                self._target_distance, self._current_distance, dt)

        rospy.loginfo(f"SPPED: {lon}, {dist}, ")  # {self._route}")

        # use whichever controller yields the lowest throttle
        if lon < dist:
            throttle = lon
        else:
            throttle = dist

        self._last_control_time = current_time

        # calculate steer
        steering = self._lat_controller.run_step(
            self._route, self._current_pose, self._current_speed)

        # piece together the control message
        control = CarlaEgoVehicleControl()

        # positive throttle outputs are directed to the throttle, negative ones to the brakes
        if throttle >= 0.0:
            control.throttle = np.clip(throttle, 0.0, 1.0)
            control.brake = 0.0
        else:
            control.brake = -np.clip(throttle, -1.0, 0.0)
            control.throttle = 0.0
        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False

        return control

    def odometry_updated(self, odo):
        """Odometry Update Callback
        """
        # calculate current speed (km/h) from twist
        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                        odo.twist.twist.linear.y ** 2 +
                                        odo.twist.twist.linear.z ** 2) * 3.6
        self._current_pose = odo.pose.pose

    def radar_updated(self, msg):
        """Radar Update Callback
        """
        self._current_distance = msg.data

    def target_speed_updated(self, target_speed):
        """
        callback on new target speed
        """
        rospy.loginfo(
            "New target speed received: {}".format(target_speed.data))
        self._target_speed = target_speed.data

    def path_updated(self, path):
        """
        callback on new route
        """
        rospy.loginfo(
            "New plan with {} waypoints received.".format(len(path.poses)))
        self._route = path

    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(10)
        # periodically run lateral and longitudinal control
        while not rospy.is_shutdown():
            control = self.run_step()
            if control:
                control.steer = -control.steer
                self.vehicle_control_publisher.publish(control)
            else:
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass


def main():
    rospy.init_node('vehicle_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    target_speed = rospy.get_param("~target_speed", 0)
    controller = VehicleController(role_name, target_speed)
    try:
        controller.run()
    finally:
        del controller
    rospy.loginfo("Done")


if __name__ == "__main__":
    main()
