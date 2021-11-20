#!/usr/bin/env python

import math
from typing import Tuple
import numpy as np
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Path, Odometry

from paf_actor.pid_control import PIDLongitudinalController
from paf_actor.stanley_control import StanleyLateralController


class VehicleController:
    """
    Combination of all Controllers that perform lateral and longitudinal control on the ego vehicle.
    Currently PID controllers are used for distance and speed control
    A Stanley Controller is used for steering control
    """

    def __init__(self, role_name: str, target_speed: float):
        """
        The Vehicle Controller class constructor

        Args:
            role_name (str): vehicle name to subscribe to
            target_speed (float): initial target speed
        """

        self._current_speed: float = 0.0  # Km/h
        self._current_pose: Pose = Pose()
        self._route: Path = Path()
        self._target_speed: float = target_speed
        self._current_distance: float = 0
        self._target_distance: float = 10

        # speed controller parameters
        args_longitudinal = {"K_P": 0.25, "K_D": 0.0, "K_I": 0.1}
        # distance control parameters
        args_dist = {"K_P": 0.2, "K_D": 0.0, "K_I": 0.01}
        # Stanley control parameters
        args_lateral = {"k": 2.5, "Kp": 1.0, "L": 2.9, "max_steer": 30.0, "min_speed": 0.1}

        self._lon_controller: PIDLongitudinalController = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller: StanleyLateralController = StanleyLateralController(**args_lateral)
        self._dist_controller: PIDLongitudinalController = PIDLongitudinalController(**args_dist)
        self._last_control_time: float = rospy.get_time()

        self._odometry_subscriber: rospy.Subscriber = rospy.Subscriber(
            f"/carla/{role_name}/odometry", Odometry, self.__odometry_updated
        )
        self.vehicle_control_publisher: rospy.Publisher = rospy.Publisher(
            f"/carla/{role_name}/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1
        )

    def __run_step(self):
        """
        This function should be called periodically to compute steering, throttle and brake commands

        Returns:
            carla_msgs_.msg.CarlaEgoVehicleControl: Ego Vehicle Control command
        """
        self._last_control_time, dt = self.__calculate_current_time_and_delta_time()

        path_msg: Path = self.__init_test_szenario()

        self._route = path_msg

        self.__calculate_target_distance()

        throttle: float = self.__calculate_throttle(dt)
        steering: float = self.__calculate_steering()
        control: CarlaEgoVehicleControl = self.__generate_control_message(throttle, steering)

        return control

    def __generate_control_message(self, throttle: float, steering: float) -> CarlaEgoVehicleControl:
        """
        Generate a control message for the CarlaEgoVehicle

        Args:
            throttle (float): The throttle to communicate to the vehicle
            steering (float): The steering to communicate to the vehicle

        Returns:
            CarlaEgoVehicleControl: The control message
        """
        # piece together the control message
        control: CarlaEgoVehicleControl = CarlaEgoVehicleControl()

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

    def __calculate_target_distance(self):
        """
        compute the saftey distance
        """
        min_dist: float = 4
        if self._current_speed > min_dist * 2:
            self._target_distance = self._current_speed * 0.55
        else:
            self._target_distance = min_dist

    def __calculate_throttle(self, dt: float) -> float:
        """
        Calculate the throttle for the vehicle

        Args:
            dt (float): the timedifference on the last step

        Returns:
            float: the throttle to use
        """
        # perform pid control step with distance and speed controllers
        lon: float = self._lon_controller.run_step(self._target_speed, self._current_speed, dt)
        dist: float = -self._dist_controller.run_step(self._target_distance, self._current_distance, dt)

        # use whichever controller yields the lowest throttle
        return lon if lon < dist else dist

    def __calculate_steering(self) -> float:
        """
        Calculate the steering angle with the Stanley Controller

        Returns:
            float: The steering angle to steer
        """
        # calculate steer
        return self._lat_controller.run_step(self._route, self._current_pose, 0.0)  # self._current_speed)

    def __init_test_szenario(self) -> Path:
        """
        Generate a test_szenrio to debug this class
        Also sets the target_speed and current_distance (artifact)

        Returns:
            Path: The path to folow
        """
        # TODO: Remove this. Used for validation
        rospy.loginfo(f"Current speed: {self._current_speed}")
        self._current_distance = 5000
        self._target_speed = 30

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
        return path_msg

    def __calculate_current_time_and_delta_time(self) -> Tuple[float, float]:
        """
        Calculates the current time and the delta time to last timesteps

        Returns:
            Tuple[float, float]: current time and delta time (dt)
        """
        # allow for variable stepsize
        current_time: float = rospy.get_time()
        dt: float = current_time - self._last_control_time
        if dt == 0.0:
            dt = 0.000001
        return current_time, dt

    def __odometry_updated(self, odo: Odometry):
        """
        Odometry update Callback

        Args:
            odo (Odometry): The Odometry
        """
        # calculate current speed (km/h) from twist
        self._current_speed = (
            math.sqrt(odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2)
            * 3.6
        )
        self._current_pose = odo.pose.pose

    def run(self):
        """
        Control loop (throttle and steer)
        """
        r = rospy.Rate(10)
        # periodically run lateral and longitudinal control
        while not rospy.is_shutdown():
            control: CarlaEgoVehicleControl = self.__run_step()
            if control:
                control.steer = -control.steer
                self.vehicle_control_publisher.publish(control)
            else:
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass


def main():
    rospy.init_node("vehicle_control", anonymous=True)
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
