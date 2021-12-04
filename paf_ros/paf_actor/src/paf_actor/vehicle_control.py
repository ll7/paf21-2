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
from paf_messages.msg import LocalPath

from paf_actor.spline import calc_spline_course


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
        self._route: LocalPath = LocalPath()
        self._target_speed: float = target_speed
        self._is_reverse: bool = False
        # TODO remove this (handled by the local planner)
        self._last_point_reached = False
        self._is_highspeed_mode = False

        # speed controller parameters
        args_longitudinal = {"K_P": 0.25, "K_D": 0.0, "K_I": 0.1}
        # distance control parameters
        args_dist = {"K_P": 0.2, "K_D": 0.0, "K_I": 0.01}
        # Stanley control parameters
        args_lateral = {"k": 2.5, "Kp": 1.0, "L": 2.9,
                        "max_steer": 30.0, "min_speed": 0.1}

        self._lon_controller: PIDLongitudinalController = PIDLongitudinalController(
            **args_longitudinal)
        self._lat_controller: StanleyLateralController = StanleyLateralController(
            **args_lateral)
        self._dist_controller: PIDLongitudinalController = PIDLongitudinalController(
            **args_dist)
        self._last_control_time: float = rospy.get_time()

        self._odometry_subscriber: rospy.Subscriber = rospy.Subscriber(
            f"/carla/{role_name}/odometry", Odometry, self.__odometry_updated
        )
        self.vehicle_control_publisher: rospy.Publisher = rospy.Publisher(
            f"/carla/{role_name}/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1
        )

        self.local_path_publisher: rospy.Publisher = rospy.Publisher(
            f"/local_planner/{role_name}/local_path", LocalPath, queue_size=1
        )

        self.local_path_subscriber: rospy.Subscriber = rospy.Subscriber(
            f"/local_planner/{role_name}/local_path", LocalPath, self.__local_path_received
        )

        self.__init_test_szenario()

    def __run_step(self):
        """
        This function should be called periodically to compute steering, throttle and brake commands

        Returns:
            carla_msgs_.msg.CarlaEgoVehicleControl: Ego Vehicle Control command
        """
        self._last_control_time, dt = self.__calculate_current_time_and_delta_time()

        self.__init_test_szenario()

        throttle: float = self.__calculate_throttle(dt)
        steering: float = self.__calculate_steering()
        control: CarlaEgoVehicleControl = self.__generate_control_message(
            throttle, steering)

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
        control.reverse = self._is_reverse
        return control

    def __calculate_throttle(self, dt: float) -> float:
        """
        Calculate the throttle for the vehicle

        Args:
            dt (float): the timedifference on the last step

        Returns:
            float: the throttle to use
        """
        # perform pid control step with distance and speed controllers

        lon: float = self._lon_controller.run_step(
            self._target_speed, self._current_speed, dt)
        # rospy.loginfo(
        #    f"Target_speed {self._target_speed}; Lon {lon}; Current_speed {self._current_speed}")

        # use whichever controller yields the lowest throttle
        return lon

    def __calculate_steering(self) -> float:
        """
        Calculate the steering angle with the Stanley Controller

        Returns:
            float: The steering angle to steer
        """
        # calculate steer
        # self._current_speed)
        return self._lat_controller.run_step(self._route, self._current_pose, self._current_speed, self._is_reverse)

    def __local_path_received(self, local_path: LocalPath) -> None:
        """
        Updates the local path and target speed based on the message argument.

        Args:
            local_path (LocalPath): The new local path from the local planner.
        """
        self._route = local_path
        self._is_reverse = local_path.target_speed < 0.0
        self._target_speed = abs(local_path.target_speed)

    def __init_test_szenario(self) -> None:
        """
        Generate a test_szenrio to debug this class
        Also sets the target_speed.

        Returns:
            Path: The path to folow
        """
        # TODO: Remove this. Used for validation
        if self._is_highspeed_mode:
            positions = [[-80, -20.5], [-80, -40.5], [-80, -60.5], [-80, -80.5], [-80, -100.5],
                         [-80, -115.5], [-80, -135.5], [-80, -150.0], [-70, -170.0], [-70.0, -190.0], [-70.0, -195.0], [-70.0, -200.0]]
            speeds = [-500.0, -100.0]
        else:
            positions = [[-84, 15.25], [-84, 20.25], [-100, 30]]
            speeds = [-30.0, 30.0]

        path_msg: LocalPath = LocalPath()

        if not self._last_point_reached:
            path_msg.target_speed = speeds[0]
        else:
            path_msg.target_speed = speeds[1]
            if self._is_highspeed_mode:
                positions = [[-80, -170.0], [-80.0, 200.0]]
            else:
                positions = [[-87, 0], [-90, 0], [-92, 0],
                             [-97, 0], [-120, 0], [-140, 0], [-200, 0]]

        ax = [x[0] for x in positions]
        ay = [x[1] for x in positions]
        cx, cy, cyaw, ck, s = calc_spline_course(
            ax, ay, ds=0.1)

        positions = [[x, y] for x, y in zip(cx, cy)]

        for point in positions:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)
        self.local_path_publisher.publish(path_msg)

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
            math.sqrt(odo.twist.twist.linear.x ** 2 +
                      odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2)
            * 3.6
        )

        self._current_pose = odo.pose.pose

        current_pos = [odo.pose.pose.position.x, odo.pose.pose.position.y]

        if self._is_highspeed_mode:
            last_position = [-80.0, -150.0]
        else:
            last_position = [-84, 15.25]
        distance = math.sqrt(
            (current_pos[0] - last_position[0]) ** 2 +
            (current_pos[1] - last_position[1]) ** 2
        )

        # rospy.loginfo(f"current_pos: {current_pos}; distance {distance}")

        if distance < 5.0:
            self._last_point_reached = True

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
