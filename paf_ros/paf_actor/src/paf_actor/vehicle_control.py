#!/usr/bin/env python

import math
from typing import Tuple
import numpy as np
import rospy

from geometry_msgs.msg import Pose
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from paf_actor.pid_control import PIDLongitudinalController
from paf_actor.stanley_control import StanleyLateralController
from paf_messages.msg import PafLocalPath, PafLogScalar


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
        self._route: PafLocalPath = PafLocalPath()
        self._target_speed: float = target_speed
        self._is_reverse: bool = False
        self._emergency_mode: bool = False
        # TODO remove this (handled by the local planner)
        self._last_point_reached = False

        self._start_time = None
        self._end_time = None

        # speed controller parameters
        args_longitudinal = {"K_P": 0.25, "K_D": 0.0, "K_I": 0.1}
        self._target_speed_offset = 1.2
        # Stanley control parameters
        args_lateral = {"k": 2.5, "Kp": 1.0, "L": 2, "max_steer": 30.0, "min_speed": 0.1}

        self._lon_controller: PIDLongitudinalController = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller: StanleyLateralController = StanleyLateralController(**args_lateral)
        self._last_control_time: float = rospy.get_time()

        self._odometry_subscriber: rospy.Subscriber = rospy.Subscriber(
            f"/carla/{role_name}/odometry", Odometry, self.__odometry_updated, queue_size=1
        )
        self.vehicle_control_publisher: rospy.Publisher = rospy.Publisher(
            f"/carla/{role_name}/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1
        )

        self.local_path_subscriber: rospy.Subscriber = rospy.Subscriber(
            "/paf/paf_local_planner/path", PafLocalPath, self.__local_path_received, queue_size=1
        )

        self.emergy_break_publisher: rospy.Publisher = rospy.Publisher(
            f"/local_planner/{role_name}/emergency_break", Bool, queue_size=1
        )

        self.local_path_subscriber: rospy.Subscriber = rospy.Subscriber(
            f"/local_planner/{role_name}/emergency_break", Bool, self.__emergency_break_received
        )

        self.speed_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self.target_speed_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self.target_speed_error_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self.steering_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self.throttle_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        # self.__init_test_szenario()

    def __run_step(self):
        """
        This function should be called periodically to compute steering, throttle and brake commands

        Returns:
            carla_msgs_.msg.CarlaEgoVehicleControl: Ego Vehicle Control command
        """
        self._last_control_time, dt = self.__calculate_current_time_and_delta_time()

        # self.__init_test_szenario()
        try:
            steering, self._target_speed, distance = self.__calculate_steering()
            self._is_reverse = self._target_speed < 0.0
            self._target_speed = abs(self._target_speed)

            throttle: float = self.__calculate_throttle(dt, distance)
        except RuntimeError:
            throttle = -1.0
            steering = 0.0
            self._is_reverse = False
            self._target_speed = 0.0
            rospy.loginfo_throttle(10, "[Actor] waiting for new local path")

        control: CarlaEgoVehicleControl = self.__generate_control_message(throttle, steering)

        msg = PafLogScalar()
        msg.section = "ACTOR speed error"
        msg.value = (self._target_speed - self._current_speed) * 3.6

        self.target_speed_error_log_publisher.publish(msg)

        msg = PafLogScalar()
        msg.section = "ACTOR speed"
        msg.value = self._current_speed * 3.6

        self.speed_log_publisher.publish(msg)

        msg = PafLogScalar()
        msg.section = "ACTOR target_speed"
        msg.value = self._target_speed * 3.6
        msg.step_as_distance = False

        self.target_speed_log_publisher.publish(msg)

        msg = PafLogScalar()
        msg.section = "ACTOR steering"
        msg.value = np.rad2deg(steering)

        self.steering_log_publisher.publish(msg)

        msg = PafLogScalar()
        msg.section = "ACTOR throttle"
        msg.value = np.clip(throttle, -1, 1)

        self.throttle_log_publisher.publish(msg)

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

        if control.brake > 0 and self._current_speed > 5:
            control.reverse = not self._is_reverse
            control.throttle = control.brake

        if self._emergency_mode:
            control.hand_brake = True  # True
            control.steer = np.rad2deg(30.0)
            control.brake = 0.0
            control.throttle = 1.0
            control.reverse = not self._is_reverse
        return control

    def __calculate_throttle(self, dt: float, distance: float) -> float:
        """
        Calculate the throttle for the vehicle

        Args:
            dt (float): the timedifference on the last step

        Returns:
            float: the throttle to use
        """
        # perform pid control step with distance and speed controllers
        target_speed = self._target_speed * self._target_speed_offset

        if distance >= 0.5 and self._current_speed > 10:
            target_speed = max(10, self._current_speed * (1 - 1e-8))

        lon: float = self._lon_controller.run_step(target_speed, self._current_speed, dt)

        return lon

    def __calculate_steering(self) -> float:
        """
        Calculate the steering angle with the Stanley Controller

        Returns:
            float: The steering angle to steer
        """
        return self._lat_controller.run_step(self._route, self._current_pose, self._current_speed, self._is_reverse)

    def __local_path_received(self, local_path: PafLocalPath) -> None:
        """
        Updates the local path and target speed based on the message argument.

        Args:
            local_path (PafLocalPath): The new local path from the local planner.
        """
        # rospy.loginfo(
        #    f"INHALT VON LOCAL_PATH with speed {local_path.target_speed}")
        self._route = local_path

    def __emergency_break_received(self, do_emergency_break: bool):
        """
        Listens to emergency break signals

        Args:
            do_emergency_break (bool): True if an emergency break is needed
        """
        self._emergency_mode = do_emergency_break.data

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
        self._current_speed = math.sqrt(
            odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2
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
