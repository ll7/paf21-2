#!/usr/bin/env python
"""
A Python file that represents the controller of the Ego Vehicle in Carla
"""
import math
from typing import Dict, Tuple
import numpy as np
import rospy

from geometry_msgs.msg import Pose
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from paf_actor.pid_control import PIDLongitudinalController
from paf_actor.stanley_control import StanleyLateralController
from paf_messages.msg import PafLocalPath, PafLogScalar, PafObstacleFollowInfo


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
        # km/h
        self._current_speed: float = 0.0
        # pose of the ego vehicle
        self._current_pose: Pose = Pose()
        # the route to follow, given by the planner
        self._route: PafLocalPath = PafLocalPath()
        # the speed the vehicle should drive with
        self._target_speed: float = target_speed
        # indicates if we drive backwards
        self._is_reverse: bool = False
        # indicates if we should go into panic mode
        self._emergency_mode: bool = False

        # timespan until the actor recognizes a stuck situation
        self._stuck_check_time: float = 2.0  # (s)
        # speed threshold which is considered stuck
        self._stuck_value_threshold: float = 1.0  # (m/s)
        # time when the car got stuck
        self._stuck_start_time: float = 0.0  # (s)
        # time when the unstuck operation started (a.k.a. rear gear)
        self._unstuck_start_time: float = 0.0  # (s)
        # max duration for the rear gear
        self._unstuck_check_time: float = 0.5  # (s)
        # true while the car is driving backwards to unstuck
        self._is_unstucking: bool = False

        # the deadlock timer gets used to check if a deadlock
        # occured and how to resolve that deadlock with different stages

        # a parameter that save the start of the deadlock timer
        self._deadlock_start_time: float = 0.0  # (s)
        # seconds after which a deadlock happened
        self._deadlock_stuck_check_time: float = 80.0  # (s)
        # a bool to indicate if we are in the deadlock resolve loop
        self._deadlock_is_unlocking: bool = False
        # a timer that check at which state we currently are
        self._deadlock_timer: float = 0.0  # (s)
        # the first phase (driving backwards)
        self._deadlock_phase_1: float = 1.0  # (s)
        # the second phase (standing_still)
        self._deadlock_phase_2: float = 2.0  # (s)
        # the third pahse (driving_forwards)
        self._deadlock_phase_3: float = 6.0  # (s)
        # the end phase, return to normal
        self._deadlock_phase_end: float = 7.0  # (s)

        # the speed to follow an obstacle in front of us
        self._obstacle_follow_speed: float = float("inf")
        # the distance to keep to an obstacle in front of us
        self._obstacle_follow_min_distance: float = 6.0  # (m)
        # the distance when an obstacle should be followed
        self._obstacle_follow_target_distance: float = 15.0  # (m)
        # indicates if we follow an obstacle
        self._obstacle_follow_active: bool = False
        # the speed to add to the obstacle speed that we follow
        self._obstalce_approaching_speed: float = 2  # (m/s)

        # the speed to drive a U-Turn
        self._u_turn_speed = 5.0  # (m/s)
        # the threshold when a u_turn is before us
        self._u_turn_heading_error_threshold = 0.8

        # speed controller parameters
        args_longitudinal: Dict[str, float] = {"K_P": 0.25, "K_D": 0.0, "K_I": 0.1}
        # an offset to bring the PID controller to the correct spped
        self._target_speed_offset: float = 1.2

        # Stanley control parameters
        args_lateral: Dict[str, float] = {"k": 2.5, "Kp": 1.0, "L": 2, "max_steer": 30.0, "min_speed": 0.1}

        # the longitude controller
        self._lon_controller: PIDLongitudinalController = PIDLongitudinalController(**args_longitudinal)
        # the lateral controller
        self._lat_controller: StanleyLateralController = StanleyLateralController(**args_lateral)

        # timer to keep track of last control command
        self._last_control_time: float = rospy.get_time()

        # Subscriber to the Odometry
        rospy.Subscriber(f"/carla/{role_name}/odometry", Odometry, self.__odometry_updated, queue_size=1)

        # the publisher of vehicle control messages
        self._vehicle_control_publisher: rospy.Publisher = rospy.Publisher(
            f"/carla/{role_name}/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1
        )

        # the subscriber to the PafLocalPath
        rospy.Subscriber("/paf/paf_local_planner/path", PafLocalPath, self.__local_path_received, queue_size=1)

        # the subscriber to the emergy_break message
        rospy.Subscriber(f"/local_planner/{role_name}/emergency_break", Bool, self.__emergency_break_received)

        # the subscriber to the PafObstacleFollowInfo message
        rospy.Subscriber("/paf/paf_obstacle_planner/following_info", PafObstacleFollowInfo, self.__handle_obstacle_msg)

        # these publishers to the tensorboard, are used for debugging
        self._speed_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self._target_speed_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self._target_speed_error_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self._steering_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

        self._throttle_log_publisher: rospy.Publisher = rospy.Publisher(
            "/paf/paf_validation/tensorboard/scalar", PafLogScalar, queue_size=1
        )

    def __run_step(self) -> CarlaEgoVehicleControl:
        """
        This function should be called periodically to compute steering, throttle and brake commands

        Returns:
            carla_msgs_.msg.CarlaEgoVehicleControl: Ego Vehicle Control command
        """
        throttle: float = 0.0
        steering: float = 0.0

        # check which control loop to follow
        if not self._deadlock_is_unlocking:
            # calculate time differences for the PID Controller
            self._last_control_time, dt = self.__calculate_current_time_and_delta_time()

            # Handle possible exception when there is no path to follow
            try:
                # get steering and target_speed values
                steering, self._target_speed, distance = self.__calculate_steering()
                # negative target speeds indicate that we want to drive backwards
                self._is_reverse = self._target_speed < 0.0
                # we need to have a positive target speed, even if we drive backwards
                self._target_speed = abs(self._target_speed)

                # only follow a obstacle if we are driving forwards and faster than the obstacle
                if not self._is_reverse:
                    if self._target_speed > self._obstacle_follow_speed:
                        self._target_speed = self._obstacle_follow_speed
                        if self._obstacle_follow_speed < 0:  # if we are to close we just want to reverse
                            steering = 0.0

                # check if we the heading_error is above a threshold that indicates a u_turn
                if np.abs(self._lat_controller.heading_error) > self._u_turn_heading_error_threshold:
                    self._target_speed = self._u_turn_speed

                # the throttle output of the PID-Controller
                throttle: float = self.__calculate_throttle(dt, distance)

                # used to drive backwards if we are stuck
                rear_gear: bool = False

                # handle unstucking
                if self._is_unstucking:
                    if rospy.get_rostime().secs - self._unstuck_start_time >= self._unstuck_check_time:
                        self._is_unstucking = False
                    else:
                        rear_gear = True

                # check if we are stuck
                elif self.__check_stuck():
                    self._is_unstucking = True
                    self._unstuck_start_time = rospy.get_rostime().secs
                    rear_gear = True

                # override throttle and steering to drive backwards
                if rear_gear:
                    throttle = 1
                    steering = 0.0
                    self._is_reverse = True

            # handle possible exceptions to come to a hold
            except RuntimeError as e:
                throttle = -1.0  # gets interpreted as braking
                steering = 0.0
                self._is_reverse = False
                self._target_speed = 0.0
                rospy.logwarn_throttle(10, f"[Actor] error ({e})")

            # check if we are in a deadlock
            if not self._deadlock_is_unlocking and self.__check_deadlock():
                self._deadlock_is_unlocking = True
                self._deadlock_timer = rospy.get_rostime().secs

        else:  # handle the deadlock routine
            # check the duration of the deadlock routinge
            deadlock_time = rospy.get_rostime().secs - self._deadlock_timer

            # handle phase one as long as the timer is below a threshold
            # drive backwards
            if deadlock_time <= self._deadlock_phase_1:
                throttle = 1.0
                steering = 0.0
                self._is_reverse = True
                rospy.loginfo_throttle(5, "[Actor] PHASE 1 of deadlock_handling")

            # handle phase two
            # stand still
            elif deadlock_time <= self._deadlock_phase_2:
                throttle = 0.0
                steering = 0.0
                self._is_reverse = False
                rospy.loginfo_throttle(5, "[Actor] PHASE 2 of deadlock_handling")

            # handle phase three
            # drive forwards
            elif deadlock_time <= self._deadlock_phase_3:
                throttle = 1.0
                steering = 0.0
                self._is_reverse = False
                rospy.loginfo_throttle(5, "[Actor] PHASE 3 of deadlock_handling")

            # check if we are at the end
            # handle last routine
            elif deadlock_time <= self._deadlock_phase_end:
                self._deadlock_is_unlocking = False
                self._deadlock_timer = 0.0
                self._deadlock_start_time = 0.0
                rospy.loginfo_throttle(5, "[Actor] PHASE END of deadlock_handling")

        # generate Control Message for carla
        control: CarlaEgoVehicleControl = self.__generate_control_message(throttle, steering)

        # Log the Actor speed error
        msg = PafLogScalar()
        msg.section = "ACTOR speed error"
        msg.value = (self._target_speed - self._current_speed) * 3.6

        self._target_speed_error_log_publisher.publish(msg)

        # Log the Actor speed
        msg = PafLogScalar()
        msg.section = "ACTOR speed"
        msg.value = self._current_speed * 3.6

        self._speed_log_publisher.publish(msg)

        # Log the Actor target speed
        msg = PafLogScalar()
        msg.section = "ACTOR target_speed"
        msg.value = self._target_speed * 3.6
        msg.step_as_distance = False

        self._target_speed_log_publisher.publish(msg)

        # Log the Actor steering
        msg = PafLogScalar()
        msg.section = "ACTOR steering"
        msg.value = np.rad2deg(steering)

        self._steering_log_publisher.publish(msg)

        # Log the Actor throttle
        msg = PafLogScalar()
        msg.section = "ACTOR throttle"
        msg.value = np.clip(throttle, -1, 1)

        self._throttle_log_publisher.publish(msg)

        # return the control Command
        return control

    def __check_stuck(self) -> bool:
        """
        Checks if we are stuck

        Returns:
            bool: True if we are stuck, else False
        """
        # we are stuck if we should drive faster than a low threshold and our current speed is too low
        stuck = self._current_speed < self._stuck_value_threshold < self._target_speed

        # only check if we are stuck if this is not an emergency
        if not self._emergency_mode and stuck:
            # if we recognized that we are stuck
            if self._stuck_start_time == 0.0:
                self._stuck_start_time = rospy.get_rostime().secs
                return False
            # if the duration of our stuck state is too long
            elif rospy.get_rostime().secs - self._stuck_start_time >= self._stuck_check_time:
                self._stuck_start_time = 0.0
                return True
        else:
            # reset stuck timer
            self._stuck_start_time = 0.0
        return False

    def __check_deadlock(self) -> bool:
        """
        Checks if we are deadlocked

        Returns:
            bool: True if we are in a deadlock, else False
        """
        # we are in a deadlock if we are to slow (under a threshold)
        deadlock = abs(self._current_speed) < self._stuck_value_threshold

        if deadlock:
            # if we recognized that a deadlock occured
            if self._deadlock_start_time == 0.0:
                self._deadlock_start_time = rospy.get_rostime().secs
                return False
            # if the duration of our deadlock is too long
            elif rospy.get_rostime().secs - self._deadlock_start_time >= self._deadlock_stuck_check_time:
                self._deadlock_start_time = 0.0
                rospy.logerr_throttle(5, "[Actor] DEADLOCK FOUND!!")
                return True
        else:
            # rest deadlock
            self._deadlock_start_time = 0.0
        return False

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

        # default behaviour
        if not self._deadlock_is_unlocking:
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

            # we are using bugs to brake as fast as possible -> reverse and press that pedal to the metal
            if control.brake > 0 and (self._current_speed > 1 or self._obstacle_follow_active):
                control.reverse = not self._is_reverse
                control.throttle = control.brake

                if self._obstacle_follow_active:
                    control.brake = 0.0
                    control.steer = 0.0

            # we are using another bug to stand still as fast as possible
            # 1. no matter how fast you are, use that handbrake
            # 2. steer as if your life on it to the left or right
            # 3. reverse your gears
            # 4. pedal to the medal as fast as possible
            # 5. ...
            # 6. PROFIT
            if self._emergency_mode:
                control.hand_brake = True
                control.steer = np.rad2deg(30.0)
                control.brake = 0.0
                control.throttle = 1.0
                control.reverse = not self._is_reverse

            # return the control message
            return control
        else:
            # default behaviour if we are in a deadlock
            control.steer = steering
            control.hand_brake = False
            control.throttle = throttle
            control.brake = 0.0
            control.reverse = self._is_reverse
            control.hand_brake = False
            return control

    def __calculate_throttle(self, dt: float, distance: float) -> float:
        """
        Calculate the throttle for the vehicle

        Args:
            dt (float): the timedifference on the last step
            distance (float): the distance between our path and the disired path

        Returns:
            float: the throttle to use
        """
        # perform pid control step with distance and speed controllers
        target_speed: float = self._target_speed * self._target_speed_offset

        # if our distance to the target path is to high, and we drive to past
        # slow down to turn faster
        if distance >= 2 and self._current_speed > 10:
            target_speed = max(10, self._current_speed * (1 - 1e-8))

        # calculate the throttle
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
        rospy.loginfo_throttle(10, f"[Actor] received new local path (len={local_path.points.__len__()})")
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
        # catch divided by zero errors
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

    def __handle_obstacle_msg(self, obstacle_follow_info: PafObstacleFollowInfo):
        """
        Handles the obstacle follow information

        Args:
            obstacle_follow_info (PafObstacleFollowInfo): The ObstacleFollowInfo
        """
        # if we have no target to follow, don't follow the message
        if not obstacle_follow_info.no_target:
            # only use this case if we are in a no rules mode
            if not obstacle_follow_info.is_vehicle and not rospy.get_param("rules_enabled", True):
                # drive slowly over pedestrains to not flip over motorcycles and bicycles
                self._obstacle_follow_speed = 10.0 / 3.6

            # if we are to close to an obstacle we could reverse (currently disabled, too many edge cases)
            elif obstacle_follow_info.distance <= self._obstacle_follow_min_distance / 2:
                rospy.loginfo_throttle(
                    3, f"[Actor] reversing for obstacle in front " f"(d={obstacle_follow_info.distance:.1f})"
                )
                self._obstacle_follow_speed = 0.0

            # we we are to close to and obstacle we come to a stand
            elif obstacle_follow_info.distance <= self._obstacle_follow_min_distance:
                rospy.loginfo_throttle(
                    3, f"[Actor] stopping for obstacle in front " f"(d={obstacle_follow_info.distance:.1f})"
                )
                self._obstacle_follow_speed = 0.0

            # if we want to follow a vehicle, adjust your speed to drive closer to him
            elif obstacle_follow_info.distance <= self._obstacle_follow_target_distance:
                rospy.loginfo_throttle(3, f"[Actor] following an obstacle (d={obstacle_follow_info.distance:.1f})")
                self._obstacle_follow_speed = obstacle_follow_info.speed + self._obstalce_approaching_speed
            else:
                self._obstacle_follow_speed = float("inf")

            # toggle the bool to follow the target vehicle speed
            self._obstacle_follow_active = self._obstacle_follow_speed != float("inf")

        else:
            # keep driving as planned
            self._obstacle_follow_active = False
            self._obstacle_follow_speed = float("inf")

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
                self._vehicle_control_publisher.publish(control)
            else:
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass

    def __del__(self):
        # reset vehicle control message
        self.__generate_control_message(0, 0)


def main():
    """
    Initializes the VehicleController and starts the run loop.
    """

    # init and get params
    rospy.init_node("vehicle_control", anonymous=True)
    role_name: str = rospy.get_param("~role_name", "ego_vehicle")
    target_speed: float = rospy.get_param("~target_speed", 0)
    controller: VehicleController = VehicleController(role_name, target_speed)

    # get it going
    try:
        controller.run()
    finally:
        del controller
    rospy.loginfo("Done")


if __name__ == "__main__":
    main()
