#!/usr/bin/env python
import copy
import time
from collections import deque

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.traffic_sign import TrafficSignIDGermany

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import (
    PafLaneletRoute,
    PafSpeedMsg,
    PafDetectedTrafficLights,
    PafTrafficSignal,
)
from classes.MapManager import MapManager
from classes.GlobalPath import GlobalPath
from classes.LocalPath import LocalPath
from classes.HelperFunctions import closest_index_of_point_list, dist
from classes.SpeedCalculator import SpeedCalculator
from paf_messages.srv import PafRoutingService
from std_msgs.msg import Bool, Empty
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    UPDATE_HZ = 10
    REPLAN_THROTTLE_SEC_GLOBAL = 5
    REPLAN_THROTTLE_SEC_LOCAL = 1
    END_OF_ROUTE_REACHED_DIST = 2
    USE_GLOBAL_STANDARD_LOOP = False

    # %ignore left, 1-%ignore right, max distance
    AMERICAN_TRAFFIC_LIGHT_LIMITS = 0.25, 0.75, 50
    EUROPEAN_TRAFFIC_LIGHT_LIMITS = 0.25, 1, 20

    rules_enabled = MapManager.get_rules_enabled()

    def __init__(self):
        """
        Setup node and init all subscribers, services and publishers needed.
        """
        self._cleared_signs = deque(maxlen=2)
        rospy.init_node("local_path_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        rospy.logwarn(f"[local planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_yaw = 0

        self._global_path = GlobalPath()
        self._local_path = LocalPath(self._global_path)
        self._local_path_idx, self._distance_to_local_path = -1, 0
        self._last_replan_request_loc = time.perf_counter()
        self._last_replan_request_glob = time.perf_counter()

        self._traffic_light_color = None
        self._last_sign = None
        self._traffic_light_detector_enabled = True
        self._speed_msg = PafSpeedMsg()

        self._network: LaneletNetwork = MapManager.get_current_scenario().lanelet_network

        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        rospy.Subscriber(
            "/paf/paf_global_planner/routing_response", PafLaneletRoute, self._process_global_path, queue_size=1
        )
        rospy.Subscriber("/paf/paf_local_planner/rules_enabled", Bool, self._change_rules, queue_size=1)
        rospy.Subscriber(
            "/paf/paf_perception/detected_traffic_lights",
            PafDetectedTrafficLights,
            self._process_traffic_lights,
            queue_size=1,
        )

        self._emergency_break_pub = rospy.Publisher(f"/local_planner/{role_name}/emergency_break", Bool, queue_size=1)
        self._srv_global_reroute = "/paf/paf_local_planner/reroute"
        self._srv_global_random = "/paf/paf_local_planner/routing_request_random"
        self._srv_global_standard_loop = "/paf/paf_local_planner/routing_request_standard_loop"
        self._speed_msg_publisher = rospy.Publisher("/paf/paf_validation/speed_text", PafSpeedMsg, queue_size=1)
        self._traffic_light_detector_toggle_pub = rospy.Publisher(
            "/paf/paf_local_planner/activate_traffic_light_detection", Bool, queue_size=1
        )
        self._emergency_break_pub.publish(Bool(True))

    def _routing_service_call(self, service_name: str):
        """
        Call routing service based on a given service name. Then process the received global path
        :param service_name: reroute, random and standard loop services allowed
        """
        rospy.loginfo_throttle(3, f"[local planner] requesting route ({service_name})")
        try:
            rospy.wait_for_service(service_name, timeout=rospy.Duration(10))
            call_service = rospy.ServiceProxy(service_name, PafRoutingService)
            response = call_service()
            self._process_global_path(response.route)
        except (rospy.ServiceException, rospy.exceptions.ROSException) as e:
            rospy.logerr_throttle(1, f"[local planner] {e}")

    def _change_rules(self, msg: Bool):
        """
        Change rules (driving mode) during runtime. Updates Speed Calculation rules and triggers rerouting.
        :param msg: Bool message to set rules_enabled to
        """
        rospy.set_param("rules_enabled", msg.data)
        if msg.data == self.rules_enabled:
            return
        self.rules_enabled = msg.data
        self._scenario = MapManager.get_current_scenario()
        if not self.rules_enabled:
            self._traffic_light_detector_toggle_pub.publish(Bool(False))
        SpeedCalculator.set_limits(self.rules_enabled)
        self._routing_service_call(self._srv_global_reroute)
        rospy.logwarn(
            f"[local planner] Rules are now {'en' if self.rules_enabled else 'dis'}abled! "
            f"Speed limits will change after starting a new route."
        )

    def _process_global_path(self, msg: PafLaneletRoute):
        """
        process a given Global Path message if it differs from the previously received route.
        :param msg: route message
        """
        if self._global_path is None or self._global_path.route.distance != msg.distance:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distance)}m")
            self._reset_detected_signs()
            self._global_path = GlobalPath(msg=msg, lanelet_network=self._network)
            self._local_path = LocalPath(self._global_path, self.rules_enabled)
            self._set_local_path_idx()
            path, _, _ = self._local_path.calculate_new_local_path(
                self._current_pose.position, self._current_speed, ignore_previous=True
            )
            self._set_local_path_idx()
            if self.rules_enabled:
                self._local_path.set_alternate_speed_next_sign(self._local_path_idx)
            self._emergency_break_pub.publish(Bool(False))
            self._publish_local_path_msg()

    def _publish_local_path_msg(self, send_empty=False):
        """
        Publish the current local path to acting
        :param send_empty: if true, publish an empty path (acting stops driving then)
        """
        self._local_path.publish(send_empty=send_empty)

    def _loop_handler(self):
        """
        Main Loop
        """
        self._publish_speed_msg()  # speed message on topdown view image

        # no LP and no GP: stop acting from driving
        if len(self._global_path) == 0 or len(self._local_path) == 0:
            rospy.loginfo_throttle(5, "[local planner] publishing empty path.")
            self._publish_local_path_msg(send_empty=True)

        # empty GP
        if len(self._global_path) == 0:
            rospy.logwarn_throttle(5, "[local planner] no route, end of route handling activated")
            self._reset_detected_signs()
            self._end_of_route_handling()

        # end of GP
        elif self._planner_at_end_of_global_path():
            rospy.logwarn_throttle(5, "[local planner] end of route reached, end of route handling activated")
            self._reset_detected_signs()
            self._global_path = GlobalPath()
            self._local_path = LocalPath(self._global_path)
            self._emergency_break_pub.publish(Bool(True))
            self._end_of_route_handling(sleep=5)

        # off route (reroute)
        elif not self._on_global_path():
            rospy.logwarn_throttle(5, "[local planner] not on global path, requesting new route")
            self._publish_local_path_msg(send_empty=True)
            self._send_global_reroute_request()

        # off local path (replan LP from current global path position)
        elif not self._on_local_path():
            rospy.loginfo_throttle(5, "[local planner] not on local path, replanning")
            self._replan_local_path()

        # STOP SIGNS: if stopped, clear stop sign (if next sign is stop sign)
        elif (
            self._last_sign is not None
            and self._is_stopped()
            and self._local_path_idx < len(self._local_path)
            and len(self._local_path) > 0
        ):
            try:
                sig_type = TrafficSignIDGermany(self._last_sign).name
            except ValueError:
                sig_type = None if self._last_sign is None else self._last_sign.type
            rospy.logwarn_throttle(
                5, f"[local planner] car is waiting for event {sig_type} (light color={self._traffic_light_color})"
            )
            self._handle_stop_event()

        # handle end of local path (replan LP from current global path position)
        elif self._planner_at_end_of_local_path():
            rospy.loginfo_throttle(5, "[local planner] local planner is replanning (end of local path)")
            self._replan_local_path()

        # ELSE: do nothing
        else:
            rospy.loginfo_throttle(5, "[local planner] local planner on route, no need to replan")
            self._publish_local_path_msg()

        # ALWAYS: update applicable signs on LP, publish LP to acting
        self._check_for_signs_on_path()
        self._publish_local_path_msg()

        # replanning condition (lane changes, reacting to cars on our lane)
        if self.rules_enabled:

            if (
                self._current_speed < 25 / 3.6
                and self._current_speed > 15 / 3.6
                and self._local_path_idx < len(self._local_path) - 150
            ):
                rospy.loginfo_throttle(
                    5, "[local planner] car is slow, no replanning locally because traffic light bug"
                )
        else:
            if self._current_speed < 25 / 3.6 and self._local_path_idx < len(self._local_path) - 150:
                rospy.loginfo_throttle(5, "[local planner] car is slow, replanning locally")

                # traffic light handling not working correctly when turned on
                self._replan_local_path()

    def _add_cleared_signal(self, signal: PafTrafficSignal) -> bool:
        """
        Try to clear a signal. Cannot clear if: signal is None, not rules_enabled, or has already been cleared
        :param signal: sign to clear
        :return: success
        """
        if signal is None or not self.rules_enabled:
            return False
        if len(self._cleared_signs) == 0 or (
            len(self._cleared_signs) > 0 and not LocalPath.signs_equal(self._cleared_signs[-1], signal)
        ):
            self._local_path.reset_alternate_speed()
            self._cleared_signs.append(signal)
            self._last_sign = None
            return True
        return False

    def _handle_stop_event(self):
        """
        Check if next sign is stop sign and clear it (if applicable)
        """
        if self._last_sign is None:
            return
        if (
            self._last_sign.type == TrafficSignIDGermany.STOP.value
            and dist(self._last_sign.point, self._current_pose.position) < 5
        ):
            rospy.loginfo_throttle(3, "[local planner] waiting for path to clear (stop sign)")
            self._reset_detected_signs(and_publish=True)

    def _reset_detected_signs(self, and_publish: bool = False):
        """
        Reset alternative (stopping-) speed and clear signs if close to them
        :param and_publish: publish cleared path if applicable
        """
        if self._last_sign is None or not self.rules_enabled:
            return
        to_clear = copy.copy(self._last_sign)
        self._local_path.reset_alternate_speed()

        if self._last_sign.type == "LIGHT":
            clear_distance = LocalPath.CLEARING_SIGN_DIST_LIGHT
        else:
            clear_distance = LocalPath.CLEARING_SIGN_DIST

        if MapManager.light_is_opposite_stop_point():
            # american traffic light
            if dist(self._current_pose.position, to_clear.point) <= clear_distance:
                self._add_cleared_signal(to_clear)
        else:
            # european traffic light
            # experimental solution better for european style traffic lights
            if dist(self._current_pose.position, to_clear.point) - LocalPath.OFFSET_LIGHTS_EU_M <= clear_distance:
                # only add to ignored list when very close
                self._add_cleared_signal(to_clear)

        if and_publish:
            self._local_path.publish()
            rospy.logwarn_throttle(5, "[local planner] publishing original path after clearing path")

    def _check_for_signs_on_path(self):
        """
        Check for the next sign and set the alternative speed to stop there (if applicable).
        Reset next sign if it is a green traffic light.
        """
        if not self.rules_enabled:
            return

        self._last_sign, found_ignored_sign = self._local_path.set_alternate_speed_next_sign(
            self._local_path_idx,
            self._cleared_signs[-1] if len(self._cleared_signs) > 0 else None,
            self._traffic_light_color,
        )
        if (
            self._last_sign is not None
            and self._last_sign.type == "LIGHT"
            and self._traffic_light_color in LocalPath.RESUME_COURSE_COLORS
        ):
            rospy.logwarn_throttle(1, f"[local planner] resuming from red light (color={self._traffic_light_color})")
            self._reset_detected_signs(and_publish=True)

        msg_info = []
        for i, (sign, idx, distance, match) in enumerate(self._local_path.traffic_signals):
            p_dist = dist(self._current_pose.position, sign.point)
            msg_info.append(f"{sign.type} ({p_dist:.1f}m)")
        rospy.loginfo_throttle(10, f"[local planner] signs on path: [{', '.join(msg_info)}]")

    def _process_traffic_lights(self, msg: PafDetectedTrafficLights):
        """
        Callback function for received messages from traffic light detection node.
        The closest light to the center of the sensor image within the range limits is selected
        :param msg: detected light states and position on camera image
        """
        if not self._traffic_light_detector_enabled:
            rospy.logwarn_throttle(
                5,
                f"[local planner] traffic light detection is currently disabled, "
                f"skipping {len(msg.states)} detections",
            )
            return
        if len(msg.states) == 0:
            self._traffic_light_color = None
            if self._is_stopped() and self._last_sign is not None and self._last_sign.type == "LIGHT":
                rospy.logerr_throttle(5, "[local planner] traffic light detection sent an empty message")
            return

        limit_l, limit_r, limit_dist = (
            self.AMERICAN_TRAFFIC_LIGHT_LIMITS
            if MapManager.light_is_opposite_stop_point()
            else self.EUROPEAN_TRAFFIC_LIGHT_LIMITS
        )

        def filter_msgs():  # filter messages with given limits
            new_msg = PafDetectedTrafficLights()
            for state, distance, position in zip(msg.states, msg.distances, msg.positions):
                if distance > limit_dist:
                    continue  # ignore all detections exceeding this range
                if position.x < limit_l or position.x > limit_r:
                    continue  # ignore side detections
                new_msg.states.append(state)
                new_msg.distances.append(distance)
                new_msg.positions.append(position)
            return new_msg

        msg2 = filter_msgs()
        if len(msg.states) == 0 or len(msg2.states) == 0:
            self._traffic_light_color = None
            rospy.loginfo_throttle(1, "[local planner] N/A []")
            return

        # select most centered light on the image
        x_list = [np.abs(p.x - 0.5) for p in msg2.positions]
        index_center = np.argmin(x_list)
        self._traffic_light_color: str = msg2.states[index_center]

        rospy.loginfo_throttle(1, f"[local planner] {self._traffic_light_color.upper()} {msg2.states}")
        if self._traffic_light_color in LocalPath.RESUME_COURSE_COLORS:
            rospy.loginfo_throttle(1, f"[local planner] clearing traffic light {self._traffic_light_color.upper()}")
            self._reset_detected_signs(and_publish=True)
        else:
            rospy.loginfo_throttle(1, f"[local planner] light detected as {self._traffic_light_color.upper()}")

    def _changing_lanes(self):
        """
        Test if the current vehicle position is within a planned lane change
        :return: truth
        """
        idx, d = closest_index_of_point_list(self._local_path.sparse_local_path, self._current_pose.position)
        if d > 10:
            return False
        return self._local_path.lane_change_at(idx)

    def _replan_local_path(self, ignore_prev=False):
        """
        Recalculate the Local Path from the current position.
        :param ignore_prev: use previous local path for smoother transition
        """
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_loc
        if self.REPLAN_THROTTLE_SEC_LOCAL > delta_t or self._changing_lanes():
            return

        self._last_replan_request_loc = t
        msg, _, _ = self._local_path.calculate_new_local_path(
            self._current_pose.position, self._current_speed, ignore_previous=ignore_prev
        )
        self._set_local_path_idx()
        if self.rules_enabled:
            self._local_path.set_alternate_speed_next_sign(self._local_path_idx)

        self._publish_local_path_msg()

    def _is_stopped(self):
        """
        Test if car is stopped (within a margin of error)
        :return: truth
        """
        margin = 0.5
        stop = -margin < self._current_speed < margin
        return stop

    def _planner_at_end_of_local_path(self):
        """
        Test if ego vehicle is at the end of the local path
        :return: truth
        """
        if len(self._local_path) == 0:
            return True
        if dist(self._local_path.message.points[-1], self._current_pose.position) < 100:
            # return false if end of route is close
            return dist(self._local_path.message.points[-1], self._global_path.target) > self.END_OF_ROUTE_REACHED_DIST

    def _planner_at_end_of_global_path(self):
        """
        Test if ego vehicle is at the end of the global path
        :return:
        """
        if len(self._global_path) == 0:
            return False
        return (
            len(self._global_path) > 0
            and dist(self._global_path.target, self._current_pose.position) <= self.END_OF_ROUTE_REACHED_DIST
            or dist(self._global_path.route.sections[-1].points[0], self._current_pose.position)
            <= self.END_OF_ROUTE_REACHED_DIST * 2
        )

    def _odometry_updated(self, odometry: Odometry):
        """Odometry Update Callback"""

        # calculate current speed (m/s) from twist
        self._current_speed = np.sqrt(
            odometry.twist.twist.linear.x ** 2 + odometry.twist.twist.linear.y ** 2 + odometry.twist.twist.linear.z ** 2
        )
        _, _, self._current_yaw = euler_from_quaternion(
            [
                odometry.pose.pose.orientation.x,
                odometry.pose.pose.orientation.y,
                odometry.pose.pose.orientation.z,
                odometry.pose.pose.orientation.w,
            ]
        )
        self._current_pose = odometry.pose.pose
        self._set_local_path_idx()

    def _set_local_path_idx(self):
        """
        Update function for current position index on local path
        """
        if len(self._local_path) == 0:
            self._local_path_idx = -1
            self._local_path.current_index = 0
        else:
            self._local_path_idx, self._distance_to_local_path = closest_index_of_point_list(
                self._local_path.message.points, self._current_pose.position
            )
            self._local_path.current_index = self._local_path_idx

    def _publish_speed_msg(self):
        """
        Publish speed debug message for top down view
        """
        if 0 <= self._local_path_idx < len(self._local_path.message.target_speed):
            self._speed_msg.limit = 0
            self._speed_msg.target = self._local_path.message.target_speed[self._local_path_idx]
            self._speed_msg_publisher.publish(self._speed_msg)

    def _on_global_path(self):
        """
        Test if vehicle is on global path (within 15m)
        :return: truth
        """
        return (
            self._global_path.get_section_and_lane_indices(self._current_pose.position, not_found_threshold_meters=15)[
                0
            ]
            >= 0
        )

    def _on_local_path(self):
        """
        Test if vehicle is on local path (within 15m)
        :return: truth
        """
        return self._distance_to_local_path < 15

    def _send_global_reroute_request(self):
        """
        Call rerouting service
        """
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_glob
        if self.REPLAN_THROTTLE_SEC_GLOBAL < delta_t:
            self._last_replan_request_glob = t
            rospy.loginfo("[local planner] requesting new global route")
            self._routing_service_call(self._srv_global_reroute)

    def _send_standard_loop_request(self):
        """
        Call standard loop service (get next waypoint)
        """
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_glob
        if self.REPLAN_THROTTLE_SEC_GLOBAL < delta_t:
            self._last_replan_request_glob = t
            rospy.loginfo("[local planner] requesting new standard loop route")
            self._routing_service_call(self._srv_global_standard_loop)

    def _send_random_global_path_request(self):
        """
        Call random global path service
        """
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_glob
        if self.REPLAN_THROTTLE_SEC_GLOBAL < delta_t:
            self._last_replan_request_glob = t
            rospy.loginfo("[local planner] requesting new random global route")
            self._routing_service_call(self._srv_global_random)

    def _end_of_route_handling(self, sleep=0):
        """
        Handle end of route event: Stop score calculation, enable emergency break
        :param sleep:
        """
        self._global_path = GlobalPath()
        rospy.Publisher("/paf/paf_validation/score/stop", Empty, queue_size=1).publish(Empty())
        if self._current_speed < 5 / 3.6:
            self._emergency_break_pub.publish(Bool(False))

        if rospy.get_param("/validation"):
            if self._current_speed < 0.01:
                if sleep > 0:
                    rospy.sleep(sleep)
                if self.USE_GLOBAL_STANDARD_LOOP:
                    self._send_standard_loop_request()
                else:
                    self._send_random_global_path_request()

    def start(self):
        """
        Start node (loop)
        """
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            self._loop_handler()
            rate.sleep()

    def __del__(self):
        """
        Destructor: Send empty path to acting (stop driving)
        """
        try:
            self._publish_local_path_msg(send_empty=True)
        except Exception:
            pass


if __name__ == "__main__":
    LocalPlanner().start()
