#!/usr/bin/env python
import time

from commonroad.scenario.lanelet import LaneletNetwork

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import (
    PafLocalPath,
    PafLaneletRoute,
    PafTopDownViewPointSet,
    PafSpeedMsg,
    PafDetectedTrafficLights,
)
from classes.MapManager import MapManager
from classes.GlobalPath import GlobalPath
from classes.LocalPath import LocalPath
from classes.HelperFunctions import closest_index_of_point_list, dist
from classes.SpeedCalculator import SpeedCalculator
from std_msgs.msg import Bool, Empty
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    UPDATE_HZ = 10
    REPLAN_THROTTLE_SEC_GLOBAL = 5
    REPLAN_THROTTLE_SEC_LOCAL = 1
    END_OF_ROUTE_REACHED_DIST = 2

    rules_enabled = MapManager.get_rules_enabled()

    def __init__(self):

        rospy.init_node("local_path_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")
        rospy.logwarn(f"[local planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._current_yaw = 0

        self._global_path = GlobalPath()
        self._local_path = LocalPath(self._global_path)
        self._local_path_idx, self._distance_to_local_path = -1, 0
        # self._sparse_local_path_idx = -1
        self._current_global_location = (0, 0)
        self._from_section, self._to_section = -1, -1

        self._traffic_light_color = None
        self._ignore_sign = None
        self._last_sign = None
        self._traffic_light_detector_enabled = False

        self._last_local_reroute = rospy.get_time()
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
        self._last_replan_request_loc = time.perf_counter()
        self._last_replan_request_glob = time.perf_counter()
        # create and start the publisher for the local path
        # self._local_plan_publisher = rospy.Publisher("/paf/paf_local_planner/path", PafLocalPath, queue_size=1)
        self._reacting_path_publisher = rospy.Publisher(
            "/paf/paf_local_planner/reacting_speed", PafLocalPath, queue_size=1
        )
        self._reroute_publisher = rospy.Publisher("/paf/paf_local_planner/reroute", Empty, queue_size=1)
        self._reroute_random_publisher = rospy.Publisher(
            "/paf/paf_local_planner/routing_request_random", Empty, queue_size=1
        )
        self._reroute_standard_loop_publisher = rospy.Publisher(
            "/paf/paf_local_planner/routing_request_standard_loop", Empty, queue_size=1
        )
        self._sign_publisher = rospy.Publisher(
            "/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1
        )
        self._speed_msg_publisher = rospy.Publisher("/paf/paf_validation/speed_text", PafSpeedMsg, queue_size=1)
        self._traffic_light_detector_toggle_pub = rospy.Publisher(
            "/paf/paf_local_planner/activate_traffic_light_detection", Bool, queue_size=1
        )

    def _change_rules(self, msg: Bool):
        rospy.set_param("rules_enabled", msg.data)
        if msg.data == self.rules_enabled:
            return
        self.rules_enabled = msg.data
        self._scenario = MapManager.get_current_scenario()
        if not self.rules_enabled:
            self._traffic_light_detector_toggle_pub.publish(Bool(False))
        SpeedCalculator.set_limits(self.rules_enabled)
        rospy.logwarn(
            f"[local planner] Rules are now {'en' if self.rules_enabled else 'dis'}abled! "
            f"Speed limits will change after starting a new route."
        )

    def _process_global_path(self, msg: PafLaneletRoute):
        if self._global_path is None or self._global_path.route.distance != msg.distance:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distance)}m")
            self._reset_detected_signs()
            self._emergency_break_pub.publish(Bool(False))
            self._global_path = GlobalPath(msg=msg)
            self._local_path = LocalPath(self._global_path, self.rules_enabled)
            self._set_local_path_idx()
            path, self._from_section, self._to_section = self._local_path.calculate_new_local_path(
                self._current_pose.position, self._current_speed, ignore_previous=True
            )

    def _publish_local_path_msg(self, send_empty=False):
        self._local_path.publish(send_empty=send_empty)

    def _loop_handler(self):
        self._publish_speed_msg()

        if len(self._global_path) == 0 or len(self._local_path) == 0:
            rospy.loginfo_throttle(5, "[local planner] publishing empty path.")
            self._publish_local_path_msg(send_empty=True)
        if len(self._global_path) == 0:
            rospy.logwarn_throttle(5, "[local planner] no route, end of route handling activated")
            self._reset_detected_signs()
            self._end_of_route_handling()
        elif self._planner_at_end_of_global_path():
            rospy.logwarn_throttle(5, "[local planner] end of route reached, end of route handling activated")
            self._reset_detected_signs()
            self._global_path = GlobalPath()
            self._local_path = LocalPath(self._global_path)
            self._emergency_break_pub.publish(Bool(True))
            self._end_of_route_handling(sleep=5)
        elif not self._on_global_path():
            rospy.logwarn_throttle(5, "[local planner] not on global path, requesting new route")
            self._publish_local_path_msg(send_empty=True)
            self._send_global_path_request()
        elif not self._on_local_path():
            rospy.loginfo_throttle(5, "[local planner] not on local path, replanning")
            self._replan_local_path()
            self._publish_local_path_msg()
        # elif self._is_stopped() and self._local_path_idx < len(self._local_path) and len(self._local_path) > 0:
        #     rospy.logwarn_throttle(5, "[local planner] car is waiting for event (red light / stop / yield)")
        #     self._handle_clear_event()  # todo change this handler
        elif self._planner_at_end_of_local_path():
            rospy.loginfo_throttle(5, "[local planner] local planner is replanning (end of local path)")
            self._replan_local_path()
            self._publish_local_path_msg()
        else:
            rospy.loginfo_throttle(5, "[local planner] local planner on route, no need to replan")
            self._publish_local_path_msg()

        self._check_for_signs_on_path()
        # if self._last_sign is not None:
        #     self._ignore_sign = self._last_sign

    def _reset_detected_signs(self):
        self._last_sign = self._ignore_sign = None

    def _check_for_signs_on_path(self):
        if not self.rules_enabled:
            return

        self._last_sign, found_ignored_sign = self._local_path.set_alternate_speed_next_sign(
            self._local_path_idx, self._ignore_sign
        )
        if self._traffic_light_color == "green" and self._last_sign is not None:
            self._ignore_sign = self._last_sign
        if self._last_sign is not None and not self._traffic_light_detector_enabled:
            self._traffic_light_detector_enabled = True
            self._traffic_light_detector_toggle_pub.publish(Bool(True))
            rospy.loginfo_throttle(1, "[local planner] enabling traffic light detection")
        # elif not found_ignored_sign and self._last_sign is None and self._traffic_light_detector_enabled:
        #     self._traffic_light_detector_enabled = False
        #     self._traffic_light_detector_toggle_pub.publish(Bool(False))
        #     rospy.loginfo_throttle(1, "[local planner] disabling traffic light detection")

        msg_info = []
        for i, sign_list in enumerate(self._local_path.traffic_signals):
            for x in sign_list:
                msg_info.append(f"{x.type} in {(i * LocalPath.DENSE_POINT_DISTANCE):.1f}m")
        rospy.loginfo_throttle(
            10, f"[local planner] signs on path: {', '.join(msg_info) if len(msg_info) > 0 else 'None'}"
        )

    def _process_traffic_lights(self, msg: PafDetectedTrafficLights):
        if not self._traffic_light_detector_enabled:
            return
        if len(msg.states) == 0:
            self._traffic_light_color = None

        max_distance = 100
        section, current_lane = self._current_global_location
        number_of_lanes = len(self._global_path.route.sections[section].points)
        zipped = list(zip(msg.states, msg.distances, msg.positions))
        zipped = [(a, b, c) for a, b, c in zipped if b < max_distance]
        number_of_signs = len(zipped)
        sort_by_x = sorted(zipped, key=lambda x: x[2].x, reverse=True)
        closest_to_car = int(np.clip(current_lane, 0, len(sort_by_x)))
        self._traffic_light_color = msg.states[closest_to_car]
        rospy.logwarn_throttle(
            5,
            f"[local planner] detected lights: {[x for x,_,_ in sort_by_x]} "
            f"(lanes: {number_of_lanes}, lights: {number_of_signs})",
        )

    def _replan_local_path(self, ignore_prev=False):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_loc
        if self.REPLAN_THROTTLE_SEC_LOCAL > delta_t:
            return

        self._last_replan_request_loc = t
        self._reset_detected_signs()
        msg, self._from_section, self._to_section = self._local_path.calculate_new_local_path(
            self._current_pose.position, self._current_speed, ignore_previous=ignore_prev, min_section=self._to_section
        )

    # def _allowed_from_stop(self):
    #     return True or self._can_continue_on_path(None)  # todo
    #
    # def _can_continue_on_path(self, signal_type):
    #     if signal_type is None:
    #         return True
    #     if signal_type == "LIGHT":
    #         return self._traffic_light_color == TrafficLightState.GREEN
    #     return True  # todo joining lanes / lane change free ??

    def _is_stopped(self):
        margin = 0.5
        stop = -margin < self._current_speed < margin
        return stop

    def _planner_at_end_of_local_path(self):
        if len(self._local_path) == 0:
            return True
        if dist(self._local_path.message.points[-1], self._current_pose.position) < 100:
            return dist(self._local_path.message.points[-1], self._global_path.target) > self.END_OF_ROUTE_REACHED_DIST

    def _planner_at_end_of_global_path(self):
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
        _, self._current_pitch, self._current_yaw = euler_from_quaternion(
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
        if len(self._local_path) == 0:
            self._local_path_idx = -1
        else:
            self._local_path_idx, self._distance_to_local_path = closest_index_of_point_list(
                self._local_path.message.points, self._current_pose.position
            )
            # self._sparse_local_path_idx, _ = closest_index_of_point_list(
            #     self._local_path.sparse_local_path, self._current_pose.position
            # )
            self._current_global_location = self._global_path.get_section_and_lane_indices(self._current_pose.position)

    def _publish_speed_msg(self):
        if 0 <= self._local_path_idx < len(self._local_path.message.target_speed):
            self._speed_msg.limit = 0
            self._speed_msg.target = self._local_path.message.target_speed[self._local_path_idx]
            self._speed_msg_publisher.publish(self._speed_msg)

    def _on_global_path(self):
        return (
            self._global_path.get_section_and_lane_indices(self._current_pose.position, not_found_threshold_meters=15)[
                0
            ]
            >= 0
        )

    def _send_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_glob
        if self.REPLAN_THROTTLE_SEC_GLOBAL < delta_t:
            self._last_replan_request_glob = t
            rospy.loginfo("[local planner] requesting new global route")
            self._reroute_publisher.publish(Empty())

    def _send_standard_loop_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_glob
        if self.REPLAN_THROTTLE_SEC_GLOBAL < delta_t:
            self._last_replan_request_glob = t
            rospy.loginfo("[local planner] requesting new standard loop route")
            self._reroute_standard_loop_publisher.publish(Empty())

    def _send_random_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request_glob
        if self.REPLAN_THROTTLE_SEC_GLOBAL < delta_t:
            self._last_replan_request_glob = t
            rospy.loginfo("[local planner] requesting new random global route")
            self._reroute_random_publisher.publish(Empty())

    def _end_of_route_handling(self, sleep=0):
        self._global_path = GlobalPath()
        rospy.Publisher("/paf/paf_validation/score/stop", Empty, queue_size=1).publish(Empty())
        if self._current_speed < 5:
            self._emergency_break_pub.publish(Bool(False))
        if self._current_speed < 0.01:
            if sleep > 0:
                rospy.sleep(sleep)
            # self._send_random_global_path_request()  # todo remove in production
            self._send_standard_loop_request()  # todo remove in production

    def start(self):
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            self._loop_handler()
            rate.sleep()

    def _on_local_path(self):
        return self._distance_to_local_path < 15

    def __del__(self):
        try:
            self._publish_local_path_msg(send_empty=True)
        except Exception:
            pass


if __name__ == "__main__":
    LocalPlanner().start()
