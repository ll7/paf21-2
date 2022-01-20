#!/usr/bin/env python
import time

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.traffic_sign import (
    TrafficLightState,
)

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import (
    PafLocalPath,
    PafLaneletRoute,
    PafTopDownViewPointSet,
    PafSpeedMsg,
    PafRouteSection,
)
from classes.MapManager import MapManager
from classes.GlobalPath import GlobalPath
from classes.LocalPath import LocalPath
from classes.HelperFunctions import closest_index_of_point_list, dist_pts
from classes.SpeedCalculator import SpeedCalculator
from std_msgs.msg import Bool, Empty
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    UPDATE_HZ = 10
    REPLAN_THROTTLE_SEC = 5
    END_OF_ROUTE_REACHED_DIST = 2
    # END_OF_ROUTE_SPEED = 5
    # MAX_ANGULAR_ERROR = np.deg2rad(45)

    rules_enabled = rospy.get_param("rules_enabled", False)

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
        self._from_section, self._to_section = -1, -1

        self._traffic_light_color = TrafficLightState.GREEN  # or None # todo fill this
        # self._following_distance = -1  # todo set following distance
        # self._following_speed = -1  # todo set following speed
        self._last_local_reroute = rospy.Time.now().to_time()
        self._speed_msg = PafSpeedMsg()

        self._network: LaneletNetwork = MapManager.get_current_scenario().lanelet_network

        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated, queue_size=1)
        rospy.Subscriber(
            "/paf/paf_global_planner/routing_response", PafLaneletRoute, self._process_global_path, queue_size=1
        )
        rospy.Subscriber("/paf/paf_local_planner/rules_enabled", Bool, self._change_rules, queue_size=1)
        self._last_replan_request = time.perf_counter()
        # create and start the publisher for the local path
        self._local_plan_publisher = rospy.Publisher("/paf/paf_local_planner/path", PafLocalPath, queue_size=1)
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

    def _change_rules(self, msg: Bool):
        rospy.set_param("rules_enabled", msg.data)
        if msg.data == self.rules_enabled:
            return
        self.rules_enabled = msg.data
        self._scenario = MapManager.get_current_scenario()
        rospy.logwarn(
            f"[local planner] Rules are now {'en' if self.rules_enabled else 'dis'}abled! "
            f"Speed limits will change after starting a new route."
        )

    def _process_global_path(self, msg: PafLaneletRoute):
        if self._global_path is None or self._global_path.route.distance != msg.distance:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distance)}m")
            self._global_path = GlobalPath(msg=msg)
            self._local_path = LocalPath(self._global_path, self.rules_enabled)
            self._set_local_path_idx()
            path, self._from_section, self._to_section = self._local_path.calculate_new_local_path(
                self._current_pose.position, self._current_speed
            )
            self._create_paf_local_path_msg(path)

    # def _get_track_angle(self, s_index, l_index):
    #     if len(self._global_path.sections) == 0:
    #         return 0
    #     if s_index == len(self._global_path.sections) - 1:
    #         s_index -= 1
    #     p1 = self._global_path.sections[s_index]
    #     p2 = self._global_path[index + 1]
    #     v2 = [p2.x - p1.x, p2.y - p1.y]
    #     return get_angle_between_vectors(v2)

    # def _get_turning_path_to(self, index, track_angle, track_err, distance):
    #     def is_left_of_target_line():
    #         a = p_target_1
    #         b = p_target_2
    #         c = self._current_pose.position
    #         return (b.x - a.x) * (c.y - a.y) > (b.y - a.y) * (c.x - a.x)
    #
    #     index_forwards = int(min(len(self._global_path.sections) - 1, index + 10 / 0.125))
    #     p_target_1 = self._global_path[index_forwards - 1]
    #     p_target_2 = self._global_path[index_forwards]
    #
    #     # d1, d2, radius = 4, 10, -5
    #     # turning_dir = 1
    #
    #     x_list, y_list = [], []
    #     pts = [
    #         xy_from_distance_and_angle(self._current_pose.position, 0, -self._current_yaw),
    #         xy_from_distance_and_angle(self._current_pose.position, 4, -self._current_yaw),
    #         xy_from_distance_and_angle(p_target_1, 5, track_angle + np.pi / 2),
    #     ]
    #
    #     for pt in pts:
    #         x_list.append(pt[0])
    #         y_list.append(pt[1])
    #     x_list = x_list[:2] + [p_target_1.x] + x_list[2:]  # , p_target_2.x]
    #     y_list = y_list[:2] + [p_target_1.y] + y_list[2:]  # , p_target_2.y]
    #
    #     x_list_out, y_list_out, _, _, _ = calc_spline_course(x_list, y_list, 0.125)
    #
    #     self._local_path = [Point2D(x, y) for x, y in zip(x_list_out, y_list_out)]
    #
    #     self._target_speed = [1 for _ in self._local_path]

    def _create_paf_local_path_msg(self, path_msg=None, send_empty=False):
        if path_msg is None:
            path_msg = self._local_path.message
        else:
            self._local_path.message = path_msg

        if send_empty:
            path_msg.header.stamp = rospy.Time.now()
            path_msg.target_speed = []
            path_msg.points = []

        self._local_plan_publisher.publish(path_msg)

    def _loop_handler(self):
        self._publish_speed_msg()

        if len(self._global_path) == 0:
            rospy.logwarn_throttle(5, "[local planner] no route, end of route handling activated")
            self._end_of_route_handling()
        elif self._planner_at_end_of_global_path():
            rospy.logwarn_throttle(5, "[local planner] end of route reached, end of route handling activated")
            self._global_path = GlobalPath()
            self._local_path = LocalPath(self._global_path)
            self._end_of_route_handling()
        elif not self._on_global_path():
            rospy.logwarn_throttle(5, "[local planner] not on global path, requesting new route")
            self._create_paf_local_path_msg(send_empty=True)
            self._send_global_path_request()
        elif not self._on_local_path():
            rospy.loginfo_throttle(5, "[local planner] not on local path, replanning")
            self._replan_local_path()
        elif self._is_stopped() or 0 < len(self._local_path) - self._local_path_idx < 100:
            rospy.logwarn_throttle(5, "[local planner] car is waiting for event (red light / stop / yield)")
            self._handle_clear_event()  # todo change this handler
        elif self._planner_at_end_of_local_path():
            rospy.loginfo_throttle(5, "[local planner] local planner is replanning (end of local path)")
            self._replan_local_path()
        else:
            rospy.loginfo_throttle(5, "[local planner] local planner on route, no need to replan")

        if self._local_path_idx < len(self._local_path) and len(self._local_path) > 0:
            rospy.loginfo_throttle(
                5,
                f"[local planner] current target speed: "
                f"{int(self._local_path.message.target_speed[self._local_path_idx] * 3.6)}",
            )

    def _is_waiting_for_clear_event(self):
        pass  # todo implement this

    def _handle_clear_event(self):
        rospy.sleep(1)  # todo remove this

        self._replan_local_path(ignore_prev=True)
        self._local_path.message.target_speed = SpeedCalculator.remove_stop_event(
            self._local_path.message.target_speed, 0
        )
        to_section, _ = self._global_path.get_section_and_lane_indices(
            self._current_pose.position, not_found_threshold_meters=50
        )
        # rospy.logwarn(f"[local planner] clearing stop / yield event (index {index})")
        if to_section < 0:
            rospy.logerr("[local planner] cannot continue from here, current section unknown")
            return
        section: PafRouteSection
        from_section = self._from_section
        to_section += 5
        for section in self._global_path.route.sections[from_section:to_section]:
            section.signals = []

        self._create_paf_local_path_msg()

    def _replan_local_path(self, ignore_prev=False):
        msg, self._from_section, self._to_section = self._local_path.calculate_new_local_path(
            self._current_pose.position, self._current_speed, ignore_previous=ignore_prev, min_section=self._to_section
        )
        self._create_paf_local_path_msg(msg)

    # def _get_current_trajectory(self):
    #     m_idx, l_idx, p_idx = self._set_current_indices()
    #     last_local_reroute = rospy.Time.now().to_time()
    #     delta_t = last_local_reroute - self._last_local_reroute
    #     if len(self._local_path) > 0 and self._planner_at_end_of_global_path(self._local_path):
    #         self._end_of_route_handling()
    #         self._local_path = []
    #         self._target_speed = []
    #         self._global_path = []
    #         is_new_message = True
    #     elif len(self._global_path.sections) > 0 and m_idx >= 0 and (
    #             delta_t > self.REPLAN_THROTTLE_SEC / 2 or m_idx == len(self._global_path.sections) - 1
    #     ):
    #         self._set_current_path()
    #         is_new_message = True
    #         rospy.loginfo_throttle(30, "[local planner] local planner is replanning")
    #         self._last_local_reroute = last_local_reroute
    #     else:
    #         is_new_message = False
    #     return self._local_path, self._target_speed, is_new_message
    #
    # def _signal_debug_print(self, signals):
    #
    #     pts1 = PafTopDownViewPointSet()
    #     pts1.label = "signals"
    #     pts1.points = [
    #         self._global_path[s.index]
    #         for s in signals
    #         if s.type in SpeedCalculator.ROLLING_EVENTS + SpeedCalculator.QUICK_BRAKE_EVENTS
    #     ]
    #     pts1.color = [255, 0, 0]
    #     self._sign_publisher.publish(pts1)
    #
    #     pts2 = PafTopDownViewPointSet()
    #     pts2.label = "speed_signs"
    #     pts2.points = [
    #         self._global_path[s.index]
    #         for s in signals
    #         if s.type == TrafficSignIDGermany.MAX_SPEED.value or s.type == "MERGE"
    #     ]
    #     pts2.color = (255, 204, 0)
    #     self._sign_publisher.publish(pts2)
    #
    #     out = []
    #     for s in signals:
    #         if self._current_point_index <= s.index < end_idx:
    #             try:
    #                 n = TrafficSignIDGermany(s.type).name
    #             except Exception:
    #                 n = s.type
    #             m = self._distances[s.index] - self._distances[self._current_point_index]
    #             m = np.round(m, 1)
    #             str_signal = f"{n} ({m}m)"
    #             if s.value >= 0:
    #                 v = np.round(s.value, 1)
    #                 if n == "MAX_SPEED":
    #                     v = np.round(v * 3.6, 1)
    #                 str_signal += f": {v}"
    #             out.append(str_signal)
    #     if len(out) > 0:
    #         rospy.loginfo_throttle(2, f"[local planner] Upcoming Traffic Signs: {', '.join(out)}")
    #
    # def _speed_debug_print(self, speeds, number_of_values=100):
    #     if len(self._distances) == 0:
    #         return
    #     step = self._distances[-1] / len(self._distances)
    #     delta_m = 1  # meter
    #     delta_idx = int(delta_m / step)
    #     n = number_of_values * delta_idx
    #     out = []
    #     for speed in speeds[:n:delta_idx]:
    #         msg = f"{np.round(speed * 3.6, 1)}"
    #         out.append(msg)
    #     if len(out) > 0:
    #         rospy.loginfo_throttle(1, f"[local planner] Upcoming Speeds: {', '.join(out)}")

    def _allowed_from_stop(self):
        return True or self._can_continue_on_path(None)  # todo

    def _can_continue_on_path(self, signal_type):
        if signal_type is None:
            return True
        if signal_type == "LIGHT":
            return self._traffic_light_color == TrafficLightState.GREEN
        return True  # todo joining lanes / lane change free ??

    def _is_stopped(self):
        margin = 0.5
        stop = -margin < self._current_speed < margin
        return stop

    def _planner_at_end_of_local_path(self):
        if len(self._local_path) == 0:
            return True
        return 300 <= len(self._local_path) - self._local_path_idx < 1000 or (
            len(self._local_path) - self._local_path_idx < 300
            and not dist_pts(self._local_path.message.points[-1], self._global_path.route.sections[-1].points[0])
            < self.END_OF_ROUTE_REACHED_DIST
        )

    def _planner_at_end_of_global_path(self):
        if len(self._global_path) == 0:
            return False
        return (
            len(self._global_path) > 0
            and dist_pts(self._global_path.target, self._current_pose.position) <= self.END_OF_ROUTE_REACHED_DIST
            or dist_pts(self._global_path.route.sections[-1].points[0], self._current_pose.position)
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

    def _publish_speed_msg(self):
        if 0 <= self._local_path_idx < len(self._local_path):
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
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new global route")
            self._reroute_publisher.publish(Empty())

    def _send_standard_loop_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new standard loop route")
            self._reroute_standard_loop_publisher.publish(Empty())

    def _send_random_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new random global route")
            self._reroute_random_publisher.publish(Empty())

    def _end_of_route_handling(self):
        self._global_path = GlobalPath()
        rospy.Publisher("/paf/paf_validation/score/stop", Empty, queue_size=1).publish(Empty())
        if self._current_speed < 1:
            # self._send_random_global_path_request()  # todo remove in production
            self._send_standard_loop_request()  # todo remove in production

    def start(self):
        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            self._loop_handler()
            rate.sleep()

    def _on_local_path(self):
        return self._distance_to_local_path < 15


if __name__ == "__main__":
    LocalPlanner().start()
