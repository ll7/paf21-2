#!/usr/bin/env python
import time
from typing import List, Tuple

from commonroad.scenario.traffic_sign import (
    TrafficLightState,
    TrafficSignIDGermany,
)

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLocalPath, Point2D as Point, PafLaneletRoute, PafTrafficSignal, PafTopDownViewPointSet
from classes.SpeedCalculator import SpeedCalculator
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    REPLANNING_THRES_DISTANCE_M = 15
    DETECT_BEHIND_SIGNALS_M = 5
    TRANSMIT_FRONT_MIN_M = 100
    TRANSMIT_FRONT_SEC = 5
    DIST_TARGET_REACHED = 5
    UPDATE_HZ = 10
    REPLAN_THROTTLE_SEC = 5
    END_OF_ROUTE_SPEED = 5  # todo remove slowdown at end of route

    def __init__(self):

        rospy.init_node("local_path_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        self._rules_enabled = False

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._current_yaw = 0
        self._current_point_index = 0

        self._speed_limit = 50 / 3.6

        self._global_path = []
        self._distances = []
        self._curve_speed = []
        self._traffic_signals = []
        self._traffic_light_color = TrafficLightState.GREEN  # or None # todo fill this
        self._following_distance = -1  # todo set following distance
        self._following_speed = -1  # todo set following speed
        self._distance_to_global_path = 0
        self._is_at_end_of_route = True

        # local path params
        self._local_path = []
        self._target_speed = []

        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated)
        rospy.Subscriber("/paf/paf_global_planner/routing_response", PafLaneletRoute, self._process_global_path)
        self._last_replan_request = time.perf_counter()
        # create and start the publisher for the local path
        self._local_plan_publisher = rospy.Publisher("/paf/paf_local_planner/path", PafLocalPath, queue_size=1)
        self._reroute_publisher = rospy.Publisher("/paf/paf_local_planner/reroute", Empty, queue_size=1)
        self._reroute_random_publisher = rospy.Publisher(
            "/paf/paf_local_planner/routing_request_random", Empty, queue_size=1
        )
        self._sign_publisher = rospy.Publisher(
            "/paf/paf_validation/draw_map_points", PafTopDownViewPointSet, queue_size=1
        )

    def _process_global_path(self, msg: PafLaneletRoute):
        if len(self._distances) == 0 or len(msg.distances) == 0 or msg.distances[-1] != self._distances[-1]:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distances[-1])}m")
        self._global_path = msg.points
        self._distances = msg.distances
        self._curve_speed = np.array(msg.curve_speed)
        self._curve_speed[-1] = self.END_OF_ROUTE_SPEED
        self._traffic_signals = msg.traffic_signals
        self._local_plan_publisher.publish(self._create_paf_local_path_msg())

    def _get_current_path(self, start_idx):
        if len(self._global_path) == 0:
            self._local_path = []
            self._distances_delta = 0.01
            return 0, 0
        self._distances_delta = self._distances[-1] / len(self._distances)
        try:
            travel_dist = max(self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * self._current_speed)
            end_idx = int(np.ceil(travel_dist / self._distances_delta)) + start_idx
            _ = self._distances[end_idx]
        except IndexError:
            end_idx = len(self._distances)
        self._local_path = self._global_path[start_idx:end_idx]
        # rospy.logwarn(f"P {start_idx}-{end_idx} {end_idx-start_idx} {l1}")
        return end_idx

    @staticmethod  # todo change to np.hypot()
    def _dist(a, b):
        x1, y1 = a
        x2, y2 = b
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _closest_index_of_point_list(self, pts_list: List[Point], target_pt: Tuple[float, float], acc: int = 1):
        if len(pts_list) == 0:
            return -1, -1
        distances = [self._dist([p.x, p.y], target_pt) for p in pts_list[::acc]]
        if len(distances) == 0:
            return len(pts_list) - 1
        idx = int(np.argmin(distances))
        return idx * acc, distances[idx]

    def _create_paf_local_path_msg(self, send_empty=False):
        """create path message for ros
        Returns:
            [type]: [description]
        """
        self._get_current_trajectory()
        path_msg = PafLocalPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.target_speed = [] if send_empty else self._target_speed
        path_msg.points = [] if send_empty else self._local_path
        return path_msg

    def _get_current_trajectory(self):
        index = self._set_current_point_index()
        end_idx = self._get_current_path(index)
        self._update_target_speed(index, end_idx)

    def _signal_debug_print(self, signals):
        end_idx = self._current_point_index + len(self._local_path)

        pts = PafTopDownViewPointSet()
        pts.label = "signals"
        pts.points = [self._global_path[s.index] for s in signals]
        pts.color = [255, 0, 0]
        self._sign_publisher.publish(pts)

        out = []
        for s in signals:
            if self._current_point_index <= s.index < end_idx:
                try:
                    n = TrafficSignIDGermany(s.type).name
                except Exception:
                    n = s.type
                m = self._distances[s.index] - self._distances[self._current_point_index]
                m = np.round(m, 1)
                str_signal = f"{n} ({m}m)"
                if s.value >= 0:
                    v = np.round(s.value, 1)
                    str_signal += f": {v}"
                out.append(str_signal)
        if len(out) > 0:
            rospy.loginfo_throttle(20, f"Upcoming Traffic Signs: {', '.join(out)}")

    def _speed_debug_print(self, speeds, number_of_values=20):
        if len(self._distances) == 0:
            return
        step = self._distances[-1] / len(self._distances)
        delta_m = 1  # meter
        delta_idx = int(delta_m / step)
        n = number_of_values * delta_idx
        out = []
        for speed in speeds[:n:delta_idx]:
            msg = f"{np.round(speed * 3.6, 1)}"
            out.append(msg)
        if len(out) > 0:
            rospy.loginfo_throttle(20, f"Upcoming Speeds: {', '.join(out)}")

    def _update_target_speed(self, start_idx, end_idx):
        if len(self._global_path) == 0:
            return 0, 0
        calc = SpeedCalculator(self._distances, start_idx, end_idx)
        signals: List[PafTrafficSignal] = self._traffic_signals
        self._signal_debug_print(signals)
        speed = self._curve_speed[start_idx:end_idx]
        if self._rules_enabled:
            speed = calc.add_speed_limits(speed, signals)
            speed = calc.add_stop_events(speed, signals, target_speed=2)
            speed = calc.add_roll_events(speed, signals, target_speed=2)
        speed = calc.add_linear_deceleration(speed)
        self._speed_debug_print(speed)
        self._target_speed = speed

    def _can_continue_on_path(self, signal_type):
        if signal_type == "LIGHT":
            return self._traffic_light_color == TrafficLightState.GREEN
        return True  # todo joining lanes / lane change free ??

    def _is_stopped(self):
        margin = 0.5
        stop = -margin < self._current_speed < margin

        rospy.loginfo_throttle(5, stop)
        return stop

    def _planner_at_end_of_route(self):
        return len(self._local_path) < 50

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
        # invert y-coordinate of odometry, because odometry sensor returns wrong values
        self._current_pose.position.y = -self._current_pose.position.y

    def _set_current_point_index(self):
        if len(self._global_path) == 0:
            self._current_point_index = 0
        else:
            current_pos = self._current_pose.position
            start_idx = max(0, self._current_point_index - 100)
            end_idx = min(len(self._global_path), self._current_point_index + 100)
            pts = self._global_path[start_idx:end_idx]
            ref = (current_pos.x, current_pos.y)
            found_idx, distance = self._closest_index_of_point_list(pts, ref, acc=10)
            if found_idx > 0 and distance < self.REPLANNING_THRES_DISTANCE_M:
                self._current_point_index = found_idx + start_idx
            else:
                found_idx, _ = self._closest_index_of_point_list(self._global_path, ref, acc=100)
                self._current_point_index = found_idx
        return self._current_point_index

    def _on_global_path(self):
        p = (self._current_pose.position.x, self._current_pose.position.y)
        i = self._current_point_index
        try:
            self._distance_to_global_path = self._dist(p, (self._global_path[i].x, self._global_path[i].y))
            return self._distance_to_global_path < self.REPLANNING_THRES_DISTANCE_M
        except IndexError:
            return False

    def _send_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new global route")
            self._reroute_publisher.publish(Empty())

    def _send_random_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new random global route")
            self._reroute_random_publisher.publish(Empty())

    def _end_of_route_handling(self):
        if self._current_speed < 5:
            self._send_random_global_path_request()  # todo remove in production

    def start(self):

        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            if not self._on_global_path():
                self._local_plan_publisher.publish(self._create_paf_local_path_msg(send_empty=True))
                self._send_global_path_request()
            elif self._planner_at_end_of_route():
                self._end_of_route_handling()
            else:
                rospy.loginfo_throttle(30, "[local planner] car is on route, no need to reroute")
                self._local_plan_publisher.publish(self._create_paf_local_path_msg())
            rate.sleep()


if __name__ == "__main__":
    LocalPlanner().start()
