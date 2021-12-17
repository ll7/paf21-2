#!/usr/bin/env python
import time

from commonroad.scenario.traffic_sign import (
    TrafficLightState,
    TrafficSignIDGermany,
)

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import (
    PafLocalPath,
    PafLaneletRoute,
    PafTrafficSignal,
    PafTopDownViewPointSet,
    Point2D,
    PafSpeedMsg,
)
from classes.SpeedCalculator import SpeedCalculator
from classes.HelperFunctions import (
    dist,
    closest_index_of_point_list,
    get_angle_between_vectors,
    xy_from_distance_and_angle,
)
from classes.Spline import calc_spline_course
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
    MAX_ANGULAR_ERROR = np.deg2rad(45)

    rules_enabled = rospy.get_param("rules_enabled", False)

    def __init__(self):

        rospy.init_node("local_path_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        rospy.logwarn(f"[local planner] Rules are {'en' if self.rules_enabled else 'dis'}abled!")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._current_yaw = 0
        self._current_point_index = 0
        self._local_path_end_index = 0

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
        self._last_local_reroute = rospy.Time.now().to_time()
        self._speed_msg = PafSpeedMsg()

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
        self._speed_msg_publisher = rospy.Publisher("/paf/paf_validation/speed_text", PafSpeedMsg, queue_size=1)

    def _process_global_path(self, msg: PafLaneletRoute):
        if len(self._distances) == 0 or len(msg.distances) == 0 or msg.distances[-1] != self._distances[-1]:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distances[-1])}m")
        self._global_path = msg.points
        self._distances = msg.distances
        self._curve_speed = np.array(msg.curve_speed)
        self._curve_speed[-1] = self.END_OF_ROUTE_SPEED
        self._traffic_signals = msg.traffic_signals
        self._create_paf_local_path_msg()

    def _get_track_angle(self, index):
        if len(self._global_path) == 0:
            return 0
        if index == len(self._global_path) - 1:
            index -= 1
        p1 = self._global_path[index]
        p2 = self._global_path[index + 1]
        v2 = [p2.x - p1.x, p2.y - p1.y]
        return get_angle_between_vectors(v2)

    def _get_turning_path_to(self, index, track_angle, track_err, distance):
        def is_left_of_target_line():
            a = p_target_1
            b = p_target_2
            c = self._current_pose.position
            return (b.x - a.x) * (c.y - a.y) > (b.y - a.y) * (c.x - a.x)

        index_forwards = int(min(len(self._global_path) - 1, index + 10 / 0.125))
        p_target_1 = self._global_path[index_forwards - 1]
        p_target_2 = self._global_path[index_forwards]

        # d1, d2, radius = 4, 10, -5
        # turning_dir = 1

        x_list, y_list = [], []
        pts = [
            xy_from_distance_and_angle(self._current_pose.position, 0, -self._current_yaw),
            xy_from_distance_and_angle(self._current_pose.position, 4, -self._current_yaw),
            xy_from_distance_and_angle(p_target_1, 5, track_angle + np.pi / 2),
        ]

        for pt in pts:
            x_list.append(pt[0])
            y_list.append(pt[1])
        x_list = x_list[:2] + [p_target_1.x] + x_list[2:]  # , p_target_2.x]
        y_list = y_list[:2] + [p_target_1.y] + y_list[2:]  # , p_target_2.y]

        # pts_count = 10
        # turn_rads = .5 * np.pi
        # for i in range(int(.2 * pts_count), pts_count + 1):
        #     x = p_target_1.x + radius * np.cos(track_angle - np.pi / 2 + turn_rads * turning_dir * i / pts_count)
        #     y = p_target_1.y - radius * np.sin(track_angle - np.pi / 2 + turn_rads * turning_dir * i / pts_count)
        #     x_list.append(x)
        #     y_list.append(y)
        # fx2a = self._current_pose.position.x + d1 * np.cos(-self._current_yaw)
        # fy2a = self._current_pose.position.y - d2* np.sin(-self._current_yaw)
        # fx1a = self._current_pose.position.x + d2 * np.cos(-self._current_yaw)
        # fy1a = self._current_pose.position.y - d2 * np.sin(-self._current_yaw)
        # # fx2a = self._current_pose.position.x + radius * np.cos(-self._current_yaw + np.pi * 1.33 * turning_dir)
        # fy2a = self._current_pose.position.y - radius * np.sin(-self._current_yaw + np.pi * 1.33 * turning_dir)
        # fx2b = self._current_pose.position.x + radius * np.cos(-self._current_yaw + np.pi * 1.66 * turning_dir)
        # fy2b = self._current_pose.position.y - radius * np.sin(-self._current_yaw + np.pi * 1.66 * turning_dir)
        # fx3a = self._current_pose.position.x + d2 * np.cos(-self._current_yaw + turning_dir * .4)
        # fy3a = self._current_pose.position.y - d2 * np.sin(-self._current_yaw + turning_dir * .4)
        # fx3b = fx3a + d2 * np.cos(-self._current_yaw + turning_dir * np.pi / 2)
        # fy3b = fy3a - d2 * np.sin(-self._current_yaw + turning_dir * np.pi / 2)

        # x_list += [p_target_1.x, p_target_2.x]
        # y_list += [p_target_1.y, p_target_2.y]

        x_list_out, y_list_out, _, _, _ = calc_spline_course(x_list, y_list, 0.125)

        self._local_path = [Point2D(x, y) for x, y in zip(x_list_out, y_list_out)]

        self._target_speed = [1 for _ in self._local_path]

    def _get_current_path(self, start_idx, distance):
        if len(self._global_path) == 0:
            self._local_path = []
            self._distances_delta = 0.01
            return self._local_path, self._target_speed
        track_angle = self._get_track_angle(start_idx)
        track_err = track_angle - self._current_yaw
        while track_err > np.pi:
            track_err -= 2 * np.pi
        if self.REPLANNING_THRES_DISTANCE_M > distance > 3 or np.abs(track_err) > self.MAX_ANGULAR_ERROR:
            # self._get_turning_path_to(start_idx, track_angle, track_err, distance)
            rospy.logwarn_throttle(10, f"off track! deg={np.rad2deg(track_err)}, d={distance}")
            # self._local_path = []
            # self._target_speed = [3 for _ in self._local_path]
            # return

        self._distances_delta = self._distances[-1] / len(self._distances)
        try:
            travel_dist = max(self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * self._current_speed)
            end_idx = int(np.ceil(travel_dist / self._distances_delta)) + start_idx
            _ = self._distances[end_idx]  # index error check
        except IndexError:
            end_idx = len(self._distances)

        end_idx = min(end_idx, len(self._global_path))
        self._local_path = self._global_path[start_idx:end_idx]
        self._local_path_end_index = end_idx - 1
        sp = self._update_target_speed(start_idx, end_idx)
        return self._local_path, sp

    def _create_paf_local_path_msg(self, send_empty=False):
        """create path message for ros
        Returns:
            [type]: [description]
        """
        pth, speeds, is_new_msg = self._get_current_trajectory()

        if is_new_msg:
            path_msg = PafLocalPath()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.target_speed = [] if send_empty else list(speeds)
            path_msg.points = [] if send_empty else pth

            while 0 < len(path_msg.target_speed) < len(path_msg.points):  # threading issue
                path_msg.target_speed.append(path_msg.target_speed[-1])

            self._local_plan_publisher.publish(path_msg)

    def _get_current_trajectory(self):
        index, distance = self._set_current_point_index()
        last_local_reroute = rospy.Time.now().to_time()
        if len(self._local_path) > 0 and self._planner_at_end_of_route(self._local_path):
            self._end_of_route_handling()
            self._local_path = []
            self._target_speed = []
            self._global_path = []
            is_new_message = True
        elif len(self._global_path) > 0 and (
            last_local_reroute - self._last_local_reroute > self.REPLAN_THROTTLE_SEC / 2
            or index + 300 > self._local_path_end_index
        ):
            self._get_current_path(index, distance)
            is_new_message = True
            rospy.loginfo_throttle(10, "local planner is replanning")
            self._last_local_reroute = last_local_reroute
        else:
            is_new_message = False
        return self._local_path, self._target_speed, is_new_message

    def _signal_debug_print(self, signals):
        end_idx = self._current_point_index + len(self._local_path)

        # pts = PafTopDownViewPointSet()
        # pts.label = "signals"
        # pts.points = [
        #     self._global_path[s.index]
        #     for s in signals
        #     if s.type in (SpeedCalculator.QUICK_BRAKE_EVENTS + SpeedCalculator.ROLLING_EVENTS)
        # ]
        # pts.color = [255, 0, 0]
        # self._sign_publisher.publish(pts)

        pts = PafTopDownViewPointSet()
        pts.label = "speed_signs"
        pts.points = [self._global_path[s.index] for s in signals if s.type == TrafficSignIDGermany.MAX_SPEED.value]
        pts.color = (255, 204, 0)
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
                    if n == "MAX_SPEED":
                        v = np.round(v * 3.6, 1)
                    str_signal += f": {v}"
                out.append(str_signal)
        if len(out) > 0:
            rospy.loginfo_throttle(2, f"Upcoming Traffic Signs: {', '.join(out)}")

    def _speed_debug_print(self, speeds, number_of_values=100):
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
            rospy.loginfo_throttle(1, f"Upcoming Speeds: {', '.join(out)}")

    def _update_target_speed(self, start_idx, end_idx):
        if len(self._global_path) == 0:
            return 0
        calc = SpeedCalculator(self._distances, start_idx, end_idx)
        self._signal_debug_print(self._traffic_signals)
        speed = self._curve_speed[start_idx:end_idx]
        if self.rules_enabled:
            speed = calc.add_stop_events(speed, self._traffic_signals, target_speed=0, buffer_m=0.5)
            speed = calc.add_roll_events(speed, self._traffic_signals, target_speed=0, buffer_m=0.5)
        if self._current_speed < 1e-3 and self._allowed_from_stop():
            rospy.sleep(1)
            speed = calc.remove_stop_event(speed, buffer_m=10)

        pts = PafTopDownViewPointSet()
        pts.label = "low_speed"
        pts.points = [self._global_path[i + start_idx] for i, s in enumerate(speed) if s < 0.1]
        pts.color = (255, 0, 255)
        self._sign_publisher.publish(pts)

        speed = calc.add_linear_deceleration(speed)
        # self._speed_debug_print(speed)
        self._target_speed = speed
        self._publish_speed_msg(start_idx, self._curve_speed)
        return speed

    def _allowed_from_stop(self):
        return True  # todo

    def _can_continue_on_path(self, signal_type):
        if signal_type == "LIGHT":
            return self._traffic_light_color == TrafficLightState.GREEN
        return True  # todo joining lanes / lane change free ??

    def _is_stopped(self):
        margin = 0.5
        stop = -margin < self._current_speed < margin

        rospy.loginfo_throttle(5, stop)
        return stop

    def _planner_at_end_of_route(self, pth=None):
        if pth is not None:
            return len(pth) < 100
        self._is_at_end_of_route = self._local_path_end_index - self._current_point_index < 100
        return self._is_at_end_of_route

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

    def _set_current_point_index(self):
        if len(self._global_path) == 0:
            self._current_point_index = 0
            distance = 0
        else:
            current_pos = self._current_pose.position
            start_idx = max(0, self._current_point_index - 300)
            end_idx = min(len(self._global_path), self._current_point_index + 300)
            pts = self._global_path[start_idx:end_idx]
            ref = (current_pos.x, current_pos.y)
            found_idx, distance = closest_index_of_point_list(pts, ref, acc=30)
            if found_idx > 0 and distance < self.REPLANNING_THRES_DISTANCE_M * 1:
                self._current_point_index = found_idx + start_idx
            else:
                found_idx, distance = closest_index_of_point_list(self._global_path, ref, acc=100)
                self._current_point_index = found_idx
        return self._current_point_index, distance

    def _publish_speed_msg(self, idx, speed):
        # idx = self._current_point_index + 100
        try:
            limit = self._speed_msg.limit if self._speed_msg.limit > 0 else 250
            sign: PafTrafficSignal
            for sign in self._traffic_signals:
                if sign.index > idx:
                    break
                if sign.type == TrafficSignIDGermany.MAX_SPEED.value:
                    limit = sign.value
                    # rospy.logerr_throttle(1, f"{TrafficSignIDGermany(sign.type).name} {limit*3.6}")
            target = speed[idx]
            if limit != self._speed_msg.limit or target != self._speed_msg.target:
                self._speed_msg.limit = limit
                self._speed_msg.target = target
                self._speed_msg_publisher.publish(self._speed_msg)
        except IndexError:
            pass

    def _on_global_path(self):
        p = (self._current_pose.position.x, self._current_pose.position.y)
        i = self._current_point_index
        try:
            self._distance_to_global_path = dist(p, (self._global_path[i].x, self._global_path[i].y))
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
            if self._planner_at_end_of_route():
                self._end_of_route_handling()
            elif not self._on_global_path():
                self._create_paf_local_path_msg(send_empty=True)
                self._send_global_path_request()
            else:
                rospy.loginfo_throttle(30, "[local planner] car is on route, no need to reroute")
                self._create_paf_local_path_msg()
            rate.sleep()


if __name__ == "__main__":
    LocalPlanner().start()
