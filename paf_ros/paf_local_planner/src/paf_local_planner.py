#!/usr/bin/env python
import time
from typing import List, Tuple, Dict

from commonroad.scenario.traffic_sign import (
    TrafficSignIDGermany as SignsDE,
    # TrafficSignIDUsa as SignsUS,
    TrafficLightState,
)

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from paf_messages.msg import PafLocalPath, Point2D as Point, PafLaneletRoute
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/CubicSpline/cubic_spline_planner.py

"""
import math
import bisect


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


class LocalPlanner:
    """class used for the local planner. Task: return a local path"""

    REPLANNING_THRES_DISTANCE_M = 15
    DETECT_BEHIND_SIGNALS_M = 5
    TRANSMIT_FRONT_MIN_M = 100
    TRANSMIT_FRONT_SEC = 5
    CAR_DECELERATION = -9.81 * 0.1
    # rospy.get_param("deceleration_g_factor", 8)  # m/s^2
    QUICK_BRAKE_EVENTS = [SignsDE.STOP.value]
    ROLLING_EVENTS = ["LIGHT", SignsDE.YIELD.value]
    DIST_TARGET_REACHED = 5
    UPDATE_HZ = 10
    REPLAN_THROTTLE_SEC = 3.0

    def __init__(self):

        rospy.init_node("local_path_node", anonymous=True)
        role_name = rospy.get_param("~role_name", "ego_vehicle")

        self._current_pose = Pose()
        self._current_speed = 0
        self._current_pitch = 0
        self._current_yaw = 0
        self._current_point_index = 0
        self._speed_limit = 250
        self._last_replan_request = time.perf_counter()
        self._global_path = []
        self._distances = []
        self._traffic_signals = []
        self._traffic_light_color = TrafficLightState.GREEN  # or None # todo fill this
        self._following_distance = -1  # todo set following distance
        self._following_speed = -1  # todo set following speed
        self.dist = 0

        # local path params
        self._local_path = []
        self._local_path_signals = {}
        self._target_speed = []

        rospy.Subscriber(f"carla/{role_name}/odometry", Odometry, self._odometry_updated)
        rospy.Subscriber("/paf/paf_global_planner/routing_response", PafLaneletRoute, self._process_global_path)
        # rospy.Subscriber(rospy.get_param("global_path_topic"), PafLaneletRoute, self._process_global_path)

        # create and start the publisher for the local path
        self._local_plan_publisher = rospy.Publisher("/paf/paf_local_planner/path", PafLocalPath, queue_size=1)
        self._reroute_publisher = rospy.Publisher("/paf/paf_local_planner/reroute", Empty, queue_size=1)

    def _current_deceleration_distance(self, target_speed=0):
        deceleration = self.CAR_DECELERATION - 9.81 * np.sin(self._current_pitch)
        deceleration_time = (target_speed - self._current_speed) / deceleration
        delay = self._current_speed / self.UPDATE_HZ
        return 0.5 * (target_speed + self._current_speed) * deceleration_time + delay

    def _process_global_path(self, msg: PafLaneletRoute):
        if len(self._distances) == 0 or len(msg.distances) == 0 or msg.distances[-1] != self._distances[-1]:
            rospy.loginfo_throttle(5, f"[local planner] receiving new route len={int(msg.distances[-1])}m")
        self._global_path = msg.points
        self._distances = msg.distances
        self._traffic_signals = msg.traffic_signals
        self._local_plan_publisher.publish(self._create_ros_msg())

    def _get_current_path(self) -> Tuple[List[Point], Dict[str, Tuple[int, float]]]:
        if len(self._global_path) == 0:
            self._local_path, self._local_path_signals = [], {}
            return
        index = self._current_point_index
        try:
            d_ref = self._distances[index]
        except IndexError:
            d_ref = 0
        try:
            delta = self._distances[index + 1] - d_ref
            travel_dist = max(self.TRANSMIT_FRONT_MIN_M, self.TRANSMIT_FRONT_SEC * self._current_speed)
            index_end = int(np.ceil(travel_dist / delta)) + index
            _ = self._distances[index_end]
        except IndexError:  # todo end of route handling
            # if (
            #         self._dist(
            #             (self._global_path[-1].x, self._global_path[-1].y),
            #             (self._current_pose.position.x, self._current_pose.position.y),
            #         )
            #         > self.DIST_TARGET_REACHED
            # ):
            #     self._send_global_path_request()
            index_end = len(self._distances)

        signals = {}
        for s in self._traffic_signals:
            if s.index < index and d_ref - self._distances[s.index] > self.DETECT_BEHIND_SIGNALS_M:
                continue
            if s.index > index_end:
                continue
            if s.type in signals:
                continue
            signals[s.type] = (self._distances[s.index] - d_ref, s.value)

        current_path = self._global_path[index:index_end]
        # spline_split = min(40, int(self._current_speed + 30)), int(200 + self._current_speed), 20
        # a, b, c = spline_split
        # self.spline_pts = int((b - a) / c)
        self._local_path, self._local_path_signals = current_path, signals

    # def _add_spline(self, pth, spline_split):
    #     if len(pth) < 200 or self.dist < 0:
    #         return pth
    #     a, b, c = spline_split
    #     my_position = (self._current_pose.position.x, self._current_pose.position.y)
    #     mx, my = my_position
    #     # sin, cos = np.sin(-self._current_yaw), np.cos(-self._current_yaw)
    #     # param = 1
    #     # ax = [mx, mx+param*cos] + \
    #     #    [x.x for x in pth[100:200:20]]
    #     # ay = [my, my+param*sin] + \
    #     #    [x.y for x in pth[100:200:20]]
    #     # cx, cy, _, _, _ = calc_spline_course(ax, ay, ds=0.1)
    #     # return self._xy_to_point2d([[x, y] for x, y in zip(cx, cy)]) + pth[200:]
    #     # pth = pth[a:b:c] + pth[b:]
    #     return pth

    def _xy_to_point2d(self, points):
        liste = []
        for point in points:
            p = Point()
            p.x, p.y = point
            liste.append(p)
        return liste

    def _closest_index_in_path(self) -> int:
        """
        returns the closest index of a point in the local path to the vehicle's current position
        :return index
        """
        my_position = (self._current_pose.position.x, self._current_pose.position.y)
        return self._closest_index_of_point_list(self._global_path[::100], my_position)

    @staticmethod
    def _dist(a, b):
        x1, y1 = a
        x2, y2 = b
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _closest_index_of_point_list(self, pts_list: List[Point], target_pt: Tuple[float, float]):

        if len(self._global_path) == 0:
            return -1
        return int(np.argmin([self._dist([p.x, p.y], target_pt) for p in pts_list]))

    def _create_ros_msg(self):
        """create path message for ros
        Returns:
            [type]: [description]
        """
        self._get_current_trajectory()
        path_msg = PafLocalPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.target_speed = self._target_speed
        path_msg.points = self._local_path
        return path_msg

    def _get_current_trajectory(self):
        self._get_current_path()
        self._update_target_speed()

    def _update_target_speed(self):
        signals = self._local_path_signals
        if self._planner_at_end_of_route():
            self._target_speed = 50 / 3.6
            rospy.loginfo_throttle(5, "speed is zero")
        else:
            target_speed = 50 / 3.6  # todo remove
            rospy.loginfo_throttle(5, f"speed is {self._current_speed * 3.6}/{target_speed * 3.6}")

        self._target_speed = list(np.ones((len(self._local_path),)) * target_speed)
        return
        braking_distance_zero = self._current_deceleration_distance(target_speed=0)
        rospy.loginfo_throttle(5, braking_distance_zero)
        signdetected = False
        for signal_type, (distance, value) in signals.items():
            should_stop_now = distance <= braking_distance_zero
            if signal_type in self.QUICK_BRAKE_EVENTS:
                if should_stop_now:
                    if self._is_stopped() and self._can_continue_on_path(signal_type):
                        self._target_speed = self._speed_limit
                    else:
                        self._target_speed = 0
                    signdetected = True

            elif signal_type in self.ROLLING_EVENTS:
                if should_stop_now:
                    if self._can_continue_on_path(signal_type):
                        self._target_speed = self._speed_limit
                    else:
                        self._target_speed = 0
                    signdetected = True

            elif signal_type == SignsDE.MAX_SPEED.value:
                braking_distance_to_speed_limit = self._current_deceleration_distance(target_speed=value)
                if distance <= braking_distance_to_speed_limit:
                    # m/s (km/h -> m/s is done in PafRoute)
                    self._speed_limit = value
                    signdetected = True

            else:
                rospy.logerr_throttle(
                    10,
                    f"[local planner] unknown signal detected: type={signal_type} at d={distance} "
                    f"(Check TrafficSignIDGermany or TrafficSignIDUsa for more information)",
                )
        if not signdetected:
            self._target_speed = self._speed_limit

        if self._following_distance > 0:
            reduce_speed = (
                self._current_deceleration_distance(target_speed=self._following_speed) <= self._following_distance
            )
            if reduce_speed:
                self._target_speed = self._following_speed

        rospy.logwarn_throttle_identical(10, f"speed should be {self._target_speed * 3.6}")
        if self._planner_at_end_of_route():
            self._target_speed = 0
            rospy.loginfo_throttle(5, "speed is zero")
        else:
            self._target_speed = min(80 / 3.6, self._target_speed)  # todo remove

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
        if len(self._global_path) < 100:
            return True
        return (
            self._dist(
                (self._current_pose.position.x, self._current_pose.position.y),
                (self._global_path[-100].x, self._global_path[-100].y),
            )
            < self.DIST_TARGET_REACHED
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
        # invert y-coordinate of odometry, because odometry sensor returns wrong values
        self._current_pose.position.y = -self._current_pose.position.y

        if len(self._global_path) <= self._current_point_index + 1:
            self._current_point_index = 0

        current_pos = self._current_pose.position

        # todo marcos algo (unten) ist schlechter wenn fahrzeug abseits von globaler routen-linie
        # prev_dist = None
        # calculate distance to current next point
        # idx = self._current_point_index
        # for i, _ in enumerate(self._global_path[idx:]):
        #     next_point = self._global_path[i + idx]
        #     distance = self._dist((current_pos.x, current_pos.y), (next_point.x, next_point.y))
        #     if prev_dist is None or prev_dist > distance:
        #         prev_dist = distance
        #     else:
        #         self._current_point_index = i + idx - 1
        #         break
        self._current_point_index = self._closest_index_of_point_list(self._global_path, (current_pos.x, current_pos.y))

        # self._current_point_index -= 20
        # self._current_point_index = min(0, self._current_point_index)

    def _on_global_path(self):
        p = (self._current_pose.position.x, self._current_pose.position.y)
        i = self._current_point_index
        try:
            self.dist = self._dist(p, (self._global_path[i].x, self._global_path[i].y))
            return self.dist < self.REPLANNING_THRES_DISTANCE_M
        except IndexError:
            return False

    def _send_global_path_request(self):
        t = time.perf_counter()
        delta_t = t - self._last_replan_request
        if self.REPLAN_THROTTLE_SEC < delta_t:
            self._last_replan_request = t
            rospy.loginfo_throttle(20, "[local planner] requesting new global route")
            self._reroute_publisher.publish(Empty())

    def start(self):

        rate = rospy.Rate(self.UPDATE_HZ)
        while not rospy.is_shutdown():
            if not self._on_global_path():
                self._send_global_path_request()
            else:
                rospy.loginfo_throttle(30, "[local planner] car is on route, no need to reroute")

            self._local_plan_publisher.publish(self._create_ros_msg())
            rate.sleep()


if __name__ == "__main__":
    LocalPlanner().start()

# rosservice call /paf_global_planner/routing_request "start:
# - -85.0
# - -75.0
# start_yaw: 1.56
# target:
# - -180.0
# - 180.0
# resolution: 0.0"
