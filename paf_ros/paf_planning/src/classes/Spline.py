"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/CubicSpline/cubic_spline_planner.py

"""
import math
import bisect
from typing import List, Union, Any, Optional

import numpy as np
from bezier import Curve

import rospy
from paf_messages.msg import Point2D
from .HelperFunctions import xy_to_pts, dist, pts_to_xy


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        """
        Constructor unchanged from source
        :param x: x values of input points
        :param y: y values of input points
        """
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_a_matrix(h)
        B = self.__calc_b_matrix(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calculate result
        :param t: if t is outside the input x, return None, else return resulting spline
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calc_derivative(self, t):
        """
        Calculate first derivative
        :param t: if t is outside the input x, return None, else return resulting function
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calc_derivative_2(self, t):
        """
        Calc second derivative
        :param t: if t is outside the input x, return None, else return resulting function
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

    def __calc_a_matrix(self, h):
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

    def __calc_b_matrix(self, h):
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
        """
        Convert list of points in A 2D spline
        :param x: x values
        :param y: y values
        """
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        """
        Calculate culminating sum
        :param x: x values
        :param y: y values
        :return: cum sum
        """
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
        dx = self.sx.calc_derivative(s)
        ddx = self.sx.calc_derivative_2(s)
        dy = self.sy.calc_derivative(s)
        ddy = self.sy.calc_derivative_2(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calc_derivative(s)
        dy = self.sy.calc_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course_from_point_list(
    xy_list: List[Union[Point2D, Any]], ds: float = 0.1, to_pts: bool = False
) -> Union[List[Point2D], np.ndarray]:
    """
    Calculate a 2D Spline from given list of points
    :param xy_list: list of points (list of tuples or objects with x,y attributes)
    :param ds: step size
    :param to_pts: convert result to List[Point2D]
    :return: list of Point2D or 2D numpy array
    """
    if len(xy_list) > 0 and hasattr(xy_list[0], "x"):
        xy_list = [(p.x, p.y) for p in xy_list]
    x, y = list(zip(*xy_list))
    assert len(x) == len(y)
    assert len(x) > 1
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry = [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        if np.any(np.isnan([ix, iy])):
            continue
        rx.append(ix)
        ry.append(iy)

    x_target, y_target = rx, ry
    out = np.array([np.array([x, y]) for x, y in zip(x_target, y_target)])
    if to_pts:
        return xy_to_pts(out)
    return out


def calc_bezier_curve(
    pts: List[Union[Point2D, Any]], ds: float = 0.1, convert_to_pts: bool = False
) -> Union[List[Point2D], List[np.ndarray]]:
    """
    Calculate a 2D Bezier Curve from given list of points with degree = len(pts)-1
    :param pts: list of points (list of tuples or objects with x,y attributes)
    :param ds: step size
    :param convert_to_pts: convert result to List[Point2D]
    :return: list of Point2D or 2D numpy array
    """

    def _calc_bezier_curve(_x, _y, degree=2):
        nodes = np.asfortranarray([_x, _y])
        return Curve(nodes, degree=degree)

    def _calc(pts2, _ds=ds):
        _x, _y = [], []
        for pt in pts2:
            try:
                _x.append(pt[0])
                _y.append(pt[1])
            except TypeError:
                _x.append(pt.x)
                _y.append(pt.y)
        valid = None
        k = 1
        for k in range(k, len(_x)):
            valid = not np.any(np.isnan(_calc_bezier_curve(_x[::k], _y[::k], degree=len(_x[::k]) - 1).evaluate(0.0)))
            if valid:
                break
        if not valid:
            raise ValueError(f"ERROR, unable to calc bezier\n{pts2}")
        if k > 2:
            msg = f"[local planner] bezier calculation is limited to every {k}th item of list (len={len(pts)})"
            try:
                rospy.logwarn_throttle(1, msg)
            except rospy.ROSInitException:
                print(msg)
        _x = _x[::k]
        _y = _y[::k]
        curve = _calc_bezier_curve(_x, _y, degree=len(_x) - 1)
        s_vals = np.linspace(0.0, 1.0, num_pts)
        r_vals = curve.evaluate_multi(s_vals)
        return r_vals

    out_pts = []
    distances = 0
    for p1, p2 in zip(pts, pts[1:]):
        distances += dist(p1, p2)
    num_pts = int(np.ceil(distances / ds))
    x_arr, y_arr = _calc(pts)
    for x, y in zip(x_arr, y_arr):
        out_pts.append(np.array([x, y]))
    if convert_to_pts:
        return xy_to_pts(out_pts)
    return out_pts


def bezier_refit_all_with_tangents(
    pts_list: List[Union[Point2D, Any]], ds1: float = 0.1, ds2: Optional[float] = None, convert_to_pts: bool = True
):
    """
    Calculate a 2D Bezier Curve given the tangent of the previous two points and the next two points (step A).
    After this calculation, a Bezier Curve is calculated from the resulting points with degree n-1,
    if ds2 is given (step B)
    :param pts_list: list of points (list of tuples or objects with x,y attributes)
    :param ds1: step size for bezier calculation (for step A)
    :param ds2: step size 2 (optional for step B)
    :param convert_to_pts: convert result to List[Point2D]
    :return: list of Point2D or 2D numpy array
    """
    if len(pts_list) < 2:
        return pts_list
    try:
        out_pts = calc_bezier_curve(pts_to_xy(pts_list[:2]))
        is_pts = True
    except AttributeError:
        out_pts = calc_bezier_curve(pts_list[:2])
        is_pts = False
    if len(out_pts) < 2:
        return out_pts
    for i, pts in enumerate(zip(pts_list, pts_list[1:], pts_list[2:], pts_list[3:])):
        pts = list(pts)
        if is_pts:
            pts[0] = xy_to_pts([out_pts[-2]])[0]
        else:
            pts[0] = out_pts[-2]
        try:
            bezier = bezier_refit_with_tangents(*pts, ds=ds1)[:-1]
        except ValueError:
            if is_pts:
                bezier = pts_to_xy(pts[1:-1])
            else:
                bezier = pts[1:-1]
        out_pts += bezier

    # remaining two points at the end
    if is_pts:
        out_pts += calc_bezier_curve(pts_to_xy(pts_list[-2:]), ds=ds1)
    else:
        out_pts += calc_bezier_curve(pts_list[-2:], ds=ds1)

    # more smoothing if wished
    if ds2 is not None:
        out_pts = calc_bezier_curve(out_pts, ds=ds2)

    if convert_to_pts:
        return xy_to_pts(out_pts)
    return out_pts


def bezier_refit_with_tangents(lane1_p1, lane1_p2, lane2_p1, lane2_p2, ds=0.1, alpha=1.0, fraction=0.99):
    """
    Calculate a Bezier Curve given four points. Example: think of a lane change.
    The first two points are on the left (0,0) and (0,1), the other two on the right lane in front (1,2) and (1,3).
    Goal of the algorithm is to find a curve, that is in (0,0) tangential to the line between the first two and in (1,3)
    tangential to the second two points.
    :param lane1_p1: p1.1 (see function description)
    :param lane1_p2: p1.2 (see function description)
    :param lane2_p1: p2.3 (see function description)
    :param lane2_p2: p2.4 (see function description)
    :param ds: step size
    :param alpha: exponent weight for the distance between the points
    :param fraction: [0-1] lower=more smooth, higher=less smooth
    :return:
    """
    try:
        lane1_p1 = np.array([lane1_p1.x, lane1_p1.y])
        lane1_p2 = np.array([lane1_p2.x, lane1_p2.y])
        lane2_p1 = np.array([lane2_p1.x, lane2_p1.y])
        lane2_p2 = np.array([lane2_p2.x, lane2_p2.y])
    except AttributeError:
        lane1_p1 = np.array(lane1_p1)
        lane1_p2 = np.array(lane1_p2)
        lane2_p1 = np.array(lane2_p1)
        lane2_p2 = np.array(lane2_p2)

    # calculate intermediate point
    lane1_p1 = np.sum([lane1_p2 * fraction, lane1_p1 * (1 - fraction)], axis=0)
    lane2_p2 = np.sum([lane2_p1 * fraction, lane2_p2 * (1 - fraction)], axis=0)

    d1 = dist(lane1_p2, lane1_p1) ** alpha
    d2 = dist(lane2_p1, lane1_p2) ** alpha
    d3 = dist(lane2_p2, lane2_p1) ** alpha
    # Modify tangent 1
    a = d1 * d1
    b = d2 * d2
    c = (2 * d1 * d1) + (3 * d1 * d2) + (d2 * d2)
    d = 3 * d1 * (d1 + d2)
    out_tangent_1_x = (a * lane2_p1[0] - b * lane1_p1[0] + c * lane1_p2[0]) / d
    out_tangent_1_y = (a * lane2_p1[1] - b * lane1_p1[1] + c * lane1_p2[1]) / d
    t1 = [out_tangent_1_x, out_tangent_1_y]
    # Modify tangent 2
    a = d3 * d3
    b = d2 * d2
    c = (2 * d3 * d3) + (3 * d3 * d2) + (d2 * d2)
    d = 3 * d3 * (d3 + d2)
    out_tangent_2_x = (a * lane1_p2[0] - b * lane2_p2[0] + c * lane2_p1[0]) / d
    out_tangent_2_y = (a * lane1_p2[1] - b * lane2_p2[1] + c * lane2_p1[1]) / d
    t2 = [out_tangent_2_x, out_tangent_2_y]

    return calc_bezier_curve([lane1_p2, t1, t2, lane2_p1], ds=ds)
