"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/CubicSpline/cubic_spline_planner.py

"""
import math
import bisect

import numpy as np
from bezier import Curve

from .HelperFunctions import xy_to_pts, dist, closest_index_of_point_list


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


def calc_spline_course_from_point_list_grouped(xy_list, ds=0.1, group_no=10):
    assert group_no > 0
    out = []
    temp = []
    for point in xy_list:
        temp.append(point)
        if len(temp) == group_no:  # tangent to previous points
            out += list(calc_spline_course_from_point_list(temp, ds))
            temp = []
    return out


def calc_spline_course_from_point_list(xy_list, ds=0.1):
    if len(xy_list) > 0 and hasattr(xy_list[0], "x"):
        xy_list = [(p.x, p.y) for p in xy_list]
    x, y = list(zip(*xy_list))
    x_target, y_target = calc_spline_course_xy_only(x, y, ds)
    return np.array([np.array([x, y]) for x, y in zip(x_target, y_target)])


def calc_spline_course_xy_only(x, y, ds=0.1):
    assert len(x) == len(y)
    assert len(x) > 1
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry = [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)

    return rx, ry


def calc_spline_course(x, y, ds=0.1):
    assert len(x) == len(y)
    assert len(x) > 1
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


def calc_spline_from_indices(from_index, to_index, full_list, ds):
    indices = list(range(from_index, to_index + 1))
    spline = [full_list[_i] for _i in indices]
    if len(spline) > 3:
        spline = calc_spline_course_from_point_list(spline, ds)
    return spline


def _calc_bezier_curve(x, y, degree=2):
    nodes = np.asfortranarray([x, y])
    return Curve(nodes, degree=degree)


def calc_bezier_curve(pts, ds=0.1):
    degree = len(pts) - 1

    def _calc(pts2):
        _x, _y = [], []
        for pt in pts2:
            _x.append(pt[0])
            _y.append(pt[1])
        curve = _calc_bezier_curve(_x, _y, degree=degree)
        s_vals = np.linspace(0.0, 1.0, num_pts)
        return curve.evaluate_multi(s_vals)

    out_pts = []
    distances = 0
    for p1, p2 in zip(pts, pts[1:]):
        distances += dist(p1, p2)
    num_pts = int(np.ceil(distances / ds))
    x_arr, y_arr = _calc(pts)
    for x, y in zip(x_arr, y_arr):
        out_pts.append(np.array([x, y]))

    return out_pts


def calc_bezier_from_indices(from_index, to_index, full_list, ds):
    indices = list(range(from_index, to_index + 1))
    bezier = [full_list[_i] for _i in indices]
    if len(bezier) > 3:
        bezier = calc_bezier_curve(bezier, ds)
    return bezier


def calc_bezier_curve_from_pts(pts, ds=0.5, max_offset_to_orig=2):
    pts_xy = [[p.x, p.y] for p in pts]
    _new_bezier_pts = calc_bezier_curve(pts_xy, ds)
    try:
        _new_spline_pts = calc_spline_course_from_point_list(pts_xy, 1)
    except AssertionError:
        return pts

    corrected_pts = []

    bezier_indices_prev = []
    bezier_indices_wrong = []
    for k, pt in enumerate(_new_bezier_pts):
        i, _dist = closest_index_of_point_list(_new_spline_pts, pt)
        if _dist > max_offset_to_orig:

            if len(bezier_indices_wrong) == 0 and len(bezier_indices_prev) > 0:
                # need to calc prev bezier again until i
                bezier = calc_bezier_from_indices(
                    np.min(bezier_indices_prev), np.max(bezier_indices_prev), _new_spline_pts, ds
                )
                corrected_pts += xy_to_pts(bezier)

            bezier_indices_wrong += [i]
        elif len(bezier_indices_wrong) > 0:
            # need to calculate new spline (bezier_wrong)
            spline = calc_spline_from_indices(
                np.min(bezier_indices_wrong), np.max(bezier_indices_wrong), _new_spline_pts, ds
            )
            corrected_pts += xy_to_pts(spline)
            bezier_indices_wrong = []
            bezier_indices_prev = []
        else:
            bezier_indices_prev += [i]

    bezier = calc_bezier_from_indices(np.min(bezier_indices_prev), np.max(bezier_indices_prev), _new_spline_pts, ds)
    corrected_pts += xy_to_pts(bezier)

    return corrected_pts
