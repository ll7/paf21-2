import copy
from typing import List, Tuple
import numpy as np
from commonroad.scenario.traffic_sign import TrafficSignIDGermany
from numpy import ndarray

from paf_messages.msg import Point2D
from .HelperFunctions import dist
from .MapManager import MapManager


class SpeedCalculator:
    """Class for handling speed in the local plan (deceleration, curve speed, etc.)"""

    CITY_SPEED_LIMIT = 50 / 3.6
    MERGE_SPEED_RESET = 50 / 3.6
    APPLY_MERGING_RESET = True
    SPEED_LIMIT_MULTIPLIER = 1
    CURVE_RAD_MEASURE_STEP = 5
    FULL_VS_HALF_DECEL_FRACTION = 0.97
    MUST_STOP_EVENTS = [TrafficSignIDGermany.STOP.value]
    CAN_STOP_EVENT = ["LIGHT", TrafficSignIDGermany.YIELD.value]

    # these values are set in set_limits function on init or gamemode change
    UNKNOWN_SPEED_LIMIT_SPEED, MAX_SPEED, MIN_SPEED, CURVE_FACTOR, MAX_DECELERATION = 100, 100, 10, 1, 10

    def __init__(self, step_size: float):
        """
        Constructor sets step size and sets speed limits based on rule-mode
        :param step_size: step size (in meter) in which the speed changes on the path.
        """
        # step size is assumed constant but has a variance of +/- 1mm
        self._step_size = step_size
        self._plots = None
        self.set_limits()

    @staticmethod
    def set_limits(rules_enabled: bool = None):
        """
        Sets speed limits with given rule set. Must be called, when rule parameter is changed at runtime from both
        global and local planner.
        :param rules_enabled: rule mode
        """
        if rules_enabled is None:
            rules_enabled = MapManager.get_rules_enabled()
        SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED = SpeedCalculator.CITY_SPEED_LIMIT if rules_enabled else 250 / 3.6
        SpeedCalculator.MAX_SPEED = 90 / 3.6 if rules_enabled else 90 / 3.6
        SpeedCalculator.MIN_SPEED = 25 / 3.6 if rules_enabled else 25 / 3.6
        SpeedCalculator.CURVE_FACTOR = 2 if rules_enabled else 2  # higher value = more drifting
        SpeedCalculator.MAX_DECELERATION = 20 if rules_enabled else 20
        # m/s^2, higher value = later and harder braking

    def _get_deceleration_distance(self, v_0: float, v_target: float) -> float:
        """
        Deceleration distance in meters based on delta speed
        :param v_0: start speed
        :param v_target: target speed
        :return: braking distance (ideal)
        """
        # s = 1/2 * d_v * t
        # a = d_v / d_t
        # => s = d_v^2 / 2a
        return (v_0 ** 2 - v_target ** 2) / (2 * self.MAX_DECELERATION)

    @staticmethod
    def _radius_to_speed_fast(curve_radius: float) -> float:
        """
        Convert a curve radius to a speed value (used for speeds > 80 kmh / 50 mph)
        :param curve_radius: radius in m
        :return: speed in m/s
        """
        # https://www.state.nj.us/transportation/eng/tools/CalculatorESafeSpeedGreaterThan50.shtm
        # wrong formula on page, correct formula in HTML only
        if curve_radius < 0:
            return SpeedCalculator.MAX_SPEED
        e = 0  # super_elevation in percent
        r = curve_radius * 3.28084  # m to ft
        speed = ((-0.03 * r) + (np.sqrt(((0.03 * r) * (0.03 * r)) + ((4 * r) * ((15 * (e / 100)) + 3.6))))) / 2
        if speed < 50:
            return -1
        return speed * 0.44704 * SpeedCalculator.CURVE_FACTOR

    @staticmethod
    def _radius_to_speed_slow(curve_radius: float) -> float:
        """
        Convert a curve radius to a speed value (used for speeds < 80 kmh / 50 mph)
        :param curve_radius: radius in m
        :return: speed in m/s
        """
        # https://www.state.nj.us/transportation/eng/tools/CalculatorESafeSpeedLessThanEqualTo50.shtm
        # wrong formula on page, correct formula in HTML only
        if curve_radius < 0:
            return SpeedCalculator.MAX_SPEED
        e = 0  # super_elevation in percent
        r = curve_radius * 3.28084  # m to ft
        speed = ((-0.015 * r) + (np.sqrt(((0.015 * r) * (0.015 * r)) + ((4 * r) * ((15 * (e / 100)) + 2.85))))) / 2
        if speed > 50:
            return -1
        return speed * 0.44704 * SpeedCalculator.CURVE_FACTOR  # mi/h to m/s

    @staticmethod
    def _radius_to_speed(curve_radius: float) -> float:
        """
        Convert a radius to a speed value. Selects between the different methods automatically
        :param curve_radius: radius in m
        :return: speed in m/s
        """
        if curve_radius < 0:
            return SpeedCalculator.MAX_SPEED

        speed = SpeedCalculator._radius_to_speed_slow(curve_radius)
        if speed < 0:
            speed = SpeedCalculator._radius_to_speed_fast(curve_radius)
        if speed < 0:
            speed = -100
        speed = np.clip(speed, SpeedCalculator.MIN_SPEED, SpeedCalculator.MAX_SPEED)
        if speed < 0:
            raise RuntimeError("this statement should not be reached ever. Check the code!")
        return float(speed)

    @staticmethod
    def get_curve_radius_list(
        path: List[Point2D], max_radius: float = 99999999, equal_dist: bool = False
    ) -> List[float]:
        """
        Convert a list of points to a list of curve radii. The last radius is ALWAYS max_radius,
        so that the length of the returning array stays the same
        :param path: list of points (path). Must be a sparse list (distances should be in range of
        CURVE_RAD_MEASURE_STEP meters), or the radius calculation goes nuts.
        :param max_radius: radius value if going straight (big number)
        :param equal_dist: if set to True, equal distance between the path points is assumed
        :return: List of radii
        """
        # https://www.mathopenref.com/arcradius.html
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points

        if equal_dist:
            radius_list, fill, _ = SpeedCalculator._get_curve_radius_list_equal_distances(path)
            out = []
            for r, f in zip(radius_list, fill):
                out += [r for _ in range(f)]
            return out

        if len(path) <= 2:
            return [max_radius for _ in path]

        radius_list = [max_radius]
        for p1, p0, p2 in zip(path, path[1:], path[2:]):
            x1, y1 = p1.x, p1.y
            x0, y0 = p0.x, p0.y
            x2, y2 = p2.x, p2.y

            if np.any(np.isnan([x1, y1, x0, y0, x2, y2])):
                raise RuntimeError()

            arc_w = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if arc_w < 1e-8:
                radius_list.append(max_radius)
                continue

            arc_h = np.abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / arc_w

            if arc_h < 1e-8:
                radius_list.append(max_radius)
            else:
                radius_list.append(arc_h / 2 + arc_w ** 2 / (8 * arc_h))

        radius_list += [max_radius]
        return radius_list

    @staticmethod
    def _get_curve_radius_list_equal_distances(path_in: List[Point2D]) -> Tuple[List[float], List[int], List[float]]:
        """
        Converts a path with not equal distances to one with equal distances between points by enlarging the array if
        the distance is smaller than CURVE_RAD_MEASURE_STEP.
        :param path_in: path with differing distances between points
        :return: path with equal distances, amount of repetitions for each point (to use again later)
        and the distances between all points (to use again later)
        """
        min_dist = SpeedCalculator.CURVE_RAD_MEASURE_STEP
        max_radius = 99999999
        path = []
        fill = [1]
        distances = [0]

        dist_measure = 0
        for prev, p in zip(path_in, path_in[1:]):
            distances.append(dist(prev, p))
            dist_measure += distances[-1]
            if dist_measure < min_dist:
                fill[-1] += 1
            else:
                dist_measure = 0
                path += [p]
                fill += [1]

        if len(path) < 3:
            return [max_radius for _ in path], fill, distances

        radius_list = SpeedCalculator.get_curve_radius_list(path, max_radius)

        return radius_list, fill, distances

    def get_curve_speed(self, path: List[Point2D]) -> List[float]:
        """
        Get curve speed for given path
        :param path: sparse or smoothed path (radius and speed calculation for every point!)
        :return: list of speeds with the same dimension
        """
        curve_radius_list, fill, distances = self._get_curve_radius_list_equal_distances(path)
        speed1 = [SpeedCalculator._radius_to_speed(r) for r in curve_radius_list]
        speed2 = []

        if len(speed1) == 0:
            return [self.UNKNOWN_SPEED_LIMIT_SPEED for _ in path]

        for r, f in zip(speed1, fill):
            speed2 += [r for _ in range(f)]
        for _ in range(len(path) - len(speed2)):
            speed2.append(speed1[-1])
        return speed2

    def _linear_deceleration_function(self, target_speed: float) -> Tuple[ndarray, float]:
        """
        Get a deceleration function as speed over distance. After FULL_VS_HALF_DECEL_FRACTION percent of the path,
        the half deceleration is used for a smoother stopping experience
        :param target_speed: target speed clipped between MIN_SPEED and MAX_SPEED
        :return: deceleration as speed array with f[0]=MAX_SPEED and f[-1]=target_speed and the distance taken
        """
        b = self.MAX_SPEED
        delta_v = self.MAX_SPEED - target_speed

        frac = 1 if target_speed < self.MIN_SPEED else self.FULL_VS_HALF_DECEL_FRACTION

        braking_distance = self._get_deceleration_distance(self.MAX_SPEED, target_speed)
        steps = np.ceil(braking_distance / self._step_size)
        m = -delta_v / steps
        y1 = [m * x + b for x in range(int(steps * frac))]

        steps = np.ceil(braking_distance * 4 / self._step_size)
        m = -delta_v / steps
        y2 = [m * x + y1[-1] for x in range(int(steps * (1 - frac)))]
        y2 = [y for y in y2 if np.abs(y) > 10]

        speed = y1 + y2

        return np.clip(speed, self.MIN_SPEED, self.MAX_SPEED), braking_distance

    def add_linear_deceleration(self, speed: List[float]) -> ndarray:
        """
        Adds deceleration speed to a given speed array
        :param speed: list of speed values (step size set in constructor)
        :return: list of speed values with linear deceleration added
        """
        time_steps = [self._step_size / dv if dv > 0 else 1e-6 for dv in speed]
        accel = np.array(list(reversed([(v2 - v1) / dt for v1, v2, dt in zip(speed, speed[1:], time_steps)])))
        length = len(accel)
        for i, (v, a) in enumerate(zip(speed, accel)):
            if a > -self.MAX_DECELERATION:
                continue
            j = length - i
            lin, _ = self._linear_deceleration_function(speed[j])
            k = j - len(lin)
            n = 0
            for n in reversed(range(k, j)):
                if n == 0 or speed[n] < lin[n - k]:
                    n += 1
                    break
            f = n - k
            try:
                speed[n:j] = lin[f:]
            except ValueError:
                pass
        try:
            speed[0] = speed[1]
        except IndexError:
            pass

        speed = np.clip(speed, 0, 999)
        return speed

    def plt_init(self, add_accel: bool = False):
        """
        Init matplotlib plot for plot velocity (and optional: acceleration data).
        Cannot be used within a ROS-Node (debugging only)
        :param add_accel: plot acceleration data as well
        """
        import matplotlib.pyplot as plt

        num_plts = 2 if add_accel else 1
        plt.close()
        fig, plts = plt.subplots(num_plts)
        plt.xlabel("Distanz in m")
        self._plots = plts if num_plts > 1 else [plts]

    @staticmethod
    def plt_show():
        """
        Show plot created before. Function plt_init and function plt_add must have been called before.
        """
        import matplotlib.pyplot as plt

        plt.show()

    def plt_add(self, speed: np.ndarray):
        """
        Add speed array to matplotlib plot
        :param speed: speed array
        """
        speed = copy.deepcopy(speed)
        length = len(speed)
        add_accel = len(self._plots) == 2
        x_values = [i * self._step_size for i in range(length)]
        if add_accel:
            accel = np.array(
                list(
                    [
                        np.clip((y - x) / self._step_size, -1.5 * self.MAX_DECELERATION, self.MAX_DECELERATION)
                        for x, y in zip(speed, speed[1:])
                    ]
                )
            )
            self._plots[1].plot(x_values[:-1], accel)
        self._plots[0].plot(x_values, [s * 3.6 for s in speed])
