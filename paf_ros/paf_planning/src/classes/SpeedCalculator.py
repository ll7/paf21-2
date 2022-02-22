import copy
from typing import List
import numpy as np
from commonroad.scenario.traffic_sign import TrafficSignIDGermany

from paf_messages.msg import Point2D
from .HelperFunctions import dist
from .MapManager import MapManager


class SpeedCalculator:
    CITY_SPEED_LIMIT = 50 / 3.6
    SPEED_LIMIT_MULTIPLIER = 1
    CURVE_RAD_MEASURE_STEP = 5
    FULL_VS_HALF_DECEL_FRACTION = 0.97
    MUST_STOP_EVENTS = [TrafficSignIDGermany.STOP.value]
    CAN_STOP_EVENT = ["LIGHT", TrafficSignIDGermany.YIELD.value]
    SPEED_LIMIT_RESTORE_EVENTS = ["MERGE"] + MUST_STOP_EVENTS + CAN_STOP_EVENT

    UNKNOWN_SPEED_LIMIT_SPEED, MAX_SPEED, MIN_SPEED, CURVE_FACTOR, MAX_DECELERATION = 100, 100, 10, 1, 10

    def __init__(self, step_size: float, index_start: int = 0):
        # step size is assumed constant but has a variance of +/- 1mm
        self._step_size = step_size
        self._index_start = index_start
        self._plots = None
        self.set_limits()

    @staticmethod
    def set_limits(rules_enabled: bool = None):
        if rules_enabled is None:
            rules_enabled = MapManager.get_rules_enabled()
        SpeedCalculator.UNKNOWN_SPEED_LIMIT_SPEED = SpeedCalculator.CITY_SPEED_LIMIT if rules_enabled else 250 / 3.6
        SpeedCalculator.MAX_SPEED = 95 / 3.6 if rules_enabled else 95 / 3.6
        SpeedCalculator.MIN_SPEED = 25 / 3.6 if rules_enabled else 25 / 3.6
        SpeedCalculator.CURVE_FACTOR = 2 if rules_enabled else 2  # higher value = more drifting
        SpeedCalculator.MAX_DECELERATION = 20 if rules_enabled else 20
        # m/s^2, higher value = later and harder braking

    def _get_deceleration_distance(self, v_0, v_target):
        # s = 1/2 * d_v * t
        # a = d_v / d_t
        # => s = d_v^2 / 2a
        return (v_0 ** 2 - v_target ** 2) / (2 * self.MAX_DECELERATION)

    @staticmethod
    def _radius_to_speed_fast(curve_radius):
        # https://www.state.nj.us/transportation/eng/tools/CalculatorESafeSpeedGreaterThan50.shtm
        if curve_radius < 0:
            return SpeedCalculator.MAX_SPEED
        e = 0  # super_elevation in percent
        r = curve_radius * 3.28084  # m to ft
        speed = ((-0.03 * r) + (np.sqrt(((0.03 * r) * (0.03 * r)) + ((4 * r) * ((15 * (e / 100)) + 3.6))))) / 2
        if speed < 50:
            return -1
        return speed * 0.44704 * SpeedCalculator.CURVE_FACTOR

    @staticmethod
    def _radius_to_speed_slow(curve_radius):
        # https://www.state.nj.us/transportation/eng/tools/CalculatorESafeSpeedLessThanEqualTo50.shtm
        if curve_radius < 0:
            return SpeedCalculator.MAX_SPEED
        e = 0  # super_elevation in percent
        r = curve_radius * 3.28084  # m to ft
        speed = ((-0.015 * r) + (np.sqrt(((0.015 * r) * (0.015 * r)) + ((4 * r) * ((15 * (e / 100)) + 2.85))))) / 2
        # speed = 0.5 * (-.015 * curve_radius + (
        #         (.015 * curve_radius) ** 2 + 4 * curve_radius * np.sqrt(15 * super_elevation + 3.6)))
        if speed > 50:
            return -1
        return speed * 0.44704 * SpeedCalculator.CURVE_FACTOR  # mi/h to m/s

    @staticmethod
    def _radius_to_speed(curve_radius):
        if curve_radius < 0:
            return SpeedCalculator.MAX_SPEED

        speed = SpeedCalculator._radius_to_speed_slow(curve_radius)
        if speed < 0:
            speed = SpeedCalculator._radius_to_speed_fast(curve_radius)
        if speed < 0:
            speed = -100
        speed = np.clip(speed, SpeedCalculator.MIN_SPEED, SpeedCalculator.MAX_SPEED)
        if speed < 0:
            raise RuntimeError("IMPORSSIBLEuiaretni")
        return speed

    @staticmethod
    def get_curve_radius_list(path: List[Point2D], max_radius=99999999, equal_dist=False):
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
    def _get_curve_radius_list_equal_distances(path_in: List[Point2D]):
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

    def get_curve_speed(self, path: List[Point2D]):
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

    def _linear_deceleration_function(self, target_speed):
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

    def add_linear_deceleration(self, speed):

        time_steps = [self._step_size / dv if dv > 0 else 1e-6 for dv in speed]
        accel = np.array(list(reversed([(v2 - v1) / dt for v1, v2, dt in zip(speed, speed[1:], time_steps)])))
        length = len(accel)
        for i, (v, a) in enumerate(zip(speed, accel)):
            if a > -self.MAX_DECELERATION:
                continue
            j = length - i
            lin, breaking_distance = self._linear_deceleration_function(speed[j])
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

    def get_event_deceleration(self, target_speed: float = 0, buffer_m: float = 0, step=None):
        if step is None:
            step = self._step_size
        buffer_idx = int(buffer_m / step) + 1  # add afterwards
        speed_limit, breaking_distance = self._linear_deceleration_function(target_speed)
        speed_limit = list(speed_limit) + [target_speed for _ in range(buffer_idx)]
        return np.clip(speed_limit, target_speed, self.MAX_SPEED), breaking_distance

    def plt_init(self, add_accel=False):
        import matplotlib.pyplot as plt

        num_plts = 2 if add_accel else 1
        plt.close()
        fig, plts = plt.subplots(num_plts)
        plt.xlabel("Distanz in m")
        self._plots = plts if num_plts > 1 else [plts]

    @staticmethod
    def plt_show():
        import matplotlib.pyplot as plt

        plt.show()

    def plt_add(self, speed):
        speed = copy.deepcopy(speed)
        length = len(speed)
        add_accel = len(self._plots) == 2
        # x_values = [self._step_size / dv for dv in speed]
        x_values = [i * self._step_size for i in range(length)]
        # x_values = [i for i in range(length)]
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
