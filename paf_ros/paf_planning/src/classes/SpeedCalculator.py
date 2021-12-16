import copy
from typing import List
import numpy as np
from commonroad.scenario.traffic_sign import TrafficSignIDGermany

from paf_messages.msg import PafTrafficSignal, Point2D


class SpeedCalculator:
    MAX_SPEED = 250 / 3.6
    MIN_SPEED = 35 / 3.6
    CURVE_FACTOR = 2  # higher value = more drifting
    MAX_DECELERATION = 40  # m/s^2, higher value = later and harder braking
    # percentage of max_deceleration (last x% meters, max/2 is used)
    FULL_VS_HALF_DECEL_FRACTION = 0.94
    QUICK_BRAKE_EVENTS = [TrafficSignIDGermany.STOP.value]
    ROLLING_EVENTS = ["LIGHT", TrafficSignIDGermany.YIELD.value]

    def __init__(self, distances: List[float], index_start: int = 0, index_end=None):
        # step size is assumed constant but has a variance of +/- 1mm
        self._step_size = distances[-1] / len(distances) if len(distances) > 0 else 1
        self._index_start = index_start
        self._index_end = len(distances) - 1 if index_end is None else index_end
        self._distances = distances[index_start:index_end]

        self._continue_on_indices = []
        self._plots = None

        # self._deceleration_delta = np.sqrt(2 * self.MAX_DECELERATION * self._step_size) * self._step_size

        pass

    def _get_deceleration_distance(self, v_0, v_target):
        # s = 1/2 * d_v * t
        # a = d_v / d_t
        # => s = d_v^2 / 2a
        return (v_0 ** 2 - v_target ** 2) / (2 * self.MAX_DECELERATION)

    #
    # def _get_deceleration_delta_v(self, braking_distance):
    #     # s = 1/2 * d_v * t
    #     # a = d_v / d_t
    #     # d_v = sqrt( 2 * a * s ) / step_size
    #     return np.sqrt(2 * self.MAX_DECELERATION * braking_distance)

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
        return np.clip(speed, SpeedCalculator.MIN_SPEED, SpeedCalculator.MAX_SPEED)

    @staticmethod
    def _get_curve_radius_list(path: List[Point2D]) -> List[float]:
        # https://www.mathopenref.com/arcradius.html
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
        max_radius = -1
        if len(path) < 3:
            return [max_radius for _ in path]

        radius_list = [max_radius]
        for p1, p0, p2 in zip(path, path[1:], path[2:]):
            x1, y1 = p1.x, p1.y
            x0, y0 = p0.x, p0.y
            x2, y2 = p2.x, p2.y

            arc_w = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            arc_h = np.abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / arc_w

            if arc_h < 1e-8:
                radius_list.append(max_radius)
            else:
                radius_list.append(arc_h / 2 + arc_w ** 2 / (8 * arc_h))

        radius_list += [max_radius]
        return radius_list

    @staticmethod
    def get_curve_speed(path: List[Point2D]):
        curve_radius_list = SpeedCalculator._get_curve_radius_list(path)
        speed1 = []
        for r in curve_radius_list[::10]:
            speed1 += [SpeedCalculator._radius_to_speed(r)] * 10
        speed1 += [speed1[-1] for _ in range(len(path) - len(speed1))]
        return speed1

    def _linear_deceleration_function(self, target_speed):
        b = self.MAX_SPEED
        delta_v = self.MAX_SPEED - target_speed

        frac = self.FULL_VS_HALF_DECEL_FRACTION

        braking_distance = self._get_deceleration_distance(self.MAX_SPEED, target_speed)
        steps = np.ceil(braking_distance / self._step_size)
        m = -delta_v / steps
        y1 = [m * x + b for x in range(int(steps * frac))]

        steps = np.ceil(braking_distance * 4 / self._step_size)
        m = -delta_v / steps
        y2 = [m * x + y1[-1] for x in range(int(steps * (1 - frac)))]

        return y1 + y2

    def add_linear_deceleration(self, speed):

        time_steps = [self._step_size / dv if dv > 0 else 1e-6 for dv in speed]
        accel = np.array(list(reversed([(v2 - v1) / dt for v1, v2, dt in zip(speed, speed[1:], time_steps)])))
        length = len(accel)
        for i, a in enumerate(accel):
            if a > -self.MAX_DECELERATION:
                continue
            j = length - i
            lin = self._linear_deceleration_function(speed[j])
            k = j - len(lin)
            n = 0
            for n in reversed(range(k, j)):
                if n == 0 or speed[n] < lin[n - k]:
                    n += 1
                    break
            f = n - k
            speed[n:j] = lin[f:]
        try:
            speed[0] = speed[1]
        except IndexError:
            pass
        return speed

    @staticmethod
    def add_speed_limits(speed, traffic_signals: List[PafTrafficSignal], last_known_target_speed=1000):
        speed_limit = np.ones_like(speed) * last_known_target_speed
        traffic_signals = sorted(traffic_signals, key=lambda x: x.index)
        for signal in traffic_signals:
            i = signal.index
            if i < 0 or i >= len(speed):
                continue
            if signal.type == TrafficSignIDGermany.MAX_SPEED.value:
                speed_limit[i:] = signal.value
        return np.clip(speed, 0, speed_limit)

    def add_stop_events(self, speed, traffic_signals: List[PafTrafficSignal], target_speed=0, events=None, buffer_m=1):
        buffer_idx = int(buffer_m / self._step_size)
        if events is None:
            events = self.QUICK_BRAKE_EVENTS
        for signal in traffic_signals:
            i = signal.index - self._index_start
            if i > self._index_end:
                return speed
            if signal.type in events:
                i0, i1 = i - buffer_idx, i + 1
                speed[i0:i1] = target_speed
        return speed

    def remove_stop_event(self, speed, start_idx=0, buffer_m=5, speed_limit=None):
        buffer_idx = int(buffer_m / self._step_size)
        if speed_limit is None:
            speed_limit = self.MAX_SPEED
        end_idx = start_idx + buffer_idx
        speed[start_idx:end_idx] = speed_limit
        return speed

    def add_roll_events(self, speed, traffic_signals: List[PafTrafficSignal], target_speed=0, buffer_m=1):
        return self.add_stop_events(speed, traffic_signals, target_speed, self.ROLLING_EVENTS, buffer_m)

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

    # string type
    # int64 index
    # float32 value
