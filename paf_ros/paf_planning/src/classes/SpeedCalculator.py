import copy
from typing import List
import numpy as np
from commonroad.scenario.traffic_sign import TrafficSignIDGermany

from paf_messages.msg import PafTrafficSignal

import matplotlib.pyplot as plt


class SpeedCalculator:
    MAX_SPEED = 180 / 3.6
    MIN_SPEED = 30 / 3.6
    CURVE_FACTOR = 9

    MAX_DECELERATION = 9.81 * 2  # m/s^2

    QUICK_BRAKE_EVENTS = [TrafficSignIDGermany.STOP.value]
    ROLLING_EVENTS = ["LIGHT", TrafficSignIDGermany.YIELD.value]

    def __init__(self, distances: List[float], curvatures: List[float], index_start: int = 0, index_end=None):
        # step size is assumed constant but has a variance of +/- 1mm
        self._step_size = distances[-1] / len(distances) if len(distances) > 0 else 1
        self._index_start = index_start
        self._index_end = len(curvatures) - 1 if index_end is None else index_end
        self._curvatures = curvatures[index_start:index_end]
        self._distances = distances[index_start:index_end]

        self._continue_on_indices = []
        self._plots = None

        # self._deceleration_delta = np.sqrt(2 * self.MAX_DECELERATION * self._step_size) * self._step_size

        pass

    def _get_deceleration_distance(self, delta_v):
        # s = 1/2 * d_v * t
        # a = d_v / d_t
        # => s = d_v^2 / 2a
        return delta_v ** 2 / (2 * self.MAX_DECELERATION)

    def _get_deceleration_delta_v(self, braking_distance):
        # s = 1/2 * d_v * t
        # a = d_v / d_t
        # d_v = sqrt( 2 * a * s ) / step_size
        return np.sqrt(2 * self.MAX_DECELERATION * braking_distance)

    def get_curve_speed(self):
        length = len(self._curvatures)
        speed = np.arange(length).astype(float)
        for i, curve in enumerate(self._curvatures):
            factor = 1 - curve * self.CURVE_FACTOR  # from 0 straightness to 1
            factor = np.clip(factor, 0, 1)
            speed[i] = np.clip(factor * self.MAX_SPEED, self.MIN_SPEED, self.MAX_SPEED)
        return speed

    def _linear_deceleration_function(self, target_speed):
        b = self.MAX_SPEED
        delta_v = self.MAX_SPEED - target_speed
        braking_distance = self._get_deceleration_distance(delta_v)

        steps = np.ceil(braking_distance / self._step_size)
        m = -delta_v / steps
        return [m * x + b for x in range(int(steps))]  # in m/s

    def add_linear_deceleration(self, speed):

        deceleration_fun = self._linear_deceleration_function
        time_steps = [self._step_size / dv for dv in speed]
        accel = np.array(list(reversed([(v2 - v1) / dt for v1, v2, dt in zip(speed, speed[1:], time_steps)])))
        length = len(accel)
        for i, a in enumerate(accel):
            if a > -self.MAX_DECELERATION:
                continue
            j = length - i
            lin = deceleration_fun(speed[j])
            k = j - len(lin)
            n = 0
            for n in reversed(range(k, j)):
                if n == 0 or speed[n] < lin[n - k]:
                    n += 1
                    break
            f = n - k
            speed[n:j] = lin[f:]
        return speed

    def add_speed_limits(self, speed, traffic_signals: List[PafTrafficSignal]):
        for signal in traffic_signals:
            i = signal.index - self._index_start
            if i > self._index_end:
                return speed
            if signal.type == TrafficSignIDGermany.MAX_SPEED.value:
                speed[i:] = np.clip(0, speed[i:], signal.value)
        return speed

    def add_stop_events(self, speed, traffic_signals: List[PafTrafficSignal], target_speed=0, events=None):
        if events is None:
            events = self.QUICK_BRAKE_EVENTS
        for signal in traffic_signals:
            i = signal.index - self._index_start
            if i > self._index_end:
                return speed
            if signal.type in events:
                i0, i1 = i - 1, i + 1
                speed[i0:i1] = target_speed
        return speed

    def add_roll_events(self, speed, traffic_signals: List[PafTrafficSignal], target_speed=0):
        return self.add_stop_events(speed, traffic_signals, target_speed, self.ROLLING_EVENTS)

    def plt_init(self):
        plt.close()
        num_plts = 2
        fig, plts = plt.subplots(num_plts)
        plt.xlabel("Distanz in m")
        self._plots = plts if num_plts > 1 else [plts]

    @staticmethod
    def plt_show():
        plt.show()

    def plt_add(self, speed, add_accel=False):
        speed = copy.deepcopy(speed)
        length = len(self._curvatures)
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
