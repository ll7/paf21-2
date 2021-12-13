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
        self._deceleration_delta = self.MAX_DECELERATION * distances[1]  # in m/s per second
        self._step_size = distances[1]
        self._index_start = index_start
        self._index_end = len(curvatures) - 1 if index_end is None else index_end
        self._curvatures = curvatures[index_start:index_end]
        self._distances = distances[index_start:index_end]

        self._continue_on_indices = []
        self._plots = None

    def get_curve_speed(self):
        length = len(self._curvatures)
        speed = np.arange(length).astype(float)
        for i, curve in enumerate(self._curvatures):
            factor = 1 - curve * self.CURVE_FACTOR  # from 0 straightness to 1
            factor = np.clip(factor, 0, 1)
            speed[i] = np.clip(factor * self.MAX_SPEED, self.MIN_SPEED, self.MAX_SPEED)
        return speed

    def _linear_deceleration_function(self, target_speed):
        m = -self._deceleration_delta
        b = self.MAX_SPEED
        delta_v = self.MAX_SPEED - target_speed
        steps = int(delta_v / self._deceleration_delta)
        return [m * x + b for x in range(steps)]

    def add_linear_deceleration(self, speed):

        deceleration_fun = self._linear_deceleration_function
        accel = np.array(list(reversed([y - x for x, y in zip(speed, speed[1:])])))
        length = len(self._curvatures)

        for i, a in enumerate(accel):
            if a < -self._deceleration_delta:
                j = length - 1 - i
                lin = deceleration_fun(speed[j])
                k = j - len(lin)
                n = 0
                for n in reversed(range(k, j)):
                    if n == 0 or speed[n] < lin[n - k]:
                        break
                f = n - k
                speed[n:j] = lin[f:]
                accel[n:j] = -self._deceleration_delta
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

    def open_plt(self):
        plt.close()
        num_plts = 2
        fig, plts = plt.subplots(num_plts)
        plt.xlabel("Distanz in m")
        self._plots = plts if num_plts > 1 else [plts]

    def add_to_plt(self, speed, add_accel=False):
        speed = copy.deepcopy(speed)
        length = len(self._curvatures)
        x_values = [i * self._step_size for i in range(length)]
        x_values = [i for i in range(length)]
        if add_accel:
            accel = np.array(list([np.clip((y - x) / self._step_size, -10, 10) for x, y in zip(speed, speed[1:])]))
            self._plots[1].plot(x_values[:-1], accel)
        self._plots[0].plot(x_values, [s * 3.6 for s in speed])

    # string type
    # int64 index
    # float32 value
