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

    MAX_DECELERATION = 9.81 * 1  # m/s^2

    QUICK_BRAKE_EVENTS = [TrafficSignIDGermany.STOP.value]
    ROLLING_EVENTS = ["LIGHT", TrafficSignIDGermany.YIELD.value]

    def __init__(self, distances: List[float], curvatures: List[float], index_start: int = 0, index_end=None):
        self.deceleration_delta = self.MAX_DECELERATION * distances[1]  # in m/s per second
        self.step_size = distances[1]
        self.index_start = index_start
        self.index_end = len(curvatures) - 1 if index_end is None else index_end
        self.curvatures = curvatures[index_start:index_end]
        self.distances = distances[index_start:index_end]

        self.continue_on_indices = []

    def get_curve_speed(self):
        length = len(self.curvatures)
        speed = np.arange(length).astype(float)
        for i, curve in enumerate(self.curvatures):
            factor = 1 - curve * self.CURVE_FACTOR  # from 0 straightness to 1
            factor = np.clip(factor, 0, 1)
            speed[i] = np.clip(factor * self.MAX_SPEED, self.MIN_SPEED, self.MAX_SPEED)
        return speed

    @staticmethod
    def _linear_function(m, b, steps, step0=0):
        return [m * x + b for x in range(step0, steps)]

    def add_linear_deceleration(self, speed):
        accel = np.array(list(reversed([y - x for x, y in zip(speed, speed[1:])])))
        length = len(self.curvatures)

        for i, a in enumerate(accel):
            if a < -self.deceleration_delta:
                j = length - 1 - i
                delta_v = speed[j] - self.MAX_SPEED
                k = j + int(delta_v / self.deceleration_delta)
                lin = self._linear_function(-self.deceleration_delta, self.MAX_SPEED, j - k)
                n = 0
                for n in reversed(range(k, j)):
                    if n == 0 or speed[n] < lin[n - k]:
                        break
                f = n - k
                speed[n:j] = lin[f:]
                accel[n:j] = -self.deceleration_delta
        return speed

    def add_speed_limits(self, speed, traffic_signals: List[PafTrafficSignal]):
        for signal in traffic_signals:
            i = signal.index - self.index_start
            if i > self.index_end:
                return speed
            if signal.type == TrafficSignIDGermany.MAX_SPEED.value:
                speed[i:] = np.clip(0, speed[i:], signal.value)
        return speed

    def add_stop_events(self, speed, traffic_signals: List[PafTrafficSignal], target_speed=0, events=None):
        if events is None:
            events = self.QUICK_BRAKE_EVENTS
        for signal in traffic_signals:
            i = signal.index - self.index_start
            if i > self.index_end:
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
        self.plts = plts if num_plts > 1 else [plts]

    def add_to_plt(self, speed, add_accel=False):
        speed = copy.deepcopy(speed)
        length = len(self.curvatures)
        x_values = [i * self.step_size for i in range(length)]
        x_values = [i for i in range(length)]
        if add_accel:
            accel = np.array(list([np.clip((y - x) / self.step_size, -10, 10) for x, y in zip(speed, speed[1:])]))
            self.plts[1].plot(x_values[:-1], accel)
        self.plts[0].plot(x_values, [s * 3.6 for s in speed])

    # string type
    # int64 index
    # float32 value
