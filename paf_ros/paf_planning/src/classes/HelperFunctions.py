from typing import List, Tuple

import numpy as np

# todo change to np.hypot()
from paf_messages.msg import Point2D


def dist(a, b):
    x1, y1 = a
    x2, y2 = b
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def closest_index_of_point_list(pts_list: List[Point2D], target_pt: Tuple[float, float], acc: int = 1):
    if len(pts_list) == 0:
        return -1, -1
    distances = [dist([p.x, p.y], target_pt) for p in pts_list[::acc]]
    if len(distances) == 0:
        return len(pts_list) - 1
    idx = int(np.argmin(distances))
    return idx * acc, distances[idx]


def quadratic_deceleration_function(max_speed, factor=1, step_size=0.125):
    fun = []
    for x in range(1000000):
        x *= 0.01
        y = factor * x ** 2
        fun.append(y)
        if y >= max_speed:
            break
    fun = np.array(list(reversed(fun)))
    return fun


def get_linear_deceleration_distance(v_0, v_target, a):
    # s = 1/2 * d_v * t
    # a = d_v / d_t
    # => s = d_v^2 / 2a
    return (v_0 ** 2 - v_target ** 2) / (2 * a)


def get_linear_deceleration_delta_v(braking_distance, a):
    # s = 1/2 * d_v * t
    # a = d_v / d_t
    # d_v = sqrt( 2 * a * s ) / step_size
    return np.sqrt(2 * a * braking_distance)


def linear_deceleration_function(max_speed, deceleration, target_speed=0, step_size=0.125):
    b = max_speed
    delta_v = max_speed - target_speed
    braking_distance = get_linear_deceleration_distance(delta_v, target_speed, deceleration)

    steps = np.ceil(braking_distance / step_size)
    m = -delta_v / steps
    return [m * x + b for x in range(int(steps))]


def speed_to_acceleration(speed_list, step_size=None, dt=None):
    if dt is None:
        time_steps = [step_size / v if v > 0 else 1e-6 for v in speed_list]
        return np.array([(v2 - v1) / dt for v1, v2, dt in zip(speed_list, speed_list[1:], time_steps)])
    if step_size is None:
        return np.array([(v2 - v1) / dt for v1, v2 in zip(speed_list, speed_list[1:])])
    raise RuntimeError("step size or dt must be not None")
