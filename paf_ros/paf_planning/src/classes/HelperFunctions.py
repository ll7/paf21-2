from typing import List, Tuple, Union, Any

import numpy as np
from commonroad.geometry.shape import Circle
from commonroad.scenario.lanelet import LaneletNetwork

from paf_messages.msg import Point2D

from .MapManager import MapManager

from geometry_msgs.msg import Point


def dist(
    a: Union[Point2D, np.ndarray, Tuple[float, float]], b: Union[Point2D, np.ndarray, Tuple[float, float]] = None
) -> float:  # todo change to np.hypot()
    """
    Distance of two Point2D points
    :param a: p1
    :param b: p2
    :return: distance
    """

    try:
        x1, y1 = a
        x2, y2 = b
    except TypeError:
        if hasattr(a, "x"):
            a = (a.x, a.y)
        if hasattr(b, "x"):
            b = (b.x, b.y)
        elif b is None:
            b = (0, 0)
        x1, y1 = a
        x2, y2 = b
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def xy_to_pts(xy_list: Union[List[Tuple[float, float], List[np.ndarray]], np.ndarray]) -> List[Point2D]:
    """
    Convert list of point tuples to list of point objects
    :param xy_list: list of tuples
    :return: list of points
    """
    out = []
    for pt in xy_list:
        try:
            x, y = pt
            pt = Point2D(x, y)
        except (TypeError, ValueError):
            pass
        if type(pt) is not Point2D:
            raise TypeError(f"unable to parse type {type(pt)} to Point2D ({pt}, {xy_list})")
        out.append(pt)
    return out


def pts_to_xy(pts_list: List[Point2D]) -> List[Tuple[float, float]]:
    """
    Convert list of point objects to list of point tuples
    :param pts_list: list of points
    :return: list of tuples
    """
    return [(p.x, p.y) for p in pts_list]


def closest_index_of_point_list(
    pts_list: Union[List[Point2D], List[Tuple[float, float]]],
    target_pt: Union[Point2D, Tuple[float, float]],
    acc: int = 1,
) -> Tuple[int, float]:
    """
    Calculates the closest index of a point to all points in a list
    :param pts_list: list of points
    :param target_pt: target point
    :param acc: search only every nth point (nth=acc)
    :return: index, distance to point
    """
    if len(pts_list) == 0:
        return -1, -1
    distances = [dist(p, target_pt) for p in pts_list[::acc]]
    if len(distances) == 0:
        return len(pts_list) - 1, 1e-10
    idx = int(np.argmin(distances))
    return idx * acc, distances[idx]


def k_closest_indices_of_point_in_list(
    k: int,
    _pts_list: Union[List[Point2D], List[Tuple[float, float]]],
    target_pt: Union[Point2D, Tuple[float, float]],
    acc: int = 1,
) -> Tuple[List[int], List[float]]:
    """
    Calculates the closest index of a point to all points in a list
    :param k: return k closest points
    :param pts_list: list of points
    :param target_pt: target point
    :param acc: search only every nth point (nth=acc)
    :return: list of indices, list of distance to points
    """
    pts_list = _pts_list[::acc]

    if len(pts_list) <= k:
        k = len(pts_list) - 1

    if len(pts_list) <= 0:
        return [], []

    distances = [dist(p, target_pt) for p in pts_list]
    if len(distances) == 0:
        return [len(pts_list) - 1], [1e-10]
    idx = np.argpartition(distances, k)
    idx = idx[:k]
    return idx * acc, [distances[i] for i in idx]


def expand_sparse_list(
    sparse_list: List[Any],
    points_current: List[Point2D],
    points_target: List[Point2D],
    fill_value: Any = None,
    indices_new: List[int] = None,
    accuracy: int = 4,
) -> Tuple[List[Any], List[int]]:
    """
    Expands a sparse list by either filling up the gaps with fill values
    or the last known values if fill value is not specified
    :param sparse_list: list to expand
    :param points_current: sparse list of points with same length as sparse list
    :param points_target: dense list of points with final length
    :param fill_value: optional filler value (previous value if not specified)
    :param indices_new: indices of sparse list in returned list
    :param accuracy: accuracy (speed) of index search (take every nth entry, 4=1m with ds=0.25 in LocalPath class)
    :return: dense list, indices_new
    """
    if len(sparse_list) < len(points_current):
        raise ValueError(f"length of sparse list {len(sparse_list)} < " f"current points {len(points_current)}")
    if indices_new is None:
        indices_new = []
        factor = (int(len(points_target) / len(points_current)) + 2) * 4
        last_target_idx = 0
        for i, pt in enumerate(points_current):
            if fill_value is not None and sparse_list[i] == fill_value:
                indices_new += [0] if len(indices_new) == 0 else [indices_new[-1] + 1]
                continue
            ind = (i + 1) * factor
            idx_found, _ = closest_index_of_point_list(points_target[last_target_idx:ind], pt, accuracy)
            last_target_idx = idx_found + last_target_idx
            indices_new += [last_target_idx]

    out = [fill_value for _ in points_target]  # None or custom placeholder

    for v, idx in zip(sparse_list, indices_new):
        idx = idx + 5
        if idx >= len(out):
            break
        elif 0 <= idx:
            out[idx] = v

    filler = sparse_list[0]
    if fill_value is None:
        for i, v in enumerate(out):
            if v is None:
                out[i] = filler
            else:
                filler = v
    return out, indices_new


def sparse_list_from_dense_pts(pts: np.ndarray, num_pts: int, distances: List[float] = None) -> List[np.ndarray]:
    """
    Shrink down a possibly irregularly spaced list of points to a sparse list of a specific length
    :param pts: points dense
    :param num_pts: target number of points in resulting list
    :param distances: optional distances between points (skips computing this list beforehand)
    :return: points sparse
    """
    path = []
    dist_measure = 0
    if distances is None:
        distances = [0.0] + [dist(prev, p) for prev, p in zip(pts, pts[1:])]
    elif len(distances) == len(pts) - 1:
        distances = [0.0] + distances
    tot_dist = int(np.sum(distances))

    min_dist = tot_dist / num_pts
    for d, p in zip(distances, pts):
        dist_measure += d
        if dist_measure >= min_dist:
            dist_measure = 0
            path += [p]

    if not np.all(path[-1] == pts[-1]):
        path += [pts[-1] + np.random.rand() * 1e-4]
    return path


def find_closest_lanelet(lanelet_network: LaneletNetwork, p: Union[Point2D, Tuple[float, float]]) -> List[int]:
    """
    find closest lanelets in network to a point (many possible on intersections)
    :param lanelet_network: current commonroad network
    :param p: point to search for
    :return: list of lanelets
    """
    if hasattr(p, "x"):
        p = (p.x, p.y)
    p = np.array(p, dtype=float)
    lanelets = lanelet_network.find_lanelet_by_position([p])[0]
    if len(lanelets) > 0:
        return lanelets
    for radius in range(3, 1000, 5):
        shape = Circle(radius=radius / 10, center=p)
        lanelets = lanelet_network.find_lanelet_by_shape(shape)
        if len(lanelets) > 0:
            break
    return lanelets


def on_bridge(pos: Point, lanelet_network: LaneletNetwork):
    if -2 < pos.z < 2:
        return False
    closest_lanelets = find_closest_lanelet(lanelet_network, pos)
    for let in closest_lanelets:
        if MapManager.lanelet_on_bridge(let):
            return True
    return False


def find_lanelet_yaw(lanelet_network: LaneletNetwork, target_pt: Point2D, lanelet_id=None) -> float:
    """
    Calculates orientation of the lanelet closest to the target point specified
    :param lanelet_network: current commonroad network
    :param target_pt: point to search for
    :param lanelet_id: give lanelet id to skip searching for them
    :return: angle in rad
    """

    if lanelet_id is None:
        lanelet_ids = find_closest_lanelet(lanelet_network, target_pt)
        if len(lanelet_ids) == 0:
            return 0
        lanelet_id = lanelet_ids[0]

    lanelet_id = lanelet_network.find_lanelet_by_id(lanelet_id)
    i, _ = closest_index_of_point_list([Point2D(p[0], p[1]) for p in lanelet_id.center_vertices], target_pt)

    if i == -1:
        return 0

    if i < len(lanelet_id.center_vertices) - 1:
        p_i = lanelet_id.center_vertices[i]
        p_j = lanelet_id.center_vertices[i + 1]
    else:
        p_i = lanelet_id.center_vertices[i - 1]
        p_j = lanelet_id.center_vertices[i]

    dx, dy = p_j[0] - p_i[0], p_j[1] - p_i[1]
    angle = np.arcsin(dy / dist((dx, dy), (0, 0)))
    if angle < 0:
        angle += np.pi * 2
    return angle


def get_angle_between_vectors(
    v1: Union[Point2D, Tuple[float, float], np.ndarray],
    v2: Union[Point2D, Tuple[float, float], np.ndarray] = None,
    is_normed: bool = False,
):
    """
    Returns the angle between two vectors
    :param v1: vector 1
    :param v2: vector 2 (optional, [0,0] if not specified)
    :param is_normed: vector is normalized (optional), skips normalizing step in calculation
    :return: angle in rad
    """
    if hasattr(v1, "x"):
        v1 = v1.x, v1.y
    if hasattr(v2, "y"):
        v2 = v2.x, v2.y

    v1 = np.array(v1)
    v2 = None if v2 is None else np.array(v2)

    if not is_normed:
        v1 = v1 / dist(v1, (0, 0))
        if v2 is not None:
            v2 = v2 / dist(v2, (0, 0))
    if v2 is None:
        v2 = [0, 1]
    dot = np.dot(v2, v1)
    det = v1[0] * v2[1] - v2[0] * v1[1]
    yaw = np.arctan2(dot, det)
    return yaw
