from math import cos, sin, sqrt
from pathlib import Path
from shapely.geometry import Point, LineString
import numpy as np
import json

import pyproj

from .road import RoadSegment, Lane, LaneType
from .constants import CENTER_COORDS, DATA_PATH


def dump_json(obj, path, pretty=True):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=4 if pretty else None, ensure_ascii=False)


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def get_file_path(filename):
    return Path(DATA_PATH).joinpath(filename)


def getPositiveHeading(hdg):
    while hdg < 0.0:
        hdg += 2.0 * np.pi
    return hdg % (np.pi * 2.0)


def get_heading(x1, y1, x2, y2):
    assert not (x1 == x2 and y1 == y2), "Can't give heading without a direction"
    x = [x1, x2]
    y = [y1, y2]
    x_arr = np.array(x) - x[0]
    y_arr = np.array(y) - y[0]

    if x_arr[1] > 0:
        phi = np.arctan(y_arr[1] / x_arr[1])
    elif x_arr[1] == 0:
        if y_arr[1] > 0:
            phi = np.pi / 2
        else:
            phi = -np.pi / 2
    else:
        if y_arr[1] >= 0:
            phi = np.arctan(y_arr[1] / x_arr[1]) + np.pi
        else:
            phi = np.arctan(y_arr[1] / x_arr[1]) - np.pi

    return getPositiveHeading(phi)


def get_distance(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_road_segment(input: list[str], total_width: float = None) -> RoadSegment:
    road = RoadSegment()
    biking_width = 1.25
    driving_width = 3.5

    lanes = []
    for nvdb_lane in input:
        same_direction = int(nvdb_lane[0]) % 2 == 1
        lane_type = LaneType.NORMAL
        for special_type in LaneType.__members__.values():
            if special_type.value in nvdb_lane:
                lane_type = special_type
                break
        lane_width = biking_width if lane_type == LaneType.BICYCLE else driving_width
        lane = Lane(lane_type, same_direction, lane_width)
        lanes.append(lane)

    if total_width: # scale to match total_width if set
        num_bike = sum(1 for lane in lanes if lane.type == LaneType.BICYCLE)
        num_driving = len(lanes) - num_bike
        default_width = num_bike * biking_width + num_driving * driving_width
        for lane in lanes:
            lane.width *= total_width / default_width

    for lane in lanes:
        road.add_lane(lane)

    return road


#transform = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:5973")
transform = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:25833")
center = transform.transform(CENTER_COORDS[0], CENTER_COORDS[1])


def get_relative_coordinates(x, y):
    return (x - center[0], y - center[1])


def calculate_cubic_curve(end: tuple[float], phi: float, length_subdivisions: int = 20):
    """Calculates and returns a parametric cubic polynomial of a curve between
    the points (0, 0) and (end[0], end[1]). The curve in the uv plane is given by:
    c(p) = (u(p), v(p)),
    u(p) = aU + bU*p + cU*p² + dU*p³,
    v(p) = aV + bV*p + cV*p² + dV*p³
    See https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/latest/specification/09_geometries/09_06_param_poly3.html

    Args:
        end (tuple[float]): s and t coordinates of end-point
        phi (float): angle of the curve at end point in radians, relative to the starting point.
            The curve always starts with phi=0 at the start point s=t=p=0.
            Example: phi=math.pi/2: the curve turns left 90 degree
            Range: -pi/2 < phi < pi/2
        length_subdivisions (int): how many parts the curve is split into to calculate the length
    Returns:
        params: tuple(float): parameters aU, aV, bU, bV, cU, cV, dU, dV.
            pRange should be set to "normalized"
        length: approximate length of the curve
    """
    sin_phi, cos_phi = sin(phi), cos(phi)
    aU = aV = bV = 0
    bU = end[0]
    cU = -(cos_phi * (end[0] - end[1]) - end[0])
    dU = (cos_phi * (end[0] - end[1]) - end[0])
    cV = -(sin_phi * (end[0] - end[1]) - 3 * end[1])
    dV = (sin_phi * (end[0] - end[1]) - 2 * end[1])

    def u(p): return aU + bU * p + cU * p ** 2 + dU * p ** 3
    def v(p): return aV + bV * p + cV * p ** 2 + dV * p ** 3
    # u2 - u1 = 0 + bU * (p2 - p1) + cU * (p2^2 - p1^2) + dU * (p2^3 - p1^3)

    ps = [i / length_subdivisions for i in range(length_subdivisions + 1)]
    length = 0
    for p1, p2 in zip(ps[:-1], ps[1:]):
        length += sqrt((u(p2) - u(p1)) ** 2 + (v(p2) - v(p1)) ** 2)

    return (aU, aV, bU, bV, cU, cV, dU, dV), length


def shorten_coordinate_list(points: list[tuple[float]], cut_length: float, from_start: bool, inclusive: bool = True):
    """Shorten a line consisting of a list of coordinates by a set length.
    The new length will be old_length - cut_length.
    The coordinates are assumed to be in the same unit as length.
    If length is longer than the total length of the input line, the returned list will consist of the last two points.

    Args:
        points (list[tuple[float]]): list of tuples of coordinates. The first two items in the tuples are used as x and y coordinates
        cut_length (float): length that should be chopped of the line
        from_start (bool): whether to shorten the line from the start or the end
        inclusive (bool, optional): Whether to overshoot the length or not. If inclusive=True and the removed segment is not exactly cut_length units long, the removed length will be longer than cut_length, else it will be shorter. Defaults to True.
    Returns:
        points (list[tuple[float]]): a shortened copy of the coordinates list 
    """
    if len(points) < 3:
        return points

    input = points if from_start else list(reversed(points))
    output = []

    compound_length = 0
    for point1, point2 in zip(input[:-1], input[1:]):
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]

        if compound_length > cut_length:
            output.append(point1)
        else:
            intermediate_length = get_distance(x1, y1, x2, y2)
            compound_length += intermediate_length

    if len(output) == 0:
        output = [point1, point2]
    else:
        output.append(point2)  # add last point

    if not from_start:
        output.reverse()

    return output


def shorten_linestring(line: LineString, cut_length: float, from_start: bool, min_length: float = 1.0):
    """Shorten a linestring by a set length.
    The new length will be old_length - cut_length.
    The coordinates are assumed to be in the same unit as length.
    If the new length is shorter than min_length, the length will be capped to min_length

    Args:
        line (LineString): list of tuples of coordinates. The first two items in the tuples are used as x and y coordinates
        cut_length (float): length that should be chopped of the line
        from_start (bool): whether to shorten the line from the start or the end
        min_length (float): Minimum length of the shortened line. If the original length is less than min_length, the original line will be returned unchanged
    Returns:
        points (list[tuple[float]]): a shortened copy of the coordinates list 
    """

    length = line.length
    if length < min_length:
        return line
    if length - cut_length < min_length:
        cut_length = length - min_length
    if not from_start:
        line = line.reverse()

    coords = list(line.coords)
    for i, p in enumerate(coords):
        distance = line.project(Point(p))
        if distance == cut_length:
            out_line = LineString(line.coords[i:])
            break
        if distance > cut_length:
            cut_point = line.interpolate(cut_length)
            out_line = LineString([(cut_point.x, cut_point.y, cut_point.z)] + coords[i:])
            break

    if not from_start:
        out_line = out_line.reverse()

    return out_line


def normalize_s_offsets(offsets, end_offset):
    """Normalize a list of offsets so that offsets[0] = 0 and end_offset would
    be 1 if it was a part of offsets

    Args:
        offsets (list[float]): list of s_offsets (range 0.0-1.1)
        end_offset (float): final s_offset. Will not be included in output

    >>> normalize_s_offsets([0.1, 0.2], 0.5)
    [0.0, 0.25]

    """
    if len(offsets) < 2:
        return [0.0]

    return [(offset - offsets[0]) / (end_offset - offsets[0]) for offset in offsets]
