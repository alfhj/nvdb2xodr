from math import atan2, cos, pi, sin, sqrt, tau
from pathlib import Path
from shapely.geometry import Point, LineString
import numpy as np
import json

import pyproj

from .constants import CENTER_COORDS, DATA_PATH

#transform = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:5973")
transform = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:25833")
center = transform.transform(CENTER_COORDS[0], CENTER_COORDS[1])


def dump_json(obj, path, pretty=True):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=4 if pretty else None, ensure_ascii=False)


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def get_file_path(filename):
    return Path(DATA_PATH).joinpath(filename)


def rotate(angle, phi):
    return (angle + phi) % tau


def get_heading(x1, y1, x2, y2):
    return atan2(y2 - y1, x2 - x1) % tau


def get_length(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_total_length(points: list[tuple[float]]):
    total = 0
    for i in range(len(points) - 1):
        x1, y1, _ = points[i]
        x2, y2, _ = points[i+1]
        total += get_length(x1, y1, x2, y2)

    return total


def get_relative_coordinates(x, y):
    return (x - center[0], y - center[1])


def get_uv_coordinates(x1, y1, h1, x2, y2, h2):
    """Get uv coordinates of point (x2, y2) relative to (x1, y1) and its heading h1
    The U-axis will point in the same direction as h1, and the V-axis will point perpendicular to it
    Return u, v, and h, where h is (x2, y2)'s heading in the new uv space
    """
    dist = get_length(x1, y1, x2, y2)
    phi = get_heading(x1, y1, x2, y2)
    phi_uv = phi - h1
    u = dist * cos(phi_uv)
    v = dist * sin(phi_uv)
    h = (h2 - h1) % tau

    return u, v, h


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
    scaling = (abs(end[0]) + abs(end[1])) * 1.0
    aU = aV = bV = 0
    bU = end[0]
    cU = -(cos_phi * scaling - end[0])
    dU = (cos_phi * scaling - end[0])
    cV = -(sin_phi * scaling - 3 * end[1])
    dV = (sin_phi * scaling - 2 * end[1])

    def u(p): return aU + bU * p + cU * p ** 2 + dU * p ** 3
    def v(p): return aV + bV * p + cV * p ** 2 + dV * p ** 3

    ps = [i / length_subdivisions for i in range(length_subdivisions + 1)]
    length = 0
    for p1, p2 in zip(ps[:-1], ps[1:]):
        length += get_length(u(p1), v(p1), u(p2), v(p2))

    return (aU, aV, bU, bV, cU, cV, dU, dV), length


def shorten_coordinate_list(points: list[tuple[float]], cut_length: float, from_start: bool, min_length: float = 1.0):
    """Shorten a line consisting of a list of coordinates by a set length.
    The new length will be old_length - cut_length.
    The coordinates are assumed to be in the same unit as length.

    Args:
        points (list[tuple[float]]): list of tuples of coordinates. The first two items in the tuples are used as x and y coordinates
        cut_length (float): length that should be chopped of the line
        from_start (bool): whether to shorten the line from the start or the end
    Returns:
        points (list[tuple[float]]): a shortened copy of the coordinates list 
    """
    def interp(a, b, t):
        return a * (1 - t) + b * t

    length = get_total_length(points)

    if length < min_length or cut_length <= 0:
        return points
    if length - cut_length < min_length:
        cut_length = length - min_length
    if not from_start:
        points = list(reversed(points))

    output = []

    total_length = 0
    for i in range(len(points) - 1):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[i+1]
        length = get_length(x1, y1, x2, y2)
        total_length += length

        if total_length == cut_length:
            output = points[i:]
        if total_length > cut_length:
            t = (cut_length - total_length) / length
            output = [(interp(x1, x2, t), interp(y1, y2, t), interp(z1, z2, t))] + points[i:]

    if not from_start:
        output.reverse()

    return output, cut_length


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
    if length < min_length or cut_length <= 0:
        return line
    if length - cut_length < min_length:
        cut_length = length - min_length
    if not from_start:
        line = line.reverse()

    coords = list(line.coords)
    for i, p in enumerate(coords):
        total_length = line.project(Point(p))
        if total_length == cut_length:
            out_line = LineString(line.coords[i:])
            break
        if total_length > cut_length:
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


def get_perpendicular_point(x, y, h, d):
    """Get a point that is on the line perpendicular to heading h on point (x, y)
    that is a distance d from (x, y). Negative d gives a point to the right of
    (x, y), and positive d gives a point to the left of (x, y)
    """
    if d == 0:
        return (x, y)

    x1 = x - d * sin(h)
    y1 = y + d * cos(h)

    return (x1, y1)
