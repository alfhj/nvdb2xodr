from pathlib import Path
import numpy as np
import json

import pyproj

from .road import Road, Lane, LaneType
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


def giveHeading(x1, y1, x2, y2):
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


def distance(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_road(input: list[str]) -> Road:
    road = Road()

    for nvdb_lane in input:
        same_direction = int(nvdb_lane[0]) % 2 == 1
        lane_type = LaneType.NORMAL
        for special_type in LaneType.__members__.values():
            if special_type.value in nvdb_lane:
                lane_type = special_type
                break
        lane = Lane(lane_type, same_direction)
        road.add_lane(lane)

    return road


#transform = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:5973")
transform = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:25833")
center = transform.transform(CENTER_COORDS[0], CENTER_COORDS[1])


def get_relative_coordinates(x, y):
    return (x - center[0], y - center[1])
