from dataclasses import dataclass
from enum import Enum

from .utils import *


class LaneType(Enum):
    NORMAL = "N"
    INVALID = "I"
    PUBLIC_TRANSPORT = "K" # Kollektiv
    BICYCLE = "S" # Sykkel
    LEFT_TURN = "V" # Venstre
    RIGHT_TURN = "H" # Høyre
    EXTRA = "O" # Oppstilling
    TOLL_BOOTH = "B" # Bomstasjon
    PASSING = "F" # Forbikjøringsfelt
    REVERSIBLE = "R" # Reversibelt


class Lane:
    def __init__(self, lane_type: LaneType, same_direction: bool, width: float):
        self.type = lane_type
        self.same_direction = same_direction
        self.width = width
        self.id = None

    def get_xodr_lane_type(self):
        return {
            LaneType.NORMAL: "driving",
            LaneType.BICYCLE: "biking",
            LaneType.LEFT_TURN: "exit",
            LaneType.RIGHT_TURN: "exit"
        }.get(self.type, "driving")


@dataclass
class ReferenceLinePoint:
    x: float  # x-coordinate in meters relative to center point
    y: float  # y-coordinate in meters relative to center point
    z: float  # z-coordinate in meters relative to center point
    length: float  # length from current point to next point
    heading: float  # heading from current point to next point
    slope: float  # slope from current point to next point

    @property
    def coords(self) -> tuple[float]:
        return (self.x, self.y, self.z)


class RoadSegment:
    def __init__(self, points: list[tuple[float]]):
        self.reference_line = points
        self.length = get_total_length(points)
        self.nvdb_lanes: list[str] = []
        self.same_lanes: list[Lane] = []  # ordered from center to edge
        self.opposite_lanes: list[Lane] = []  # ordered from center to edg

    def add_lane(self, lane: Lane):
        if lane.same_direction:
            lane.id = len(self.same_lanes) + 1
            self.same_lanes.append(lane)
        else:
            lane.id = -(len(self.opposite_lanes) + 1)
            self.opposite_lanes.append(lane)

    def add_nvdb_lanes(self, input: list[str], total_width: float = None):
        self.nvdb_lanes = input
        biking_width = 1.5
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
            self.add_lane(lane)

    def get_lanes(self) -> list[Lane]:
        center_lane = Lane(LaneType.INVALID, True, 0)
        return sorted(self.opposite_lanes, key=lambda l: l.id) + [center_lane] + sorted(self.same_lanes, key=lambda l: l.id)


class Road:
    def __init__(self, road_segments: list[RoadSegment], shorten: float = 0):
        self.reference_line: list[ReferenceLinePoint] = []
        self.lanes: list[tuple[float, Lane]] = []

        points = road_segments[0].reference_line
        for segment in road_segments[1:]:
            points.extend(segment.reference_line[1:])

        for i in range(len(points) - 1):
            x1, y1, z1 = points[i]
            x2, y2, z2 = points[i+1]

            if z2 == -999999:  # missing z placeholder
                _, _, z3 = points[i+2]
                z2 = (z1 + z3) / 2
                points[i+1] = (x2, y2, z2)

            x1, y1 = get_relative_coordinates(x1, y1)
            x2, y2 = get_relative_coordinates(x2, y2)
            heading = get_heading(x1, y1, x2, y2)
            length = get_length(x1, y1, x2, y2)
            slope = (z2 - z1) / length
            self.reference_line.append(ReferenceLinePoint(x1, y1, z1, length, heading, slope))
        self.reference_line.append(ReferenceLinePoint(x2, y2, z2, 0, heading, slope))

        self.reference_line, cut_start = shorten_point_list(self.reference_line, cut_length=shorten, from_start=True)
        self.reference_line, cut_end = shorten_point_list(self.reference_line, cut_length=shorten, from_start=False)
        self.length = sum(segment.length for segment in road_segments) - cut_start - cut_end

        previous_offset = None
        previous_lanes = None
        length = 0
        for segment in road_segments:
            s_offset = max(0, length - cut_start)
            if segment.nvdb_lanes != previous_lanes and s_offset < self.length:
                if s_offset == previous_offset:
                    self.lanes.pop()

                self.lanes.append((s_offset, segment))
                previous_offset = s_offset
                previous_lanes = segment.nvdb_lanes

            length += segment.length


class RoadNetwork:
    def __init__(self):
        self.roads: list[Road] = []
        self.nodes: dict[int, list[int]] = []
        self.minmax_xy = [1e9, 1e9, -1e9, -1e9]  # min_x, min_y, max_x, max_y

    def add_road(self, road: Road):
        for p in road.reference_line:
            if p.x < self.minmax_xy[0]:
                self.minmax_xy[0] = p.x
            if p.y < self.minmax_xy[1]:
                self.minmax_xy[1] = p.y
            if p.x > self.minmax_xy[2]:
                self.minmax_xy[2] = p.x
            if p.y > self.minmax_xy[3]:
                self.minmax_xy[3] = p.y

        self.roads.append(road)

    def get_road(self, road_id: int):
        pass

    def get_junction_roads(self, node_id: int):
        pass


def shorten_point_list(points: list[ReferenceLinePoint], cut_length: float, from_start: bool, min_length: float = 1.0) -> tuple[list[ReferenceLinePoint], float]:
    """Shorten a line consisting of a list of coordinates by a set length.
    The new length will be old_length - cut_length.
    The coordinates are assumed to be in the same unit as length.

    Args:
        points (list[ReferenceLinePoint]): list of points
        cut_length (float): length that should be chopped of the line
        from_start (bool): whether to shorten the line from the start or the end
    Returns:
        points (list[ReferenceLinePoint]): a shortened copy of the points list 
    """
    total_length = sum(p.length for p in points)

    if total_length < min_length or cut_length <= 0:
        return points, 0
    if total_length - cut_length < min_length:
        cut_length = total_length - min_length
    if not from_start:
        points = list(reversed(points))

    output = []

    cumulative_length = 0
    for i in range(len(points) - 1):
        x1, y1, z1 = points[i].coords
        x2, y2, z2 = points[i+1].coords
        length = get_length(x1, y1, x2, y2)
        cumulative_length += length

        if cumulative_length == cut_length:
            output = points[i:]
            break
        if cumulative_length > cut_length:
            t = (cumulative_length - cut_length) / length
            x3 = x1 * t + x2 * (1 - t)
            y3 = y1 * t + y2 * (1 - t)
            z3 = z1 * t + z2 * (1 - t)
            cut_point = ReferenceLinePoint(x3, y3, z3, cumulative_length - cut_length, points[i].heading, points[i].slope)
            output = [cut_point] + points[i+1:]
            break

    if not from_start:
        output.reverse()

    return output, cut_length
