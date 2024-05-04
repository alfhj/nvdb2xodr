from enum import Enum


class LaneType(Enum):
    NORMAL = "N"
    PUBLIC_TRANSPORT = "K" # Kollektiv
    BICYCLE = "S" # Sykkel
    LEFT_TURN = "V" # Venstre
    RIGHT_TURN = "H" # Høyre
    EXTRA = "O" # Oppstilling
    TOLL_BOOTH = "B" # Bomstasjon
    PASSING = "F" # Forbikjøringsfelt
    REVERSIBLE = "R" # Reversibelt
    INVALID = "I"


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


class RoadSegment:
    def __init__(self):
        self.reference_line: list[tuple[float]] = []
        self.same_lanes: list[Lane] = []  # ordered from center to edge
        self.opposite_lanes: list[Lane] = []  # ordered from center to edg

    def add_lane(self, lane: Lane):
        if lane.same_direction:
            lane.id = len(self.same_lanes) + 1
            self.same_lanes.append(lane)
        else:
            lane.id = -(len(self.opposite_lanes) + 1)
            self.opposite_lanes.append(lane)

    def get_lanes(self) -> list[Lane]:
        center_lane = Lane(LaneType.INVALID, True, 0)
        return sorted(self.opposite_lanes, key=lambda l: l.id) + [center_lane] + sorted(self.same_lanes, key=lambda l: l.id)
