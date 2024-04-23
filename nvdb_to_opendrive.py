import re
import lxml.etree as ET
from lxml.etree import Element, ElementTree
from datetime import datetime
from src.utils import get_relative_coordinates, giveHeading, distance, load_json, get_road, get_file_path
from src.road import LaneType
from src.constants import CENTER_COORDS, road_types, detail_levels


def SubElement(parent: Element, **kwargs):
    for key in kwargs:
        kwtype = type(kwargs[key])
        if kwtype == float:
            kwargs[key] = f"{kwargs[key]:.17f}"
        elif kwtype != str:
            kwargs[key] = str(kwargs[key])
    ET.SubElement


Element.SubElement = SubElement


def filter_road_sequence(sequence):
    out = []
    for chain in sequence["veglenker"]:
        if chain["detaljnivå"] not in detail_levels:
            continue
        if chain["typeVeg"] not in road_types:
            continue
        if "sluttdato" in chain:
            continue
        out.append(chain)
    out.sort(key=lambda c: c["startposisjon"])

    return out


def merge_linked_locations(roads: list):
    """Merge road chains that are linked with 'superstedfesting' to their respective roads chain sequences

    Args:
        roads (list): list of NVDB road sequences
    """
    for sequence in roads:
        for chain in filter_road_sequence(sequence):
            if "superstedfesting" in chain:
                new_chain = chain.copy()
                link_info = new_chain.pop("superstedfesting")
                linked_sequence = next(seq for seq in roads if seq["veglenkesekvensid"] == link_info["veglenkesekvensid"])

                new_chain["startposisjon"] = link_info["startposisjon"]
                new_chain["sluttposisjon"] = link_info["sluttposisjon"]
                new_chain["feltoversikt"] = link_info["kjørefelt"]

                new_chains = []
                for linked_chain in linked_sequence["veglenker"]:
                    if linked_chain["veglenkenummer"] == new_chain["veglenkenummer"]:
                        new_chains.append(new_chain)
                    else:
                        new_chains.append(linked_chain)
                linked_sequence["veglenker"] = new_chains

                chain["sluttdato"] = "MOVED"  # mark original chain as outdated


def startBasicXODRFile() -> Element:
    root = ET.Element("OpenDRIVE")
    header = ET.SubElement(root, "header", revMajor="1", revMinor="8", name="Glosehaugen", version="0.02", date=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"))
    #ET.SubElement(header, "geoReference").text = ET.CDATA(f"+proj=tmerc +lat_0=0 +lon_0=0 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +vunits=m")
    ET.SubElement(header, "geoReference").text = ET.CDATA(f"+proj=tmerc +lat_0={CENTER_COORDS[0]} +lon_0={CENTER_COORDS[1]} +x_0=0 +y_0=0 +ellps=GRS80 +units=m +vunits=m")
    return root


def fillNormalRoad(root: Element, sequence):
    # start road
    road = ET.Element("road", name=sequence["adresse"], id=str(sequence["veglenkesekvensid"]), rule="RHT", junction="-1")
    link = ET.SubElement(road, "link")
    #ET.SubElement(link, "predecessor", elementType="junction", elementId=road_input["startJunction"])
    #ET.SubElement(link, "successor", elementType="junction", elementId=road_input["endJunction"])
    roadType = ET.SubElement(road, "type", s="0.0", type="town")
    ET.SubElement(roadType, "speed", max="50", unit="km/h")

    # create geometry
    planView = ET.SubElement(road, "planView")
    elevationProfile = ET.SubElement(road, "elevationProfile")
    chains = filter_road_sequence(sequence)
    points = []
    for i, chain in enumerate(chains):
        #if "sluttdato" in chain: #chain["type"] != "HOVED" or
        #    return

        points_string = re.search(r"LINESTRING Z\((.*)\)", chain["geometri"]["wkt"]).group(1)
        points_list = [tuple(float(p) for p in ps.strip().split(" ")) for ps in points_string.split(",")]
        points.extend(points_list[1:] if i > 0 else points_list)

    if len(points) < 2:
        return

    minmax_xy = [points[0][0], points[0][1], points[0][0], points[0][1]]  # min_x, min_y, max_x, max_y
    lengths = []
    for i in range(len(points)-1):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[i+1]

        if z2 == -999999:  # missing z placeholder
            _, _, z3 = points[i+2]
            z2 = (z1 + z3) / 2
            points[i+1] = (x2, y2, z2)

        x1, y1 = get_relative_coordinates(x1, y1)
        x2, y2 = get_relative_coordinates(x2, y2)
        heading = giveHeading(x1, y1, x2, y2)
        length = distance(x1, y1, x2, y2)
        lengths.append(length)
        slope = (z2 - z1) / length

        if x1 < minmax_xy[0]:
            minmax_xy[0] = x1
        if y1 < minmax_xy[1]:
            minmax_xy[1] = y1
        if x1 > minmax_xy[2]:
            minmax_xy[2] = x1
        if y1 > minmax_xy[3]:
            minmax_xy[3] = y1

        geometry = ET.Element("geometry", s=str(sum(lengths[:-1])), x=str(x1), y=str(y1), hdg=str(heading), length=str(length))
        #if element["curvature"] == 0.0:
        ET.SubElement(geometry, "line")
        #else:
        #    geometry.SubElement("arc", curvature=element["curvature"])
        ET.SubElement(elevationProfile, "elevation", s=str(sum(lengths[:-1])), a=str(z1), b=str(slope), c="0.0", d="0.0")

        planView.append(geometry)

    # set min/max coordinates
    header = root.find("header")
    header.set("west", str(minmax_xy[0]))
    header.set("south", str(minmax_xy[0]))
    header.set("east", str(minmax_xy[0]))
    header.set("north", str(minmax_xy[0]))
    road.set("length", str(sum(lengths)))

    #add lanes
    lanes = ET.SubElement(road, "lanes")
    ET.SubElement(lanes, "laneOffset", s="0.0", a="0.0", b="0.0", c="0.0", d="0.0")
    lengths = []
    for chain in chains:
        lengths.append(chain["lengde"])
        laneSection = ET.SubElement(lanes, "laneSection", s=str(sum(lengths[:-1])))
        left = ET.SubElement(laneSection, "left")
        center = ET.SubElement(laneSection, "center")
        right = ET.SubElement(laneSection, "right")
        lanes_list = chain.get("feltoversikt", [])
        nvdb_road = get_road(lanes_list, chain.get("vegbredde"))
        for nvdb_lane in nvdb_road.get_lanes():
            if nvdb_lane.type == LaneType.INVALID:
                lane = ET.SubElement(center, "lane", id="0", type="none", level="false")
                ET.SubElement(lane, "roadMark", sOffset="0.0", type="broken", material="standard", color="white", width="0.125", laneChange="none")
                continue

            parent = right if nvdb_lane.same_direction else left

            lane = ET.SubElement(parent, "lane", id=str(nvdb_lane.id), type=nvdb_lane.get_xodr_lane_type(), level="false")
            ET.SubElement(lane, "link")
            ET.SubElement(lane, "width", sOffset="0.0", a=str(nvdb_lane.width), b="0.0", c="0.0", d="0.0")
            ET.SubElement(lane, "roadMark", sOffset="0.0", type="solid", material="standard", color="white", laneChange="none")

    root.append(road)
    return root


if __name__ == "__main__":
    roads = load_json(get_file_path("veglenkesekvens2a.json"))

    merge_linked_locations(roads)

    root = startBasicXODRFile()

    # road sequence < road chain < road segment
    for sequence in roads:
        #levels = set(chain["detaljnivå"] for chain in sequence["veglenker"])
        #types = set(chain["typeVeg"] for chain in sequence["veglenker"])
        #if any(t not in road_types for t in types): #any(l not in detail_levels for l in levels) or
        #    continue

        fillNormalRoad(root, sequence)

        #for sequence["veglenker"] #chain in sorted(sequence["veglenker"], key=lambda c: c["veglenkenummer"]):
        #    if chain["detaljnivå"] not in detail_levels or chain["typeVeg"] not in road_types:
        #        continue

        #    points_string = re.search(r"LINESTRING Z\((.*)\)", chain["geometri"]["wkt"]).group(1)
        #    points = [tuple(float(p) for p in ps.strip().split(" ")) for ps in points_string.split(",")]
        #    print(points)

    #ElementTree.tostring(xodr, xml_declaration=True)
    ET.indent(root, space="    ")
    #print(ET.tostring(xodr, doctype='<?xml version="1.0" encoding="UTF-8"?>', pretty_print=True).decode())
    ElementTree(root).write(get_file_path("gloshaugen_nvdb.xodr"), doctype='<?xml version="1.0" encoding="UTF-8"?>', encoding="utf-8")
