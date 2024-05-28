import re
import lxml.etree as ET
from lxml.etree import Element, ElementTree
from datetime import datetime
from shapely import from_wkt, get_num_points, line_merge
from shapely.ops import linemerge, unary_union
from shapely.geometry import Point, LineString, MultiLineString
from src.utils import *
from src.road import JunctionConnection, JunctionRoad, LaneType, Road, RoadNetwork, RoadSegment
from src.constants import CENTER_COORDS, JUNCTION_MARGIN, road_types, detail_levels


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


def get_nodes(roads: list) -> dict[int, list[int]]:
    """Get nodes from NVDB roads

    Args:
        roads (list): NVDB veglenker
    Returns:
        nodes (dict[int, list[int]]): Dictionary that maps node IDs to a list of road sequence IDs that are connected to that node
    """

    nodes = {}
    for sequence in roads:
        for chain in filter_road_sequence(sequence):
            for endpoint in ["startport", "sluttport"]:
                portal = next(portal for portal in sequence["porter"] if portal["id"] == chain[endpoint])
                node_id = portal["tilkobling"]["nodeid"]
                nodes.setdefault(node_id, []).append(sequence["veglenkesekvensid"])

    return nodes


def split_road_into_parts(roads: list):
    """Split road

    Args:
        roads (list): _description_
    """
    pass


def startBasicXODRFile() -> Element:
    root = ET.Element("OpenDRIVE")
    header = ET.SubElement(root, "header", revMajor="1", revMinor="6", name="Glosehaugen", version="0.02", date=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"))
    #ET.SubElement(header, "geoReference").text = ET.CDATA(f"+proj=tmerc +lat_0=0 +lon_0=0 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +vunits=m")
    ET.SubElement(header, "geoReference").text = ET.CDATA(f"+proj=tmerc +lat_0={CENTER_COORDS[0]} +lon_0={CENTER_COORDS[1]} +x_0=0 +y_0=0 +ellps=GRS80 +units=m +vunits=m")
    return root


def generate_single_road(sequence: dict, road_object: Road) -> Element:
    # start road
    road = ET.Element("road", name=sequence["adresse"], id=road_object.id, rule="RHT", junction="-1")
    link = ET.SubElement(road, "link")
    roadType = ET.SubElement(road, "type", s="0", type="town")
    ET.SubElement(roadType, "speed", max="50", unit="km/h")

    # create geometry
    planView = ET.SubElement(road, "planView")
    elevationProfile = ET.SubElement(road, "elevationProfile")

    total_length = 0
    for p in road_object.reference_line[:-1]:
        geometry = ET.Element("geometry", s=str(total_length), x=str(p.x), y=str(p.y), hdg=str(p.heading), length=str(p.length))
        ET.SubElement(geometry, "line")
        ET.SubElement(elevationProfile, "elevation", s=str(total_length), a=str(p.z), b=str(p.slope), c="0", d="0")

        planView.append(geometry)
        total_length += p.length

    road.set("length", str(total_length))

    #add lanes
    lanes = ET.SubElement(road, "lanes")
    ET.SubElement(lanes, "laneOffset", s="0", a="0", b="0", c="0", d="0")
    for lane_segment in road_object.lanes:
        laneSection = ET.SubElement(lanes, "laneSection", s=str(lane_segment.s_offset))
        left = ET.SubElement(laneSection, "left")
        center = ET.SubElement(laneSection, "center")
        right = ET.SubElement(laneSection, "right")
        for nvdb_lane in lane_segment.segment.get_lanes():
            if nvdb_lane.type == LaneType.INVALID:
                lane = ET.SubElement(center, "lane", id="0", type="none", level="false")
                ET.SubElement(lane, "roadMark", sOffset="0", type="broken", material="standard", color="white", width="0.125", laneChange="none")
                continue

            parent = right if nvdb_lane.same_direction else left
            lane = ET.SubElement(parent, "lane", id=str(nvdb_lane.id), type=nvdb_lane.get_xodr_lane_type(), level="false")
            ET.SubElement(lane, "link")
            ET.SubElement(lane, "width", sOffset="0", a=str(nvdb_lane.width), b="0", c="0", d="0")
            ET.SubElement(lane, "roadMark", sOffset="0", type="solid", material="standard", color="white", laneChange="none")

    return road


def generate_road_sequence(root: Element, sequence: dict, nodes: dict[int, list[int]], road_network: RoadNetwork):
    chains = filter_road_sequence(sequence)
    portals_to_nodes = {portal["id"]: portal["tilkobling"]["nodeid"] for portal in sequence["porter"]}

    start_node_id = None
    road_segments = []
    id_suffix = 1
    for i, chain in enumerate(chains):
        if "feltoversikt" not in chain:
            continue

        points_string = re.search(r"LINESTRING Z\((.*)\)", chain["geometri"]["wkt"]).group(1)
        points_list = [tuple(float(p) for p in ps.strip().split(" ")) for ps in points_string.split(",")]
        if len(points_list) < 2:
            continue

        lanes_list = chain["feltoversikt"]
        road_segment = RoadSegment(points_list)
        road_segment.add_nvdb_lanes(lanes_list, chain.get("vegbredde"))
        road_segments.append(road_segment)

        start_node_id = start_node_id if start_node_id is not None else portals_to_nodes[chain["startport"]]
        end_node_id = portals_to_nodes[chain["sluttport"]]

        #print(len(nodes[end_node_id]))
        if len(nodes[end_node_id]) <= 2 and i != len(chains) - 1:  # normal road connection and not at end
            continue  # merge current with next road segment

        # make road
        road_id = f"{sequence['veglenkesekvensid']}_{id_suffix}"
        road_object = Road(road_segments, road_id, shorten=JUNCTION_MARGIN)
        road_network.add_road(road_object)
        road_network.add_junction(str(start_node_id), JunctionConnection(road_object, start=True))
        road_network.add_junction(str(end_node_id), JunctionConnection(road_object, start=False))
        road = generate_single_road(sequence, road_object)
        root.append(road)

        start_node_id = None
        road_segments = []
        id_suffix += 1

    return root


def generate_junction_road(road_object: JunctionRoad, junction_id: str, in_road: JunctionConnection, out_road: JunctionConnection) -> Element:
    aU, aV, bU, bV, cU, cV, dU, dV = road_object.params

    # start road
    road = ET.Element("road", name=road_object.id, id=road_object.id, junction=junction_id, length=str(road_object.length))
    link = ET.SubElement(road, "link")
    ET.SubElement(link, "predecessor", elementType="road", elementId=in_road.road.id, contactPoint="start" if in_road.start else "end")
    ET.SubElement(link, "successor", elementType="road", elementId=out_road.road.id, contactPoint="start" if out_road.start else "end")
    roadType = ET.SubElement(road, "type", s="0", type="town")
    ET.SubElement(roadType, "speed", max="50", unit="km/h")

    # create geometry
    planView = ET.SubElement(road, "planView")
    elevationProfile = ET.SubElement(road, "elevationProfile")
    geometry = ET.SubElement(planView, "geometry", s="0", x=str(road_object.start_point.x), y=str(road_object.start_point.y), hdg=str(road_object.start_point.heading), length=str(road_object.length))
    ET.SubElement(geometry, "paramPoly3", aU=str(aU), aV=str(aV), bU=str(bU), bV=str(bV), cU=str(cU), cV=str(cV), dU=str(dU), dV=str(dV), pRange="normalized")
    ET.SubElement(elevationProfile, "elevation", s="0", a=str(road_object.start_point.z), b=str(road_object.slope), c="0", d="0")

    #add lanes
    lane_object = road_object.lanes[0]
    lanes = ET.SubElement(road, "lanes")
    ET.SubElement(lanes, "laneOffset", s="0", a="0", b="0", c="0", d="0")
    laneSection = ET.SubElement(lanes, "laneSection", s="0")
    center = ET.SubElement(laneSection, "center")
    right = ET.SubElement(laneSection, "right")
    center_lane = ET.SubElement(center, "lane", id="0", type="none", level="false")
    #ET.SubElement(center_lane, "roadMark", sOffset="0", type="broken", material="standard", color="white", width="0.125", laneChange="none")
    lane = ET.SubElement(right, "lane", id=str(lane_object.id), type=lane_object.get_xodr_lane_type(), level="false")
    ET.SubElement(lane, "link")
    ET.SubElement(lane, "width", sOffset="0", a=str(lane_object.width), b="0", c="0", d="0")
    #ET.SubElement(lane, "roadMark", sOffset="0", type="solid", material="standard", color="white", laneChange="none")

    return road


def generate_junctions(root: Element, road_network: RoadNetwork):
    """Generates OpenDrive junctions. The junctions consist of several roads connecting each lane into the junction.
    The number of generated roads in each junction will be ,

    Args:
        root (Element): _description_
        roads (dict): _description_
        node (tuple[int, list[int]]): _description_
    """
    junction_elements = []

    for junction_id, connections in road_network.junctions.items():
        #if len(connections) < 3:
        #    continue

        junction_element = ET.Element("junction", id=junction_id, name=junction_id)

        i = 0
        for connection in connections:
            connection_element = None
            connected_road = root.find(f"./road[@id='{connection.road.id}']/link")
            ET.SubElement(connected_road, "predecessor" if connection.start else "successor", elementType="junction", elementId=junction_id)

            segment = connection.road.lanes[0 if connection.start else -1].segment
            incoming_lanes = segment.opposite_lanes if connection.start else segment.same_lanes
            endpoint = connection.road.reference_line[0 if connection.start else -1].copy()
            heading = connection.road.reference_line[0 if connection.start else -2].heading
            endpoint.heading = (heading + pi) % tau if connection.start else heading

            offset = 0
            for lane in incoming_lanes:
                if not lane.is_drivable():
                    continue

                x, y = get_perpendicular_point(endpoint.x, endpoint.y, endpoint.heading, -offset)
                endpoint.x = x
                endpoint.y = y

                for out_connection in connections:
                    if out_connection is connection:
                        continue

                    out_segment = out_connection.road.lanes[0 if out_connection.start else -1].segment
                    out_lanes = out_segment.same_lanes if out_connection.start else out_segment.opposite_lanes
                    out_endpoint = out_connection.road.reference_line[0 if out_connection.start else -1].copy()
                    out_heading = out_connection.road.reference_line[0 if out_connection.start else -2].heading
                    out_endpoint.heading = out_heading if out_connection.start else (out_heading + pi) % tau

                    out_offset = 0
                    for out_lane in out_lanes:
                        if not out_lane.is_drivable():
                            continue

                        x, y = get_perpendicular_point(out_endpoint.x, out_endpoint.y, out_endpoint.heading, -out_offset)
                        out_endpoint.x = x
                        out_endpoint.y = y

                        junction_name = f"junction_{connection.road.id}_to_{out_connection.road.id}_lane_{lane.id}_to_{out_lane.id}"
                        junction_road = JunctionRoad(endpoint, out_endpoint, junction_name)
                        road = generate_junction_road(junction_road, junction_id, connection, out_connection)
                        root.append(road)

                        connection_element = ET.SubElement(junction_element, "connection", id=str(i), incomingRoad=connection.road.id, connectingRoad=junction_road.id, contactPoint="start")
                        ET.SubElement(connection_element, "laneLink", **{"from": str(lane.id)}, to=str(out_lane.id))

                        out_offset += out_lane.width
                        i += 1
                offset += lane.width
        junction_elements.append(junction_element)
    root.extend(junction_elements)


if __name__ == "__main__":
    input_file = "veglenkesekvens2a.json"
    output_file = "../OpenDrive/gloshaugen_nvdb.xodr"
    print(f"Converting NVDB file {input_file} to OpenDrive format")
    start_time = datetime.now()

    roads = load_json(get_file_path(input_file))

    merge_linked_locations(roads)
    nodes = get_nodes(roads)
    road_network = RoadNetwork()

    root = startBasicXODRFile()

    # road sequence > road chain > road segment
    i = 0
    for sequence in roads:
        generate_road_sequence(root, sequence, nodes, road_network)
        i += 1
        #if i == 19:
        #    break

    generate_junctions(root, road_network)

    # set min/max coordinates
    header = root.find("header")
    minx, miny, maxx, maxy = road_network.minmax_xy
    header.set("west", str(minx))
    header.set("south", str(miny))
    header.set("east", str(maxx))
    header.set("north", str(maxy))

    #ElementTree.tostring(xodr, xml_declaration=True)
    ET.indent(root, space="    ")
    #print(ET.tostring(xodr, doctype='<?xml version="1.0" encoding="UTF-8"?>', pretty_print=True).decode())
    ElementTree(root).write(get_file_path(output_file), doctype='<?xml version="1.0" encoding="UTF-8"?>', encoding="utf-8")

    total_time = datetime.now() - start_time
    print(f"Finished in {total_time.total_seconds():.2f} seconds")
