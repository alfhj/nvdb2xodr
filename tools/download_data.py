import requests
from tqdm import tqdm
from src.utils import dump_json


def get_json(url, params):
    resp = requests.get(url, params=params)
    try:
        resp.raise_for_status()
    except requests.exceptions.HTTPError as e:
        print(f"HTTP error occurred: {e}")
        print(f"Response content: {resp.content}")
        raise
    return resp.json()


def get_roads(coords, **kwargs):
    url = "https://nvdbapiles-v3.atlas.vegvesen.no/vegnett/veglenkesekvenser"
    params = {
        "srid": "25833",
        "kartutsnitt": coords,
        "alle_versjoner": "true",
        **kwargs
    }
    resp = get_json(url, params)
    return resp["objekter"]


def get_objects(obj_type, coords, **kwargs):
    url = f"https://nvdbapiles-v3.atlas.vegvesen.no/vegobjekter/{obj_type}"
    params = {
        "srid": "25833",
        #"vegsystemreferanse": "5001 KV3058 K S1D1", #change vegsystemreferanse to the FV number or EV number you want
        "segmentering": "false",
        "kartutsnitt": coords,
        "alle_versjoner": "false",
        "inkluder": "egenskaper",
        **kwargs
    }
    resp = get_json(url, params)
    return resp["objekter"]


def merge_data(addresses, roads, widths):
    print("Merging addresses")
    id_to_address = {}
    for address in addresses:
        roadName = next(egenskap for egenskap in address["egenskaper"] if egenskap["navn"] == "Adressenavn")["verdi"]
        location = next(egenskap for egenskap in address["egenskaper"] if egenskap["navn"] == "Liste av lokasjonsattributt")
        for innhold in location["innhold"]:
            sequenceId = innhold["veglenkesekvensid"]
            id_to_address[sequenceId] = roadName

    road_merged = []
    for road in tqdm(roads):
        new_road = road.copy()
        for chain in new_road["veglenker"]:
            middle_pos = (chain["startposisjon"] + chain["sluttposisjon"]) / 2
            for width_info in widths:
                properties = next(egenskap for egenskap in width_info["egenskaper"] if egenskap["navn"] == "Liste av lokasjonsattributt")["innhold"][0]
                if properties["veglenkesekvensid"] == road["veglenkesekvensid"] and properties["startposisjon"] <= chain["startposisjon"] < properties["sluttposisjon"]:
                    width = next((egenskap for egenskap in width_info["egenskaper"] if egenskap["navn"] == "Vegbredde, totalt"), None) \
                        or next((egenskap for egenskap in width_info["egenskaper"] if egenskap["navn"] == "Dekkebredde"), None) \
                        or next((egenskap for egenskap in width_info["egenskaper"] if egenskap["navn"] == "Kjørebanebredde"), None)
                    chain["vegbredde"] = width.get("verdi")
                    break

        new_road["adresse"] = id_to_address.get(road["veglenkesekvensid"], "")
        road_merged.append(new_road)
    return road_merged


def run(boundary: str, output_file: str):
    print("Downloading roads")
    roads = get_roads(boundary)

    print("Downloading addresses")
    addresses = get_objects(538, boundary)

    print("Downloading road widths")
    widths = get_objects(583, boundary)

    print("Merging addresses and road widths")
    road_merged = merge_data(addresses, roads, widths)

    print("Writing JSON output")
    dump_json(road_merged, output_file)
