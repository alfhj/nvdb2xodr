DATA_PATH = "../Notebooks"  # where to load data from
CENTER_COORDS = (63.41771242884644, 10.40335350836009)  # x and y coordinates are specified in meters from this point
JUNCTION_MARGIN = 3  # junctions are generated by shrinking roads JUNCTION_MARGIN meters away from the connection point of the junction, then connecting every lane of the roads going into the junction


#detail_levels = set(chain.get("detaljnivå") for seq in nvdb for chain in seq["veglenker"])
detail_levels = [
    #"Kjørebane",
    "Vegtrase",
    "Vegtrase og kjørebane"
]
road_types = [
    "Enkel bilveg",
    #"Fortau",
    #"Gang- og sykkelveg",
    #"Gangfelt",
    #"Gangveg",
    "Kanalisert veg",
    "Rundkjøring",
    #"Sykkelveg",
    #"Trapp"
]
