DATA_PATH = "../Notebooks"

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
