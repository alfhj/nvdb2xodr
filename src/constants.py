DATA_PATH = "../Notebooks"
CENTER_COORDS = (63.41771242884644, 10.40335350836009)

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
