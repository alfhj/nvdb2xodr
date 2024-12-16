# nvdb2xodr
Convert Vegvesen NVDB road networks to OpenDRIVE files

## Usage:
1. Install requirements: `pip install -r requirements.txt`
2. Make a new configuration of the area you want to convert in `run_tools.py` and make sure `run_download_data()` is run in `__main__`
3. Run the download script: `run_tools.py`
4. Make a new configuration in `nvdb_to_opendrive.py` using the same JSON file
5. Run it: `python nvdb_to_opendrive.py`

## TODO:
- [ ] Handle changing number of lanes properly
- [ ] Infer turn restrictions
- [ ] Incorporate optional road polygon data
- [ ] Expand to OSM dataset
- [ ] Lane access for bus lanes
- [ ] Setting types for exit ramps
- [x] Fix roundabouts and missing roads
- [x] Junctions