import json
import os
import uuid
from tools import download_data
import nvdb_to_opendrive
from pathlib import Path
from time import time


def run_download_data():
    gløshaugen = ("270000,7039700,271200,7041000", "../Notebooks/veglenkesekvens_gloshaugen.json")
    sandmoen = ("267500,7030500,268500,7031500", "../Notebooks/veglenkesekvens_sandmoen.json")

    download_data.run(*sandmoen)


def benchmark_function(function, measure_memory=True):
    if measure_memory:
        import tracemalloc
        tracemalloc.start()

    start_time = time()
    result = function()
    total_time = time() - start_time

    response = {"execution_time": total_time}
    if result is not None:
        response["result"] = result

    if measure_memory:
        memory_usage = tracemalloc.get_traced_memory()
        response["memory_usage"] = memory_usage
        tracemalloc.stop()

    return response


def run_download_data_multi():
    center = (268000, 7031000)
    width_min = 500
    width_max = 5000
    width_delta = 100
    def output_file(width): return f"../Notebooks/nvdb_multi_sandmoen/veglenkesekvens_sandmoen_{width}.json"
    output_path = "../Notebooks/nvdb_download_times_sandmoen.json"

    Path(output_file("")).parent.mkdir(exist_ok=True)

    benchmark_results = []
    for width in range(width_min, width_max + 1, width_delta):
        print(f"Downloading area of size {width}*{width} m^2")
        boundary = f"{center[0]-width//2},{center[1]-width//2},{center[0]+width//2},{center[1]+width//2}"
        result = benchmark_function(lambda: download_data.run(boundary, output_file(width)), measure_memory=False)
        result["width"] = width
        benchmark_results.append(result)

    with open(output_path, "w") as f:
        json.dump(benchmark_results, f, indent=4)


def run_nvdb_to_opendrive_multi(measure_memory=True):
    input_path = "../Notebooks/nvdb_multi_sandmoen"
    output_path = "../Notebooks/nvdb_execution_times_sandmoen.json"

    files = {}
    for file in Path(input_path).glob("*.json"):
        width = int(file.stem.split("_")[-1])
        files[width] = file

    benchmark_results = []
    for width, file in sorted(files.items()):
        print(f"Running nvdb_to_opendrive.py on area of size {width}*{width} m^2")
        output_file = f"temp-{uuid.uuid4()}.xodr"
        config = nvdb_to_opendrive.Config(str(file), output_file, "")

        result = benchmark_function(lambda: nvdb_to_opendrive.main(config))
        result["width"] = width
        benchmark_results.append(result)

        os.remove(output_file)

    with open(output_path, "w") as f:
        json.dump(benchmark_results, f, indent=4)


if __name__ == "__main__":
    run_download_data()
    #run_download_data_multi()
    #run_nvdb_to_opendrive_multi()
