from src.utils import *
import math
import doctest
import importlib.util
import os
from src import utils
import random


def import_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    foo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(foo)
    return foo


if __name__ == "__main__":
    doctest.testmod(utils, verbose=True)

    #prefix = "src"
    #for name in os.listdir(prefix):
    #    if not name.endswith(".py"):
    #        continue

    #for i in [1, 2, 5, 10, 20, 50, 100, 1000, 10000]:
    #    params, length = calculate_cubic_curve((66.9, 38.3), 0, i)
    #    print(i, length)

    for _ in range(10):
        #x1 = random.uniform(-1000, 1000)
        #y1 = random.uniform(-1000, 1000)
        #x2 = random.uniform(-1000, 1000)
        #y2 = random.uniform(-1000, 1000)
        var = {name: random.uniform(-1000, 1000) for name in ["x1", "y1", "x2", "y2"]}
        var["x2-x1"] = var["x2"] - var["x1"]
        var["y2-y1"] = var["y2"] - var["y1"]
        var["ang1"] = get_heading(var["x1"], var["y1"], var["x2"], var["y2"])
        var["ang2"] = get_heading1(var["x1"], var["y1"], var["x2"], var["y2"])
        assert math.isclose(var["ang1"], var["ang2"])
        for name in var:
            print(name, var[name])
        print()
