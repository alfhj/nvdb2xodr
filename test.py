from src.utils import *
import math
import doctest
import importlib.util
import os
from src import utils


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
