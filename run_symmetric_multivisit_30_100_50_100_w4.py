import glob
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import run_multivisit_experiment as exp


def main():
    files_50 = sorted(glob.glob(r"test_data/special_data/symmetric_patch_multivisit_30_100/50/*.dat"))
    files_100 = sorted(glob.glob(r"test_data/special_data/symmetric_patch_multivisit_30_100/100/*.dat"))
    exp.INSTANCES = files_50 + files_100

    exp.THETAS = [1, 2]
    exp.RUNS = 3
    exp.DRONE_COUNTS = [2]
    exp.TRUCK_COUNTS = [2]
    exp.DRONE_CAPACITIES = [4, 8]
    exp.DRONE_LIMIT_TIMES = [60, 90, 120]
    exp.WORKERS = 4
    exp.OUTPUT_CSV = os.path.join(
        "result", f"multivisit_symmetric30_100_50_100_w4_{int(time.time())}.csv"
    )

    print("instances_50=", len(files_50))
    print("instances_100=", len(files_100))
    print("instances_total=", len(exp.INSTANCES))
    print("workers=", exp.WORKERS)

    print(
        "total_tasks=",
        len(exp.INSTANCES)
        * len(exp.THETAS)
        * exp.RUNS
        * len(exp.DRONE_COUNTS)
        * len(exp.TRUCK_COUNTS)
        * len(exp.DRONE_CAPACITIES)
        * len(exp.DRONE_LIMIT_TIMES),
    )

    exp.main()
    print("done_csv=", exp.OUTPUT_CSV)


if __name__ == "__main__":
    main()
