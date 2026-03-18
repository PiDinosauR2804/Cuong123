import glob
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import run_multivisit_experiment as exp


def main():
    exp.INSTANCES = sorted(glob.glob(r"test_data/special_data/symmetric_patch_multivisit_30_100/20/*.dat"))
    exp.THETAS = [1, 2]
    exp.RUNS = 3
    exp.DRONE_COUNTS = [2]
    exp.TRUCK_COUNTS = [2]
    exp.DRONE_CAPACITIES = [4]
    exp.DRONE_LIMIT_TIMES = [60,90, 120]
    exp.WORKERS = 4
    exp.OUTPUT_CSV = os.path.join("result", f"multivisit_symmetric20_30_100_w4_{int(time.time())}.csv")

    print("instances=", len(exp.INSTANCES))
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
