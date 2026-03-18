import glob
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
	sys.path.insert(0, SCRIPT_DIR)

import run_multivisit_experiment as exp


CLUSTER_PATTERN = r"test_data/data_demand_random_50_batch_all1_equal_cluster/C201*.dat"
RANDOM_PATTERN = r"test_data/data_demand_random_50_batch_all1_equal_random/C201*.dat"


cluster_instances = sorted(glob.glob(CLUSTER_PATTERN))
random_instances = sorted(glob.glob(RANDOM_PATTERN))
exp.INSTANCES = cluster_instances + random_instances

# Requested parameter grid
exp.THETAS = [1, 2]
exp.RUNS = 5
exp.DRONE_COUNTS = [1]
exp.TRUCK_COUNTS = [2]
exp.DRONE_CAPACITIES = [4, 8]
exp.DRONE_LIMIT_TIMES = [60, 90, 120]
exp.WORKERS = None

print("cluster_instances=", len(cluster_instances))
print("random_instances=", len(random_instances))
print("total_instances=", len(exp.INSTANCES))
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
