import itertools
import subprocess
import sys
import time
from multiprocessing import Pool
from pathlib import Path
from typing import Optional, Tuple

import pandas as pd


def parse_main_output(stdout: str) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[str]]:
    best_fitness = None
    best_solution = None
    best_multi_visit_fitness = None
    best_multi_visit_solution = None

    in_multi_visit_block = False
    for line in stdout.splitlines():
        stripped = line.strip()
        if line.startswith("Best fitness:"):
            best_fitness = line.split(":", 1)[1].strip()
        elif line.startswith("Best solution:"):
            best_solution = line.split(":", 1)[1].strip()
        elif stripped == "Best multi visit solution found:":
            in_multi_visit_block = True
        elif stripped.startswith("No multi visit solution was found"):
            in_multi_visit_block = False
        elif in_multi_visit_block and stripped.startswith("system_completion_time:"):
            best_multi_visit_fitness = stripped.split(":", 1)[1].strip()
        elif in_multi_visit_block and stripped.startswith("solution:"):
            best_multi_visit_solution = stripped.split(":", 1)[1].strip()

    return best_fitness, best_solution, best_multi_visit_fitness, best_multi_visit_solution


def run_instance(args):
    instance, drone_capacity, drone_limit_time, run_id = args
    cmd = [
        sys.executable,
        "main.py",
        "--instance",
        str(instance),
        "--nimp",
        "30",
        "--seg",
        "8",
        "--div",
        "3",
        "--gamma1",
        "9.0",
        "--gamma2",
        "3.0",
        "--gamma3",
        "1.0",
        "--gamma4",
        "0.2",
        "--seed",
        "42",
        "--A",
        str(drone_capacity),
        "--L_d",
        str(drone_limit_time),
    ]
    print(f"Running: {' '.join(map(str, cmd))}")

    start = time.perf_counter()
    result = subprocess.run(cmd, capture_output=True, text=True)
    runtime_sec = time.perf_counter() - start

    output_file = f"result_{Path(instance).stem}_A{drone_capacity}_L{drone_limit_time}_run{run_id}.txt"
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(result.stdout)
        f.write("\n--- STDERR ---\n")
        f.write(result.stderr)

    best_fitness, best_solution, best_multi_visit_fitness, best_multi_visit_solution = parse_main_output(result.stdout)
    return {
        "instance": instance,
        "A": drone_capacity,
        "L_d": drone_limit_time,
        "run_id": run_id,
        "exit_code": result.returncode,
        "runtime_sec": runtime_sec,
        "best_fitness": best_fitness,
        "best_solution": best_solution,
        "best_multi_visit_solution": best_multi_visit_solution,
        "best_multi_visit_fitness": best_multi_visit_fitness,
        "output_file": output_file,
    }


def main():
    instances = [r"test_data\data_new\1.dat"]
    drone_capacities = [4, 8]
    drone_limit_times = [60, 90, 120]
    run_ids = [1, 2]

    param_grid = list(itertools.product(instances, drone_capacities, drone_limit_times, run_ids))
    print(f"Total runs: {len(param_grid)}")

    with Pool(min(12, len(param_grid))) as pool:
        results = pool.map(run_instance, param_grid)

    results = sorted(results, key=lambda r: (r["instance"], r["A"], r["L_d"], r["run_id"]))
    for row in results:
        print(
            f"{row['instance']} | A={row['A']} | L_d={row['L_d']} | run={row['run_id']} | "
            f"exit={row['exit_code']} | time={row['runtime_sec']:.2f}s | best={row['best_fitness']}"
        )

    df = pd.DataFrame(results)
    output_excel = "batch_results_data_new_1.2.xlsx"
    df.to_excel(output_excel, index=False)
    print(f"Excel saved: {output_excel}")


if __name__ == "__main__":
    main()
