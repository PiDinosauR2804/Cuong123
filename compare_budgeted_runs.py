from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
DEFAULT_INSTANCES = [
    ROOT / "test_data" / "data_demand_random_50_batch_all1_equal_cluster" / "C101_0.5.dat",
    ROOT / "test_data" / "data_demand_random_50_batch_all1_equal_cluster" / "C101_1.dat",
    ROOT / "test_data" / "data_demand_random_50_batch_all1_equal_random" / "C101_0.5.dat",
    ROOT / "test_data" / "data_demand_random_50_batch_all1_equal_random" / "C101_1.dat",
]
DEFAULT_CSV = ROOT / "Result" / "budget_compare_10min.csv"
DEFAULT_JSON = ROOT / "Result" / "budget_compare_10min.json"


SOURCE_REVISED2_SNIPPET = r"""
import json, sys, time
from Source_revised2 import read_data_file
from Source_revised2.ats import AtsParams, adaptive_tabu_search

instance_path = sys.argv[1]
source_profile = sys.argv[2]
data = read_data_file(instance_path)

if source_profile == "practical":
    params = AtsParams(
        seed=42,
        nimp=5,
        seg=2,
        div=1,
        truck_max_neighbors=20,
        drone_max_neighbors=20,
        use_drone_refine=True,
    )
elif source_profile == "light":
    params = AtsParams(
        seed=42,
        nimp=2,
        seg=1,
        div=0,
        truck_max_neighbors=10,
        drone_max_neighbors=10,
        use_drone_refine=True,
    )
else:
    params = AtsParams(seed=42)

start = time.time()
result = adaptive_tabu_search(
    data=data,
    params=params,
    verbose=False,
)
elapsed = time.time() - start
payload = {
    "status": "ok",
    "objective": result.best_eval.objective,
    "initial_objective": result.initial_eval.objective,
    "segments_run": result.segments_run,
    "diversification_rounds": result.diversification_rounds,
    "truck_routes": result.best_solution.to_legacy()[0],
    "drone_queue": result.best_solution.to_legacy()[1],
    "drone_trip_count": len(result.best_solution.to_legacy()[1]),
    "elapsed_seconds": elapsed,
}
print(json.dumps(payload, ensure_ascii=False))
"""


TEST_SIMILARITY_SNIPPET = r"""
import io, json, random, sys, time
from contextlib import redirect_stdout

import numpy as np

import Data
import test_similarity

instance_path = sys.argv[1]
budget_seconds = int(sys.argv[2])

random.seed(42)
np.random.seed(42)
test_similarity.TIME_LIMIT = budget_seconds
Data.read_data_random(instance_path)

buf = io.StringIO()
start = time.time()
with redirect_stdout(buf):
    best_fitness, best_sol, data = test_similarity.Tabu_search_for_CVRP(1)
elapsed = time.time() - start

best_multi = data.get("best_multi_visit_fitness")
best_multi_sol = data.get("best_multi_visit_sol")
drone_trip_count = 0
if isinstance(best_sol, list) and len(best_sol) > 1 and isinstance(best_sol[1], list):
    drone_trip_count = len(best_sol[1])

payload = {
    "status": "ok",
    "objective": best_fitness,
    "best_multi_visit_fitness": best_multi,
    "solution": best_sol,
    "best_multi_visit_solution": best_multi_sol,
    "drone_trip_count": drone_trip_count,
    "elapsed_seconds": elapsed,
}
print(json.dumps(payload, ensure_ascii=False))
"""


@dataclass
class JobResult:
    solver: str
    instance: str
    status: str
    objective: Any = None
    elapsed_seconds: float | None = None
    drone_trip_count: int | None = None
    initial_objective: Any = None
    segments_run: int | None = None
    diversification_rounds: int | None = None
    best_multi_visit_fitness: Any = None
    solution: Any = None
    extra: dict[str, Any] | None = None
    stderr: str = ""


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Source_revised2 and test_similarity with the same time budget."
    )
    parser.add_argument(
        "--budget-seconds",
        type=int,
        default=600,
        help="Wall-clock budget per solver/instance in seconds.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
        help="How many solver jobs to run in parallel.",
    )
    parser.add_argument(
        "--csv-out",
        type=Path,
        default=DEFAULT_CSV,
        help="Where to write the comparison CSV.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=DEFAULT_JSON,
        help="Where to write the comparison JSON.",
    )
    parser.add_argument(
        "--instances",
        nargs="*",
        type=Path,
        default=DEFAULT_INSTANCES,
        help="Instance files to benchmark.",
    )
    parser.add_argument(
        "--source-profile",
        choices=["default", "practical", "light"],
        default="practical",
        help="Parameter preset for Source_revised2.",
    )
    return parser.parse_args()


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def _run_subprocess(
    cmd: list[str],
    *,
    budget_seconds: int,
    solver: str,
    instance: Path,
) -> JobResult:
    start = time.time()
    try:
        proc = subprocess.run(
            cmd,
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=budget_seconds + 30,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return JobResult(
            solver=solver,
            instance=str(instance),
            status="timeout",
            elapsed_seconds=time.time() - start,
        )

    elapsed = time.time() - start
    stdout = proc.stdout.strip()
    stderr = proc.stderr.strip()

    if proc.returncode != 0:
        return JobResult(
            solver=solver,
            instance=str(instance),
            status="error",
            elapsed_seconds=elapsed,
            stderr=stderr or stdout,
        )

    try:
        payload = json.loads(stdout.splitlines()[-1])
    except Exception:
        return JobResult(
            solver=solver,
            instance=str(instance),
            status="error",
            elapsed_seconds=elapsed,
            stderr=f"Could not parse JSON output. stderr={stderr!r} stdout_tail={stdout[-500:]!r}",
        )

    return JobResult(
        solver=solver,
        instance=str(instance),
        status=payload.get("status", "ok"),
        objective=payload.get("objective"),
        elapsed_seconds=payload.get("elapsed_seconds", elapsed),
        drone_trip_count=payload.get("drone_trip_count"),
        initial_objective=payload.get("initial_objective"),
        segments_run=payload.get("segments_run"),
        diversification_rounds=payload.get("diversification_rounds"),
        best_multi_visit_fitness=payload.get("best_multi_visit_fitness"),
        solution=payload.get("solution")
        if "solution" in payload
        else {
            "truck_routes": payload.get("truck_routes"),
            "drone_queue": payload.get("drone_queue"),
        },
        extra=payload,
        stderr=stderr,
    )


def _run_source_revised2(instance: Path, budget_seconds: int, source_profile: str) -> JobResult:
    cmd = ["python3", "-c", SOURCE_REVISED2_SNIPPET, str(instance), source_profile]
    return _run_subprocess(
        cmd,
        budget_seconds=budget_seconds,
        solver="Source_revised2",
        instance=instance,
    )


def _run_test_similarity(instance: Path, budget_seconds: int) -> JobResult:
    cmd = ["python", "-c", TEST_SIMILARITY_SNIPPET, str(instance), str(budget_seconds)]
    return _run_subprocess(
        cmd,
        budget_seconds=budget_seconds,
        solver="test_similarity",
        instance=instance,
    )


def _result_to_row(result: JobResult) -> dict[str, Any]:
    return {
        "solver": result.solver,
        "instance": result.instance,
        "status": result.status,
        "objective": result.objective,
        "initial_objective": result.initial_objective,
        "best_multi_visit_fitness": result.best_multi_visit_fitness,
        "elapsed_seconds": result.elapsed_seconds,
        "drone_trip_count": result.drone_trip_count,
        "segments_run": result.segments_run,
        "diversification_rounds": result.diversification_rounds,
        "stderr": result.stderr,
    }


def _print_summary(rows: list[JobResult]) -> None:
    grouped: dict[str, list[JobResult]] = {}
    for row in rows:
        grouped.setdefault(row.instance, []).append(row)

    print(f"{'Instance':<90} {'Solver':<18} {'Status':<10} {'Objective':<12} {'Time(s)':<10} {'DroneTrips':<10}")
    print("-" * 160)
    for instance in sorted(grouped):
        for result in sorted(grouped[instance], key=lambda x: x.solver):
            print(
                f"{instance:<90} "
                f"{result.solver:<18} "
                f"{result.status:<10} "
                f"{str(result.objective):<12} "
                f"{str(round(result.elapsed_seconds or 0.0, 2)):<10} "
                f"{str(result.drone_trip_count):<10}"
            )


def main() -> None:
    args = _parse_args()
    instances = [path.resolve() for path in args.instances]
    jobs = []
    for instance in instances:
        jobs.append(("Source_revised2", instance))
        jobs.append(("test_similarity", instance))

    results: list[JobResult] = []
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as executor:
        future_map = {}
        for solver, instance in jobs:
            if solver == "Source_revised2":
                future = executor.submit(
                    _run_source_revised2,
                    instance,
                    args.budget_seconds,
                    args.source_profile,
                )
            else:
                future = executor.submit(_run_test_similarity, instance, args.budget_seconds)
            future_map[future] = (solver, instance)

        for future in as_completed(future_map):
            result = future.result()
            results.append(result)
            print(
                f"[done] {result.solver} | {Path(result.instance).name} | "
                f"status={result.status} | objective={result.objective} | "
                f"time={round(result.elapsed_seconds or 0.0, 2)}s"
            )

    _ensure_parent(args.csv_out)
    _ensure_parent(args.json_out)

    with args.csv_out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "solver",
                "instance",
                "status",
                "objective",
                "initial_objective",
                "best_multi_visit_fitness",
                "elapsed_seconds",
                "drone_trip_count",
                "segments_run",
                "diversification_rounds",
                "stderr",
            ],
        )
        writer.writeheader()
        for result in sorted(results, key=lambda x: (x.instance, x.solver)):
            writer.writerow(_result_to_row(result))

    json_payload = [
        {
            **_result_to_row(result),
            "solution": result.solution,
            "extra": result.extra,
        }
        for result in sorted(results, key=lambda x: (x.instance, x.solver))
    ]
    args.json_out.write_text(
        json.dumps(json_payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    _print_summary(results)
    print(f"\nCSV saved to: {args.csv_out}")
    print(f"JSON saved to: {args.json_out}")


if __name__ == "__main__":
    main()
