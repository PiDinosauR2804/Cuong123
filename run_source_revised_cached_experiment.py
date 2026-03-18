from __future__ import annotations

import os
import time
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor, as_completed

import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Source_revised.Function import ProblemData, read_dat
from Source_revised.Solution import evaluate_solution
from Source_revised.main import AtsParams, adaptive_tabu_search


def _extract_trip_customers(solution: Any) -> List[List[int]]:
    if not isinstance(solution, list) or len(solution) < 2:
        return []
    trips = solution[1]
    if not isinstance(trips, list):
        return []

    all_trips: List[List[int]] = []
    for trip in trips:
        if not isinstance(trip, list):
            continue
        customers: List[int] = []
        for leg in trip:
            if not isinstance(leg, list) or len(leg) < 2:
                continue
            delivered = leg[1]
            if isinstance(delivered, list):
                for c in delivered:
                    if isinstance(c, int):
                        customers.append(c)
        all_trips.append(customers)
    return all_trips


def _extract_trip_launches(solution: Any) -> List[List[int]]:
    if not isinstance(solution, list) or len(solution) < 2:
        return []
    trips = solution[1]
    if not isinstance(trips, list):
        return []

    launch_trips: List[List[int]] = []
    for trip in trips:
        if not isinstance(trip, list):
            continue
        launches: List[int] = []
        for leg in trip:
            if not isinstance(leg, list) or len(leg) < 1:
                continue
            launch_city = leg[0]
            if isinstance(launch_city, int):
                launches.append(launch_city)
        launch_trips.append(launches)
    return launch_trips


def _calc_single_trip_distance(data: ProblemData, launch_points: List[int]) -> float:
    if not launch_points:
        return 0.0
    distance = data.drone_euclid_distance[0][launch_points[0]]
    for i in range(len(launch_points) - 1):
        distance += data.drone_euclid_distance[launch_points[i]][launch_points[i + 1]]
    distance += data.drone_euclid_distance[launch_points[-1]][0]
    return float(distance)


def calc_drone_trip_distances(data: ProblemData, solution: Any) -> Tuple[List[float], Optional[float]]:
    launch_trips = _extract_trip_launches(solution)
    if not launch_trips:
        return [], None

    distances = [_calc_single_trip_distance(data, points) for points in launch_trips]
    if not distances:
        return [], None
    return distances, (sum(distances) / len(distances))


def calc_drone_trip_stats(data: ProblemData, solution: Any) -> Tuple[Optional[float], Optional[float]]:
    trips = _extract_trip_customers(solution)
    if not trips:
        return None, None

    total_customers = 0
    total_demand = 0.0
    for customers in trips:
        total_customers += len(customers)
        for c in customers:
            if 0 <= c < len(data.demands):
                total_demand += data.demands[c]

    trip_count = len(trips)
    if trip_count == 0:
        return None, None

    return total_customers / trip_count, total_demand / trip_count


def count_multi_visit_trips(solution: Any) -> int:
    if not isinstance(solution, list) or len(solution) < 2:
        return 0
    trips = solution[1]
    if not isinstance(trips, list):
        return 0
    return sum(1 for trip in trips if isinstance(trip, list) and len(trip) > 1)


def _wait_stats_from_eval(ev) -> Tuple[Dict[int, float], Dict[int, float], Optional[float], Optional[float]]:
    truck_wait = ev.truck_wait_by_city or {}
    drone_wait = ev.drone_wait_by_city or {}

    avg_truck = None
    avg_drone = None
    if truck_wait:
        avg_truck = sum(truck_wait.values()) / len(truck_wait)
    if drone_wait:
        avg_drone = sum(drone_wait.values()) / len(drone_wait)
    return truck_wait, drone_wait, avg_truck, avg_drone


def _row_template(instance: str, drone_count: int, truck_count: int, drone_capacity: int, drone_limit_time: int) -> Dict[str, Any]:
    return {
        "instance": instance,
        "drone_count": drone_count,
        "truck_count": truck_count,
        "drone_capacity": drone_capacity,
        "drone_limit_time": drone_limit_time,
        "best_fitness": None,
        "best_sol": None,
        "best_sol_multi_visit_trip_count": None,
        "best_multi_visit_fitness": None,
        "best_multi_visit_sol": None,
        "best_multi_visit_sol_multi_visit_trip_count": None,
        "best_sol_avg_customers_per_drone_trip": None,
        "best_sol_avg_demand_per_drone_trip": None,
        "best_sol_drone_trip_distances": None,
        "best_sol_avg_distance_per_drone_trip": None,
        "best_sol_truck_wait_by_point": None,
        "best_sol_drone_wait_by_point": None,
        "best_sol_avg_truck_wait_by_point": None,
        "best_sol_avg_drone_wait_by_point": None,
        "best_multi_visit_sol_avg_customers_per_drone_trip": None,
        "best_multi_visit_sol_avg_demand_per_drone_trip": None,
        "best_multi_visit_sol_drone_trip_distances": None,
        "best_multi_visit_sol_avg_distance_per_drone_trip": None,
        "best_multi_visit_sol_truck_wait_by_point": None,
        "best_multi_visit_sol_drone_wait_by_point": None,
        "best_multi_visit_sol_avg_truck_wait_by_point": None,
        "best_multi_visit_sol_avg_drone_wait_by_point": None,
        "runtime_sec": None,
        "status": "INIT",
    }


def run_one_task(task: Tuple[str, int, int, int, int, int, int, int, int]) -> Dict[str, Any]:
    (
        instance,
        run_id,
        drone_count,
        truck_count,
        drone_capacity,
        drone_limit_time,
        nimp,
        seg,
        div,
    ) = task

    t0 = time.time()
    row = _row_template(instance, drone_count, truck_count, drone_capacity, drone_limit_time)
    row["run"] = run_id

    if not os.path.exists(instance):
        row["status"] = "DATA_NOT_FOUND"
        row["runtime_sec"] = round(time.time() - t0, 3)
        return row

    try:
        data = read_dat(instance)
        data.number_drone = int(drone_count)
        data.number_truck = int(truck_count)
        data.drone_capacity = float(drone_capacity)
        data.drone_limit_time = float(drone_limit_time)

        params = AtsParams(
            nimp=nimp,
            seg=seg,
            div=div,
            seed=42 + run_id,
            use_eval_cache=True,
        )

        best_sol, best_fit = adaptive_tabu_search(instance, params)
        ev = evaluate_solution(best_sol, data)

        best_trip_count = count_multi_visit_trips(best_sol)
        best_avg_cust, best_avg_demand = calc_drone_trip_stats(data, best_sol)
        best_dists, best_avg_dist = calc_drone_trip_distances(data, best_sol)
        best_truck_wait, best_drone_wait, best_avg_truck_wait, best_avg_drone_wait = _wait_stats_from_eval(ev)

        # Source_revised ATS does not track a separate best-multi-visit solution.
        # Use best_sol when it already contains multi-visit trips; otherwise keep empty fields.
        if best_trip_count > 0:
            mv_sol = best_sol
            mv_fit = best_fit
            mv_trip_count = best_trip_count
            mv_avg_cust, mv_avg_demand = best_avg_cust, best_avg_demand
            mv_dists, mv_avg_dist = best_dists, best_avg_dist
            mv_truck_wait, mv_drone_wait = best_truck_wait, best_drone_wait
            mv_avg_truck_wait, mv_avg_drone_wait = best_avg_truck_wait, best_avg_drone_wait
        else:
            mv_sol = None
            mv_fit = None
            mv_trip_count = None
            mv_avg_cust, mv_avg_demand = None, None
            mv_dists, mv_avg_dist = [], None
            mv_truck_wait, mv_drone_wait = {}, {}
            mv_avg_truck_wait, mv_avg_drone_wait = None, None

        row.update(
            {
                "best_fitness": best_fit,
                "best_sol": str(best_sol),
                "best_sol_multi_visit_trip_count": best_trip_count,
                "best_multi_visit_fitness": mv_fit,
                "best_multi_visit_sol": str(mv_sol),
                "best_multi_visit_sol_multi_visit_trip_count": mv_trip_count,
                "best_sol_avg_customers_per_drone_trip": None if best_avg_cust is None else round(best_avg_cust, 6),
                "best_sol_avg_demand_per_drone_trip": None if best_avg_demand is None else round(best_avg_demand, 6),
                "best_sol_drone_trip_distances": str([round(v, 6) for v in best_dists]),
                "best_sol_avg_distance_per_drone_trip": None if best_avg_dist is None else round(best_avg_dist, 6),
                "best_sol_truck_wait_by_point": str({k: round(v, 6) for k, v in best_truck_wait.items()}),
                "best_sol_drone_wait_by_point": str({k: round(v, 6) for k, v in best_drone_wait.items()}),
                "best_sol_avg_truck_wait_by_point": None if best_avg_truck_wait is None else round(best_avg_truck_wait, 6),
                "best_sol_avg_drone_wait_by_point": None if best_avg_drone_wait is None else round(best_avg_drone_wait, 6),
                "best_multi_visit_sol_avg_customers_per_drone_trip": None if mv_avg_cust is None else round(mv_avg_cust, 6),
                "best_multi_visit_sol_avg_demand_per_drone_trip": None if mv_avg_demand is None else round(mv_avg_demand, 6),
                "best_multi_visit_sol_drone_trip_distances": str([round(v, 6) for v in mv_dists]),
                "best_multi_visit_sol_avg_distance_per_drone_trip": None if mv_avg_dist is None else round(mv_avg_dist, 6),
                "best_multi_visit_sol_truck_wait_by_point": str({k: round(v, 6) for k, v in mv_truck_wait.items()}),
                "best_multi_visit_sol_drone_wait_by_point": str({k: round(v, 6) for k, v in mv_drone_wait.items()}),
                "best_multi_visit_sol_avg_truck_wait_by_point": None if mv_avg_truck_wait is None else round(mv_avg_truck_wait, 6),
                "best_multi_visit_sol_avg_drone_wait_by_point": None if mv_avg_drone_wait is None else round(mv_avg_drone_wait, 6),
                "status": "OK" if ev.feasible else "INFEASIBLE",
                "runtime_sec": round(time.time() - t0, 3),
            }
        )

    except Exception as exc:  # noqa: BLE001
        row["status"] = f"ERROR: {type(exc).__name__}: {exc}"
        row["runtime_sec"] = round(time.time() - t0, 3)

    return row


def main() -> None:
    instance = r"test_data\data_demand_random_50_batch_all1_equal_cluster\C101_0.5.dat"
    drone_count = 2
    truck_count = 2
    drone_capacity = 4
    drone_limit_times = [60, 90, 120]
    runs_per_config = 2

    nimp = 30
    seg = 4
    div = 3

    workers = 12

    tasks: List[Tuple[str, int, int, int, int, int, int, int, int]] = []
    for drone_limit_time in drone_limit_times:
        for run_id in range(1, runs_per_config + 1):
            tasks.append(
                (
                    instance,
                    run_id,
                    drone_count,
                    truck_count,
                    drone_capacity,
                    drone_limit_time,
                    nimp,
                    seg,
                    div,
                )
            )

    rows: List[Dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(run_one_task, task) for task in tasks]
        for future in as_completed(futures):
            rows.append(future.result())

    rows.sort(key=lambda r: (r["instance"], r["drone_limit_time"], r.get("run", 0)))

    # Keep exactly the requested output columns.
    output_columns = [
        "instance",
        "drone_count",
        "truck_count",
        "drone_capacity",
        "drone_limit_time",
        "best_fitness",
        "best_sol",
        "best_sol_multi_visit_trip_count",
        "best_multi_visit_fitness",
        "best_multi_visit_sol",
        "best_multi_visit_sol_multi_visit_trip_count",
        "best_sol_avg_customers_per_drone_trip",
        "best_sol_avg_demand_per_drone_trip",
        "best_sol_drone_trip_distances",
        "best_sol_avg_distance_per_drone_trip",
        "best_sol_truck_wait_by_point",
        "best_sol_drone_wait_by_point",
        "best_sol_avg_truck_wait_by_point",
        "best_sol_avg_drone_wait_by_point",
        "best_multi_visit_sol_avg_customers_per_drone_trip",
        "best_multi_visit_sol_avg_demand_per_drone_trip",
        "best_multi_visit_sol_drone_trip_distances",
        "best_multi_visit_sol_avg_distance_per_drone_trip",
        "best_multi_visit_sol_truck_wait_by_point",
        "best_multi_visit_sol_drone_wait_by_point",
        "best_multi_visit_sol_avg_truck_wait_by_point",
        "best_multi_visit_sol_avg_drone_wait_by_point",
        "runtime_sec",
        "status",
    ]

    df = pd.DataFrame(rows)
    df = df[output_columns]

    out_dir = Path("Result") / "excel_result"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_xlsx = out_dir / f"source_revised_cached_C101_0.5_seg4_div3_{int(time.time())}.xlsx"

    df.to_excel(out_xlsx, index=False)

    ok_rows = int((df["status"] == "OK").sum())
    print(f"output_excel={out_xlsx}")
    print(f"total_rows={len(df)}")
    print(f"ok_rows={ok_rows}")
    print("workers=6")
    print("seg=4 div=3 cache=True")


if __name__ == "__main__":
    main()
