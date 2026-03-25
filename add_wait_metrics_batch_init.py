from __future__ import annotations

import argparse
import ast
import csv
import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import heapq

from Source_revised2 import Solution, read_data_file, validate_solution
from Source_revised2.problem_data import ProblemData
from Source_revised2.solution import TruckRoute, TruckStop


EPS = 1e-9
PROJECT_ROOT = Path(__file__).resolve().parent


@dataclass
class WaitStats:
    feasible: bool
    violations: List[str]
    trip_truck_wait: Dict[int, float]
    trip_drone_wait: Dict[int, float]



def _parse_solution(raw: Any) -> Optional[Solution]:
    text = str(raw or "").strip()
    if not text or text.lower() in {"none", "nan", "null"}:
        return None
    for parser in (json.loads, ast.literal_eval):
        try:
            parsed = parser(text)
            return Solution.from_legacy(parsed)
        except Exception:
            continue
    return None



def _resolve_instance_path(instance_raw: str) -> Path:
    normalized = str(instance_raw or "").strip().replace("\\", "/")
    p = Path(normalized)
    if p.is_absolute():
        return p.resolve()
    return (PROJECT_ROOT / normalized).resolve()



def _max_release(data: ProblemData, customers: List[int]) -> float:
    vals: List[float] = []
    for customer in customers:
        if 0 <= customer < data.number_of_cities:
            vals.append(data.release_dates[customer])
    return max(vals) if vals else 0.0



def _normalize_route(route: TruckRoute) -> TruckRoute:
    copied = TruckRoute(stops=[TruckStop(city=s.city, drone_customers=list(s.drone_customers)) for s in route.stops])
    if not copied.stops or copied.stops[0].city != 0:
        copied.stops.insert(0, TruckStop(city=0, drone_customers=[]))
    if copied.stops[-1].city != 0:
        copied.stops.append(TruckStop(city=0, drone_customers=[]))
    return copied



def _compute_wait_stats(solution: Solution, data: ProblemData) -> WaitStats:
    validation = validate_solution(solution, data)
    if not validation.feasible:
        return WaitStats(False, list(validation.violations), {}, {})

    violations: List[str] = []
    normalized_routes = [_normalize_route(route) for route in solution.truck_routes]
    if not normalized_routes:
        return WaitStats(False, ["No truck route found"], {}, {})

    route_cities: List[List[int]] = []
    pending: List[List[List[int]]] = []
    city_owner: Dict[int, int] = {}
    city_pos_by_truck: List[Dict[int, int]] = []

    for t_idx, route in enumerate(normalized_routes):
        cities = [stop.city for stop in route.stops]
        route_cities.append(cities)
        pending.append([list(stop.drone_customers) for stop in route.stops])

        pos_map: Dict[int, int] = {}
        for pos, city in enumerate(cities):
            if city < 0 or city >= data.number_of_cities:
                violations.append(f"Truck {t_idx} has out-of-range city {city}")
                continue
            if pos == 0 or pos == len(cities) - 1:
                if city != 0:
                    violations.append(f"Truck {t_idx} must start/end at depot 0")
                continue
            if city == 0:
                violations.append(f"Truck {t_idx} has depot inside route at position {pos}")
                continue
            if city not in pos_map:
                pos_map[city] = pos
            previous_owner = city_owner.get(city)
            if previous_owner is not None and previous_owner != t_idx:
                violations.append(
                    f"Customer city {city} appears in multiple truck routes: {previous_owner} and {t_idx}"
                )
            city_owner[city] = t_idx
        city_pos_by_truck.append(pos_map)

    truck_idx = [0 for _ in normalized_routes]
    truck_time = [0.0 for _ in normalized_routes]

    for t_idx in range(len(normalized_routes)):
        depot_packages = pending[t_idx][0]
        depart = _max_release(data, depot_packages)
        pending[t_idx][0] = []
        truck_time[t_idx] = depart

        if len(route_cities[t_idx]) >= 2:
            next_city = route_cities[t_idx][1]
            if next_city != 0:
                truck_time[t_idx] += data.truck_time_matrix[0][next_city]
            truck_idx[t_idx] = 1

    def advance_truck_through_empty(t_idx: int) -> None:
        while True:
            idx = truck_idx[t_idx]
            if idx >= len(route_cities[t_idx]) - 1:
                return
            current_city = route_cities[t_idx][idx]
            if current_city == 0:
                return
            if pending[t_idx][idx]:
                return
            next_city = route_cities[t_idx][idx + 1]
            truck_time[t_idx] += data.truck_time_matrix[current_city][next_city]
            truck_idx[t_idx] += 1

    def project_arrival_to_city(t_idx: int, target_pos: int) -> Tuple[Optional[float], Optional[int]]:
        if target_pos < truck_idx[t_idx]:
            return None, route_cities[t_idx][truck_idx[t_idx]]

        sim_idx = truck_idx[t_idx]
        sim_time = truck_time[t_idx]
        while sim_idx < target_pos:
            if sim_idx >= len(route_cities[t_idx]) - 1:
                return None, None
            current_city = route_cities[t_idx][sim_idx]
            if current_city == 0:
                return None, None
            if pending[t_idx][sim_idx]:
                return None, current_city
            next_city = route_cities[t_idx][sim_idx + 1]
            sim_time += data.truck_time_matrix[current_city][next_city]
            sim_idx += 1
        return sim_time, None

    def move_truck_to_city(t_idx: int, target_pos: int, trip_idx: int, leg_idx: int) -> bool:
        if target_pos < truck_idx[t_idx]:
            violations.append(
                f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} already passed launch city position {target_pos}"
            )
            return False

        while truck_idx[t_idx] < target_pos:
            if truck_idx[t_idx] >= len(route_cities[t_idx]) - 1:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} cannot reach target position {target_pos}"
                )
                return False
            current_city = route_cities[t_idx][truck_idx[t_idx]]
            if current_city == 0:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} reached depot before target position {target_pos}"
                )
                return False
            if pending[t_idx][truck_idx[t_idx]]:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} blocked at city {current_city} "
                    f"with undelivered package(s) {pending[t_idx][truck_idx[t_idx]]}"
                )
                return False
            next_city = route_cities[t_idx][truck_idx[t_idx] + 1]
            truck_time[t_idx] += data.truck_time_matrix[current_city][next_city]
            truck_idx[t_idx] += 1
        return True

    if data.number_drone <= 0 and solution.drone_queue:
        violations.append("number_drone <= 0 but drone_queue is not empty")
    drone_heap: List[Tuple[float, int]] = [(0.0, i) for i in range(max(data.number_drone, 0))]
    heapq.heapify(drone_heap)

    trip_truck_wait: Dict[int, float] = {}
    trip_drone_wait: Dict[int, float] = {}

    for trip_idx, trip in enumerate(solution.drone_queue):
        for t in range(len(normalized_routes)):
            advance_truck_through_empty(t)

        if not trip.legs:
            trip_truck_wait[trip_idx] = 0.0
            trip_drone_wait[trip_idx] = 0.0
            continue

        if not drone_heap:
            violations.append(f"Trip {trip_idx}: no drone available")
            trip_truck_wait[trip_idx] = float("inf")
            trip_drone_wait[trip_idx] = float("inf")
            continue

        first_leg = trip.legs[0]
        first_owner = city_owner.get(first_leg.launch_city)
        first_target_pos = None
        truck_arrive_first = None

        if first_owner is None:
            violations.append(
                f"Trip {trip_idx}: first launch city {first_leg.launch_city} is not in any truck route"
            )
        else:
            first_target_pos = city_pos_by_truck[first_owner].get(first_leg.launch_city)
            if first_target_pos is None:
                violations.append(
                    f"Trip {trip_idx}: cannot locate first launch city {first_leg.launch_city} on truck {first_owner}"
                )
            else:
                truck_arrive_first, blocking_city = project_arrival_to_city(first_owner, first_target_pos)
                if truck_arrive_first is None:
                    if blocking_city is not None:
                        violations.append(
                            f"Trip {trip_idx}: truck {first_owner} cannot reach first launch city "
                            f"{first_leg.launch_city} because it is blocked at city {blocking_city}"
                        )
                    else:
                        violations.append(
                            f"Trip {trip_idx}: truck {first_owner} cannot reach first launch city {first_leg.launch_city}"
                        )

        all_customers: List[int] = []
        for leg in trip.legs:
            all_customers.extend(leg.customers)
        release_bound = _max_release(data, all_customers)

        drone_ready_time, drone_id = heapq.heappop(drone_heap)
        if truck_arrive_first is None:
            depart_time = max(drone_ready_time, release_bound)
        else:
            leg0_flight = data.drone_time_matrix[0][first_leg.launch_city]
            depart_time = max(drone_ready_time, release_bound, truck_arrive_first - leg0_flight)

        drone_clock = depart_time
        previous_city = 0
        truck_wait_sum = 0.0
        drone_wait_sum = 0.0

        for leg_idx, leg in enumerate(trip.legs):
            launch = leg.launch_city
            owner = city_owner.get(launch)
            if owner is None:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: launch city {launch} is not in any truck route"
                )
                continue

            target_pos = city_pos_by_truck[owner].get(launch)
            if target_pos is None:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: cannot locate launch city {launch} on truck {owner}"
                )
                continue

            if not move_truck_to_city(owner, target_pos, trip_idx, leg_idx):
                continue

            fly_time = data.drone_time_matrix[previous_city][launch]
            drone_arrival = drone_clock + fly_time

            truck_arrival = truck_time[owner]
            sync_time = max(drone_arrival, truck_arrival)

            drone_wait = max(0.0, sync_time - drone_arrival)
            truck_wait = max(0.0, sync_time - truck_arrival)
            drone_wait_sum += drone_wait
            truck_wait_sum += truck_wait

            service_end = sync_time + data.unloading_time
            truck_time[owner] = service_end
            drone_clock = service_end

            for customer in leg.customers:
                if customer < 0 or customer >= data.number_of_cities:
                    violations.append(f"Trip {trip_idx}, leg {leg_idx}: customer {customer} out of range")
                    continue

                removed = False
                if customer in pending[owner][target_pos]:
                    pending[owner][target_pos].remove(customer)
                    removed = True
                else:
                    for s in range(target_pos, len(pending[owner])):
                        if customer in pending[owner][s]:
                            pending[owner][s].remove(customer)
                            removed = True
                            break
                if not removed:
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx}: customer {customer} not found in pending packages "
                        f"of truck {owner}"
                    )

            previous_city = launch
            advance_truck_through_empty(owner)

        back_time = data.drone_time_matrix[previous_city][0]
        drone_clock += back_time
        heapq.heappush(drone_heap, (drone_clock, drone_id))

        trip_truck_wait[trip_idx] = truck_wait_sum
        trip_drone_wait[trip_idx] = drone_wait_sum

    feasible = len(violations) == 0
    return WaitStats(feasible=feasible, violations=violations, trip_truck_wait=trip_truck_wait, trip_drone_wait=trip_drone_wait)



def _avg_wait_per_trip(wait_by_trip: Dict[int, float], trip_count: int) -> float:
    if trip_count <= 0:
        return 0.0
    total = 0.0
    for i in range(trip_count):
        total += float(wait_by_trip.get(i, 0.0))
    return total / trip_count



def _fmt_float(x: Optional[float]) -> str:
    if x is None:
        return ""
    s = f"{float(x):.9f}".rstrip("0").rstrip(".")
    return s if s else "0"



def _compute_for_solution(sol_text: Any, data: ProblemData) -> Tuple[str, str]:
    sol = _parse_solution(sol_text)
    if sol is None:
        return "", ""

    ws = _compute_wait_stats(sol, data)
    if not ws.feasible:
        return "", ""

    trip_count = len(sol.drone_queue)
    avg_truck = _avg_wait_per_trip(ws.trip_truck_wait, trip_count)
    avg_drone = _avg_wait_per_trip(ws.trip_drone_wait, trip_count)
    return _fmt_float(avg_truck), _fmt_float(avg_drone)



def add_wait_columns(input_csv: Path, output_csv: Optional[Path] = None, backup: bool = True) -> Tuple[int, int]:
    input_csv = input_csv.resolve()
    if not input_csv.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    rows: List[Dict[str, str]] = []
    with input_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    new_cols = [
        "best_avg_truck_wait_per_trip",
        "best_avg_drone_wait_per_trip",
        "best_multi_avg_truck_wait_per_trip",
        "best_multi_avg_drone_wait_per_trip",
    ]
    for c in new_cols:
        if c not in fieldnames:
            fieldnames.append(c)

    cache: Dict[Tuple[str, str, str], ProblemData] = {}
    ok = 0

    for row in rows:
        instance = str(row.get("instance", "")).strip()
        a_txt = str(row.get("A", "") or "").strip()
        l_txt = str(row.get("L", "") or "").strip()

        key = (instance, a_txt, l_txt)
        data = cache.get(key)
        if data is None:
            p = _resolve_instance_path(instance)
            data = read_data_file(p)
            data.drone_capacity = float(a_txt or 0)
            data.drone_limit_time = float(l_txt or 0)
            cache[key] = data

        b_t, b_d = _compute_for_solution(row.get("best_solution", ""), data)
        m_t, m_d = _compute_for_solution(row.get("best_multi_solution", ""), data)

        row["best_avg_truck_wait_per_trip"] = b_t
        row["best_avg_drone_wait_per_trip"] = b_d
        row["best_multi_avg_truck_wait_per_trip"] = m_t
        row["best_multi_avg_drone_wait_per_trip"] = m_d
        ok += 1

    out_path = output_csv.resolve() if output_csv else input_csv
    if backup and out_path == input_csv:
        backup_path = input_csv.with_suffix(input_csv.suffix + ".bak_before_wait_cols")
        shutil.copy2(input_csv, backup_path)

    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})

    return len(rows), ok



def main() -> None:
    parser = argparse.ArgumentParser(description="Add average truck/drone wait per drone trip to batch_init.csv")
    parser.add_argument("--input-csv", type=Path, default=PROJECT_ROOT / "batch_init.csv")
    parser.add_argument("--output-csv", type=Path, default=None)
    parser.add_argument("--no-backup", action="store_true")
    args = parser.parse_args()

    total, ok = add_wait_columns(
        input_csv=args.input_csv,
        output_csv=args.output_csv,
        backup=not args.no_backup,
    )
    print(f"rows_total={total}")
    print(f"rows_processed={ok}")
    print(f"output_csv={(args.output_csv.resolve() if args.output_csv else args.input_csv.resolve())}")


if __name__ == "__main__":
    main()
