from __future__ import annotations

from typing import Any, List, Optional, Tuple
import copy

from .Function import ProblemData
from .Solution import DroneLeg, TruckStop, evaluate_solution, normalize_solution


def _sum_demand(data: ProblemData, customers: List[int]) -> float:
    return sum(data.demands[c] for c in customers)


def _raw_from_parsed(trucks: List[List[TruckStop]], trips: List[List[DroneLeg]]) -> Any:
    truck_raw: List[List[List[Any]]] = []
    for route in trucks:
        truck_raw.append([[s.city, s.drone_customers[:]] for s in route])
    trip_raw: List[List[List[Any]]] = []
    for trip in trips:
        trip_raw.append([[leg.launch_city, leg.customers[:]] for leg in trip])
    return [truck_raw, trip_raw]


def _remove_customer_everywhere(trucks: List[List[TruckStop]], trips: List[List[DroneLeg]], customer: int) -> None:
    for route in trucks:
        for stop in route:
            stop.drone_customers = [c for c in stop.drone_customers if c != customer]
    for trip in trips:
        for leg in trip:
            leg.customers = [c for c in leg.customers if c != customer]


def _cleanup_trips(trips: List[List[DroneLeg]]) -> None:
    i = 0
    while i < len(trips):
        trips[i] = [leg for leg in trips[i] if leg.customers]
        if not trips[i]:
            trips.pop(i)
        else:
            i += 1


def _candidate_metrics(raw_solution: Any, data: ProblemData) -> Tuple[float, float]:
    result = evaluate_solution(raw_solution, data)
    if not result.feasible:
        return float("inf"), float("inf")
    return result.system_completion_time, sum(result.truck_time.values())


def _release_gap_limit(data: ProblemData) -> float:
    releases = data.release_dates[1:]
    if not releases:
        return 0.0
    return (sum(releases) / len(releases)) / 2.0


def _release_compatible(data: ProblemData, customer: int, leg_customers: List[int], max_gap: float) -> bool:
    base_release = data.release_dates[customer]
    for c in leg_customers:
        if abs(base_release - data.release_dates[c]) > max_gap + 1e-9:
            return False
    return True


def best_sync_point_relocation(raw_solution: Any, data: ProblemData, base_fit: float) -> Optional[Any]:
    trucks, trips = normalize_solution(raw_solution)
    best_sol: Optional[Any] = None
    best_fit = base_fit
    best_truck_sum = float("inf")

    for i, trip in enumerate(trips):
        for j, _ in enumerate(trip):
            for k, target_trip in enumerate(trips):
                if i == k:
                    continue
                for pos in range(len(target_trip) + 1):
                    cand_trucks = copy.deepcopy(trucks)
                    cand_trips = copy.deepcopy(trips)
                    moving = cand_trips[i].pop(j)
                    if not cand_trips[i]:
                        cand_trips.pop(i)
                        k2 = k - 1 if i < k else k
                    else:
                        k2 = k
                    cand_trips[k2].insert(pos, moving)
                    cand_raw = _raw_from_parsed(cand_trucks, cand_trips)
                    f, truck_sum = _candidate_metrics(cand_raw, data)
                    if f < best_fit or (abs(f - best_fit) <= 1e-9 and truck_sum < best_truck_sum):
                        best_fit = f
                        best_truck_sum = truck_sum
                        best_sol = cand_raw
    return best_sol


def best_package_relocation(raw_solution: Any, data: ProblemData, base_fit: float) -> Optional[Any]:
    trucks, trips = normalize_solution(raw_solution)
    best_sol: Optional[Any] = None
    best_fit = base_fit
    best_truck_sum = float("inf")
    max_release_gap = _release_gap_limit(data)

    for t_idx, route in enumerate(trucks):
        for s_idx, stop in enumerate(route):
            if not stop.drone_customers:
                continue
            for customer in stop.drone_customers[:]:
                demand = data.demands[customer]
                for target_idx in range(s_idx + 1, len(route)):
                    launch_city = route[target_idx].city
                    if launch_city == 0:
                        continue
                    if 2.0 * data.drone_travel_time[0][launch_city] + data.unloading_time > data.drone_limit_time + 1e-9:
                        continue

                    for tr_idx, trip in enumerate(trips):
                        for leg_idx, leg in enumerate(trip):
                            if leg.launch_city != launch_city:
                                continue
                            if not _release_compatible(data, customer, leg.customers, max_release_gap):
                                continue
                            leg_demand = _sum_demand(data, leg.customers)
                            if leg_demand + demand > data.drone_capacity + 1e-9:
                                continue
                            cand_trucks = copy.deepcopy(trucks)
                            cand_trips = copy.deepcopy(trips)
                            _remove_customer_everywhere(cand_trucks, cand_trips, customer)
                            cand_trucks[t_idx][target_idx].drone_customers.append(customer)
                            cand_trips[tr_idx][leg_idx].customers.append(customer)
                            _cleanup_trips(cand_trips)
                            cand_raw = _raw_from_parsed(cand_trucks, cand_trips)
                            f, truck_sum = _candidate_metrics(cand_raw, data)
                            if f < best_fit or (abs(f - best_fit) <= 1e-9 and truck_sum < best_truck_sum):
                                best_fit = f
                                best_truck_sum = truck_sum
                                best_sol = cand_raw

                    if demand <= data.drone_capacity + 1e-9:
                        # If existing legs at this launch city are full, allow creating
                        # a new trip (feasibility is still checked by evaluator).
                        cand_trucks = copy.deepcopy(trucks)
                        cand_trips = copy.deepcopy(trips)
                        _remove_customer_everywhere(cand_trucks, cand_trips, customer)
                        cand_trucks[t_idx][target_idx].drone_customers.append(customer)
                        cand_trips.append([DroneLeg(launch_city=launch_city, customers=[customer])])
                        _cleanup_trips(cand_trips)
                        cand_raw = _raw_from_parsed(cand_trucks, cand_trips)
                        f, truck_sum = _candidate_metrics(cand_raw, data)
                        if f < best_fit or (abs(f - best_fit) <= 1e-9 and truck_sum < best_truck_sum):
                            best_fit = f
                            best_truck_sum = truck_sum
                            best_sol = cand_raw

    return best_sol


def best_drone_trip_reordering(raw_solution: Any, data: ProblemData, base_fit: float) -> Optional[Any]:
    trucks, trips = normalize_solution(raw_solution)
    best_sol: Optional[Any] = None
    best_fit = base_fit
    best_truck_sum = float("inf")

    if len(trips) <= 1:
        return None

    for i in range(len(trips)):
        for j in range(len(trips) + 1):
            if j == i or j == i + 1:
                continue
            cand_trucks = copy.deepcopy(trucks)
            cand_trips = copy.deepcopy(trips)
            trip = cand_trips.pop(i)
            insert_pos = j - 1 if i < j else j
            cand_trips.insert(insert_pos, trip)
            cand_raw = _raw_from_parsed(cand_trucks, cand_trips)
            f, truck_sum = _candidate_metrics(cand_raw, data)
            if f < best_fit or (abs(f - best_fit) <= 1e-9 and truck_sum < best_truck_sum):
                best_fit = f
                best_truck_sum = truck_sum
                best_sol = cand_raw
    return best_sol


def best_merge_two_trips_multi_visit(raw_solution: Any, data: ProblemData) -> Optional[Any]:
    """Try merging two drone trips into one multi-leg trip.

    The candidate is accepted only if feasible; selection follows the same
    objective pair (system completion time, truck completion sum).
    """
    trucks, trips = normalize_solution(raw_solution)
    if len(trips) < 2:
        return None

    best_sol: Optional[Any] = None
    best_fit = float("inf")
    best_truck_sum = float("inf")

    for i in range(len(trips)):
        for j in range(i + 1, len(trips)):
            if not trips[i] or not trips[j]:
                continue
            for forward in (True, False):
                cand_trips = copy.deepcopy(trips)
                first = i
                second = j
                merged = cand_trips[first] + cand_trips[second] if forward else cand_trips[second] + cand_trips[first]
                cand_trips[first] = merged
                cand_trips.pop(second)

                cand_raw = _raw_from_parsed(copy.deepcopy(trucks), cand_trips)
                f, truck_sum = _candidate_metrics(cand_raw, data)
                if f < best_fit or (abs(f - best_fit) <= 1e-9 and truck_sum < best_truck_sum):
                    best_fit = f
                    best_truck_sum = truck_sum
                    best_sol = cand_raw

    return best_sol
