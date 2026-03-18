from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple
import copy
import random

from .Function import ProblemData
from .Solution import DroneLeg, TruckStop, evaluate_solution, fitness, normalize_solution
from .move_truck_only import is_within_drone_range, tabu_search_truck_only, phase1_makespan
from .drone_move import best_sync_point_relocation, best_package_relocation, best_drone_trip_reordering


def _all_customers(data: ProblemData) -> List[int]:
    return list(range(1, data.number_of_cities))


def _build_truck_only_solution(routes: List[List[int]]) -> Any:
    trucks_raw: List[List[List[Any]]] = []
    for route in routes:
        truck_route: List[List[Any]] = [[0, route[:]]]
        for c in route:
            truck_route.append([c, []])
        trucks_raw.append(truck_route)
    return [trucks_raw, []]


def build_initial_solution_phase1(data: ProblemData) -> Any:
    customers = sorted(_all_customers(data), key=lambda c: data.release_dates[c])
    k = max(1, data.number_truck)

    routes: List[List[int]] = [[] for _ in range(k)]
    for i, c in enumerate(customers[:k]):
        routes[i].append(c)

    for c in customers[k:]:
        best_cost = float("inf")
        best_routes: Optional[List[List[int]]] = None
        for r_idx in range(k):
            for pos in range(len(routes[r_idx]) + 1):
                cand = [r[:] for r in routes]
                cand[r_idx].insert(pos, c)
                cost = phase1_makespan(cand, data)
                if cost < best_cost:
                    best_cost = cost
                    best_routes = cand
        if best_routes is not None:
            routes = best_routes

    return _build_truck_only_solution(routes)


def _truck_routes_from_raw(raw_solution: Any) -> List[List[int]]:
    trucks, _ = normalize_solution(raw_solution)
    routes: List[List[int]] = []
    for r in trucks:
        routes.append([stop.city for stop in r if stop.city != 0])
    return routes


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


def _trip_total_demand(trip: List[DroneLeg], data: ProblemData) -> float:
    return sum(sum(data.demands[c] for c in leg.customers) for leg in trip)


def _try_assign_customer_at_stop(
    raw_solution: Any,
    owner: int,
    stop_idx: int,
    customer: int,
    data: ProblemData,
    *,
    require_existing_sync: bool,
) -> Optional[Tuple[Any, Any]]:
    trucks, trips = normalize_solution(raw_solution)
    launch_city = trucks[owner][stop_idx].city
    if launch_city == 0:
        return None
    if not is_within_drone_range(data, launch_city):
        return None
    if data.demands[customer] > data.drone_capacity + 1e-9:
        return None

    best_raw: Optional[Any] = None
    best_eval: Optional[Any] = None

    # Prefer reusing an existing synchronization point at the same launch city.
    for tr_idx, trip in enumerate(trips):
        for leg_idx, leg in enumerate(trip):
            if leg.launch_city != launch_city:
                continue
            leg_demand = sum(data.demands[c] for c in leg.customers)
            if leg_demand + data.demands[customer] > data.drone_capacity + 1e-9:
                continue
            if _trip_total_demand(trip, data) + data.demands[customer] > data.drone_capacity + 1e-9:
                continue

            cand_trucks = copy.deepcopy(trucks)
            cand_trips = copy.deepcopy(trips)
            _remove_customer_everywhere(cand_trucks, cand_trips, customer)
            cand_trucks[owner][stop_idx].drone_customers.append(customer)
            cand_trips[tr_idx][leg_idx].customers.append(customer)
            _cleanup_trips(cand_trips)
            cand_raw = _raw_from_parsed(cand_trucks, cand_trips)
            cand_eval = evaluate_solution(cand_raw, data)
            if not cand_eval.feasible:
                continue
            if best_eval is None or (
                cand_eval.system_completion_time,
                sum(cand_eval.truck_time.values()),
            ) < (
                best_eval.system_completion_time,
                sum(best_eval.truck_time.values()),
            ):
                best_raw = cand_raw
                best_eval = cand_eval

    if best_raw is not None and best_eval is not None:
        return best_raw, best_eval

    if require_existing_sync:
        return None

    # Create a new drone trip at this launch city.
    cand_trucks = copy.deepcopy(trucks)
    cand_trips = copy.deepcopy(trips)
    _remove_customer_everywhere(cand_trucks, cand_trips, customer)
    cand_trucks[owner][stop_idx].drone_customers.append(customer)
    cand_trips.append([DroneLeg(launch_city=launch_city, customers=[customer])])
    _cleanup_trips(cand_trips)
    cand_raw = _raw_from_parsed(cand_trucks, cand_trips)
    cand_eval = evaluate_solution(cand_raw, data)
    if not cand_eval.feasible:
        return None
    return cand_raw, cand_eval


def _seed_single_trip_release_desc(raw_solution: Any, data: ProblemData) -> Optional[Any]:
    routes = _truck_routes_from_raw(raw_solution)
    if not routes:
        return None

    # Rebuild a clean truck-only baseline first: all packages at depot, no trips.
    seed_trucks: List[List[TruckStop]] = []
    for route in routes:
        parsed = [TruckStop(city=0, drone_customers=route[:])]
        for c in route:
            parsed.append(TruckStop(city=c, drone_customers=[]))
        seed_trucks.append(parsed)
    current = _raw_from_parsed(seed_trucks, [])
    current_eval = evaluate_solution(current, data)
    if not current_eval.feasible:
        return None

    owner_pos: Dict[int, Tuple[int, int]] = {}
    for t_idx, route in enumerate(routes):
        for pos_idx, city in enumerate(route):
            stop_idx = pos_idx + 1
            owner_pos[city] = (t_idx, stop_idx)
    customers_desc = sorted(owner_pos.keys(), key=lambda c: data.release_dates[c], reverse=True)

    for idx, customer in enumerate(customers_desc):
        if data.demands[customer] > data.drone_capacity + 1e-9:
            continue
        t_idx, c_stop_idx = owner_pos[customer]
        trucks_now, trips_now = normalize_solution(current)

        # Candidate A for the first processed package:
        # try customer stop first, then earlier stops, keep only improving moves.
        if idx == 0:
            best_move: Optional[Tuple[Any, Any]] = None
            for s_idx in range(c_stop_idx, 0, -1):
                cand = _try_assign_customer_at_stop(
                    current,
                    t_idx,
                    s_idx,
                    customer,
                    data,
                    require_existing_sync=False,
                )
                if cand is None:
                    continue
                cand_raw, cand_eval = cand
                if cand_eval.system_completion_time + 1e-9 >= current_eval.system_completion_time:
                    continue
                if best_move is None or (
                    cand_eval.system_completion_time,
                    sum(cand_eval.truck_time.values()),
                ) < (
                    best_move[1].system_completion_time,
                    sum(best_move[1].truck_time.values()),
                ):
                    best_move = (cand_raw, cand_eval)
            if best_move is not None:
                current, current_eval = best_move
            continue

        # Candidate B for remaining packages:
        # 1) try previous synchronization positions (same truck) with improvement.
        prev_sync_positions = [
            s_idx
            for s_idx in range(1, c_stop_idx + 1)
            if trucks_now[t_idx][s_idx].city != 0 and len(trucks_now[t_idx][s_idx].drone_customers) > 0
        ]
        best_reuse: Optional[Tuple[Any, Any]] = None
        for s_idx in prev_sync_positions:
            cand = _try_assign_customer_at_stop(
                current,
                t_idx,
                s_idx,
                customer,
                data,
                require_existing_sync=True,
            )
            if cand is None:
                continue
            cand_raw, cand_eval = cand
            if cand_eval.system_completion_time + 1e-9 >= current_eval.system_completion_time:
                continue
            if best_reuse is None or (
                cand_eval.system_completion_time,
                sum(cand_eval.truck_time.values()),
            ) < (
                best_reuse[1].system_completion_time,
                sum(best_reuse[1].truck_time.values()),
            ):
                best_reuse = (cand_raw, cand_eval)
        if best_reuse is not None:
            current, current_eval = best_reuse
            continue

        # 2) fallback: create/attach at customer stop or any earlier feasible stop.
        #    Accept the best feasible move (even without immediate improvement).
        best_fallback: Optional[Tuple[Any, Any]] = None
        for s_idx in range(c_stop_idx, 0, -1):
            cand = _try_assign_customer_at_stop(
                current,
                t_idx,
                s_idx,
                customer,
                data,
                require_existing_sync=False,
            )
            if cand is None:
                continue
            cand_raw, cand_eval = cand
            if best_fallback is None or (
                cand_eval.system_completion_time,
                sum(cand_eval.truck_time.values()),
            ) < (
                best_fallback[1].system_completion_time,
                sum(best_fallback[1].truck_time.values()),
            ):
                best_fallback = (cand_raw, cand_eval)
        if best_fallback is not None:
            current, current_eval = best_fallback

    return current


def tabu_search_phase1(raw_solution: Any, data: ProblemData) -> Any:
    routes = _truck_routes_from_raw(raw_solution)
    best_routes = tabu_search_truck_only(routes, data)
    return _build_truck_only_solution(best_routes)


def local_search_phase2(raw_solution: Any, data: ProblemData) -> Any:
    base_solution = copy.deepcopy(raw_solution)
    base_eval = evaluate_solution(base_solution, data)
    safe_solution = None
    if not base_eval.feasible:
        # Hard fallback: rebuild truck-only structure from current truck routes.
        fallback = _build_truck_only_solution(_truck_routes_from_raw(raw_solution))
        fallback_eval = evaluate_solution(fallback, data)
        if fallback_eval.feasible:
            base_solution = fallback
            base_eval = fallback_eval
            safe_solution = copy.deepcopy(fallback)
        else:
            # Last-resort fallback from data construction.
            fallback2 = build_initial_solution_phase1(data)
            fallback2_eval = evaluate_solution(fallback2, data)
            if fallback2_eval.feasible:
                base_solution = fallback2
                base_eval = fallback2_eval
                safe_solution = copy.deepcopy(fallback2)
            else:
                return fallback2
    else:
        safe_solution = copy.deepcopy(base_solution)

    seeded = _seed_single_trip_release_desc(raw_solution, data)
    current = copy.deepcopy(seeded if seeded is not None else base_solution)
    current_eval = evaluate_solution(current, data)
    if not current_eval.feasible:
        current = base_solution
        current_eval = base_eval
    current_fit = current_eval.system_completion_time
    current_truck_sum = sum(current_eval.truck_time.values())

    while True:
        c1 = best_sync_point_relocation(current, data, current_fit)
        c2 = best_package_relocation(current, data, current_fit)
        c3 = best_drone_trip_reordering(current, data, current_fit)

        raw_candidates = [c for c in [c1, c2, c3] if c is not None]
        if not raw_candidates:
            break

        # Feasible-only policy: ignore all infeasible candidates in search.
        evaluated = []
        for cand in raw_candidates:
            ev = evaluate_solution(cand, data)
            if ev.feasible:
                evaluated.append((cand, ev))

        if not evaluated:
            break

        best_next, best_eval = min(
            evaluated,
            key=lambda item: (
                item[1].system_completion_time,
                sum(item[1].truck_time.values()),
            ),
        )

        best_next_fit = best_eval.system_completion_time
        best_next_truck_sum = sum(best_eval.truck_time.values())

        # Accept strict fitness improvement first. If fitness does not improve,
        # still accept when total truck completion time improves.
        accept = False
        if best_next_fit + 1e-9 < current_fit:
            accept = True
        elif best_next_truck_sum + 1e-9 < current_truck_sum:
            accept = True
        if not accept:
            break

        current = best_next
        current_fit = best_next_fit
        current_truck_sum = best_next_truck_sum

    final_eval = evaluate_solution(current, data)
    if final_eval.feasible:
        return current
    return copy.deepcopy(safe_solution if safe_solution is not None else base_solution)


def build_two_phase_solution(data: ProblemData, seed: Optional[int] = None) -> Any:
    if seed is not None:
        random.seed(seed)

    init_sol = build_initial_solution_phase1(data)
    phase1_sol = tabu_search_phase1(init_sol, data)
    phase2_sol = local_search_phase2(phase1_sol, data)
    return phase2_sol
