from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import copy

from .Function import ProblemData
from .Solution import DroneLeg, SimulationResult, TruckStop, evaluate_solution, normalize_solution
from .move_truck_only import move_1_0, move_1_1, move_2_1, two_opt, is_within_drone_range


@dataclass
class TruckDroneNeighbor:
    neighborhood: str
    moved_customers: List[int]
    solution: Any
    fitness: float
    truck_completion_sum: float


def _evaluate_cached(
    raw_solution: Any,
    data: ProblemData,
    eval_cache: Optional[Dict[str, SimulationResult]],
) -> SimulationResult:
    if eval_cache is None:
        return evaluate_solution(raw_solution, data)
    key = repr(raw_solution)
    cached = eval_cache.get(key)
    if cached is not None:
        return cached
    ev = evaluate_solution(raw_solution, data)
    eval_cache[key] = ev
    return ev


def _raw_from_parsed(trucks: List[List[TruckStop]], trips: List[List[DroneLeg]]) -> Any:
    truck_raw: List[List[List[Any]]] = []
    for route in trucks:
        truck_raw.append([[s.city, s.drone_customers[:]] for s in route])
    trip_raw: List[List[List[Any]]] = []
    for trip in trips:
        trip_raw.append([[leg.launch_city, leg.customers[:]] for leg in trip])
    return [truck_raw, trip_raw]


def _routes_from_trucks(trucks: List[List[TruckStop]]) -> List[List[int]]:
    return [[stop.city for stop in route if stop.city != 0] for route in trucks]


def _owner_and_pos(routes: List[List[int]]) -> Tuple[Dict[int, int], Dict[int, int]]:
    owner: Dict[int, int] = {}
    pos: Dict[int, int] = {}
    for t_idx, route in enumerate(routes):
        for p, c in enumerate(route):
            owner[c] = t_idx
            pos[c] = p
    return owner, pos


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


def _customer_drop_assignment(trucks: List[List[TruckStop]], customer: int) -> Optional[Tuple[int, int]]:
    for t_idx, route in enumerate(trucks):
        for s_idx, stop in enumerate(route):
            if customer in stop.drone_customers:
                return t_idx, s_idx
    return None


def _invalid_customers(trucks: List[List[TruckStop]], routes: List[List[int]], data: ProblemData) -> List[int]:
    owner, pos = _owner_and_pos(routes)
    invalid: List[int] = []
    for c in owner.keys():
        assign = _customer_drop_assignment(trucks, c)
        if assign is None:
            invalid.append(c)
            continue
        t_idx, s_idx = assign
        if t_idx != owner[c]:
            invalid.append(c)
            continue
        drop_city = trucks[t_idx][s_idx].city
        if drop_city != 0:
            drop_pos = s_idx - 1
            if drop_pos > pos[c]:
                invalid.append(c)
                continue
            if not is_within_drone_range(data, drop_city):
                invalid.append(c)
                continue
    return sorted(set(invalid), key=lambda c: (owner[c], pos[c]))


def _trip_total_demand(data: ProblemData, trip: List[DroneLeg]) -> float:
    return sum(sum(data.demands[c] for c in leg.customers) for leg in trip)


def _launch_city_used_in_any_trip(trips: List[List[DroneLeg]], launch_city: int) -> bool:
    for trip in trips:
        for leg in trip:
            if leg.launch_city == launch_city:
                return True
    return False


def _try_direct_truck_assignment(
    trucks: List[List[TruckStop]],
    trips: List[List[DroneLeg]],
    owner: int,
    customer: int,
    data: ProblemData,
    eval_cache: Optional[Dict[str, SimulationResult]],
) -> Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]]]]:
    cand_trucks = copy.deepcopy(trucks)
    cand_trips = copy.deepcopy(trips)
    _remove_customer_everywhere(cand_trucks, cand_trips, customer)

    depot_loaded = cand_trucks[owner][0].drone_customers[:]
    # Paper hierarchy: direct truck assignment is allowed when release(customer)
    # does not exceed the max release among packages currently loaded at depot.
    max_release = max((data.release_dates[c] for c in depot_loaded), default=float("inf"))
    if data.release_dates[customer] > max_release + 1e-9:
        return None

    if customer not in cand_trucks[owner][0].drone_customers:
        cand_trucks[owner][0].drone_customers.append(customer)
    _cleanup_trips(cand_trips)

    ev = _evaluate_cached(_raw_from_parsed(cand_trucks, cand_trips), data, eval_cache)
    if not ev.feasible:
        return None
    return cand_trucks, cand_trips


def _append_to_existing_sync(
    trucks: List[List[TruckStop]],
    trips: List[List[DroneLeg]],
    owner: int,
    stop_idx: int,
    customer: int,
    data: ProblemData,
    eval_cache: Optional[Dict[str, SimulationResult]],
) -> Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]]]]:
    launch_city = trucks[owner][stop_idx].city
    demand = data.demands[customer]
    if launch_city == 0:
        return None
    if not is_within_drone_range(data, launch_city):
        return None

    best: Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]], float, float]] = None
    for tr_idx, trip in enumerate(trips):
        for leg_idx, leg in enumerate(trip):
            if leg.launch_city != launch_city:
                continue
            leg_demand = sum(data.demands[c] for c in leg.customers)
            if leg_demand + demand > data.drone_capacity + 1e-9:
                continue
            if _trip_total_demand(data, trip) + demand > data.drone_capacity + 1e-9:
                continue

            cand_trucks = copy.deepcopy(trucks)
            cand_trips = copy.deepcopy(trips)
            _remove_customer_everywhere(cand_trucks, cand_trips, customer)
            cand_trucks[owner][stop_idx].drone_customers.append(customer)
            cand_trips[tr_idx][leg_idx].customers.append(customer)
            _cleanup_trips(cand_trips)

            ev = _evaluate_cached(_raw_from_parsed(cand_trucks, cand_trips), data, eval_cache)
            if not ev.feasible:
                continue
            fit = ev.system_completion_time
            truck_sum = sum(ev.truck_time.values())
            if best is None or (fit, truck_sum) < (best[2], best[3]):
                best = (cand_trucks, cand_trips, fit, truck_sum)

    if best is None:
        return None
    return best[0], best[1]


def _create_new_trip(
    trucks: List[List[TruckStop]],
    trips: List[List[DroneLeg]],
    owner: int,
    stop_idx: int,
    customer: int,
    data: ProblemData,
    eval_cache: Optional[Dict[str, SimulationResult]],
) -> Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]]]]:
    launch_city = trucks[owner][stop_idx].city
    if launch_city == 0:
        return None
    if data.demands[customer] > data.drone_capacity + 1e-9:
        return None
    if not is_within_drone_range(data, launch_city):
        return None

    cand_trucks = copy.deepcopy(trucks)
    cand_trips = copy.deepcopy(trips)
    _remove_customer_everywhere(cand_trucks, cand_trips, customer)
    cand_trucks[owner][stop_idx].drone_customers.append(customer)
    cand_trips.append([DroneLeg(launch_city=launch_city, customers=[customer])])
    _cleanup_trips(cand_trips)

    ev = _evaluate_cached(_raw_from_parsed(cand_trucks, cand_trips), data, eval_cache)
    if not ev.feasible:
        return None
    return cand_trucks, cand_trips


def _depot_cascade_by_release(
    trucks: List[List[TruckStop]],
    trips: List[List[DroneLeg]],
    routes: List[List[int]],
    owner: int,
    customer: int,
    data: ProblemData,
    eval_cache: Optional[Dict[str, SimulationResult]],
) -> Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]]]]:
    threshold = data.release_dates[customer]
    cand_trucks = copy.deepcopy(trucks)
    cand_trips = copy.deepcopy(trips)

    # As requested: when customer is taken at depot, remove from drone trips
    # all same-truck packages with smaller release dates.
    same_truck_earlier_release = [c for c in routes[owner] if data.release_dates[c] < threshold]
    affected = set(same_truck_earlier_release + [customer])
    for c in affected:
        _remove_customer_everywhere(cand_trucks, cand_trips, c)
        if c not in cand_trucks[owner][0].drone_customers:
            cand_trucks[owner][0].drone_customers.append(c)

    _cleanup_trips(cand_trips)
    ev = _evaluate_cached(_raw_from_parsed(cand_trucks, cand_trips), data, eval_cache)
    if not ev.feasible:
        return None
    return cand_trucks, cand_trips


def _best_reuse_existing_sync(
    trucks: List[List[TruckStop]],
    trips: List[List[DroneLeg]],
    owner: int,
    customer_pos: int,
    customer: int,
    data: ProblemData,
    eval_cache: Optional[Dict[str, SimulationResult]],
) -> Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]]]]:
    best: Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]], float, float]] = None

    for s_idx in range(1, customer_pos + 2):
        reuse = _append_to_existing_sync(trucks, trips, owner, s_idx, customer, data, eval_cache)
        for cand in [reuse]:
            if cand is None:
                continue
            ev = _evaluate_cached(_raw_from_parsed(cand[0], cand[1]), data, eval_cache)
            if not ev.feasible:
                continue
            fit = ev.system_completion_time
            truck_sum = sum(ev.truck_time.values())
            if best is None or (fit, truck_sum) < (best[2], best[3]):
                best = (cand[0], cand[1], fit, truck_sum)

    if best is None:
        return None
    return best[0], best[1]


def _best_new_trip(
    trucks: List[List[TruckStop]],
    trips: List[List[DroneLeg]],
    owner: int,
    customer_pos: int,
    customer: int,
    data: ProblemData,
    eval_cache: Optional[Dict[str, SimulationResult]],
) -> Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]]]]:
    best: Optional[Tuple[List[List[TruckStop]], List[List[DroneLeg]], float, float]] = None

    # Prefer creating a new trip from an already-used launch city first.
    # If that is not feasible, allow any feasible launch city.
    for require_used_launch in [True, False]:
        for s_idx in range(customer_pos + 1, 0, -1):
            launch_city = trucks[owner][s_idx].city
            used = _launch_city_used_in_any_trip(trips, launch_city)
            if require_used_launch and not used:
                continue
            if (not require_used_launch) and used:
                continue

            cand = _create_new_trip(trucks, trips, owner, s_idx, customer, data, eval_cache)
            if cand is None:
                continue
            ev = _evaluate_cached(_raw_from_parsed(cand[0], cand[1]), data, eval_cache)
            if not ev.feasible:
                continue
            fit = ev.system_completion_time
            truck_sum = sum(ev.truck_time.values())
            if best is None or (fit, truck_sum) < (best[2], best[3]):
                best = (cand[0], cand[1], fit, truck_sum)
        if best is not None:
            break

    if best is None:
        return None
    return best[0], best[1]


def _build_candidate_structure(
    base_trucks: List[List[TruckStop]],
    base_trips: List[List[DroneLeg]],
    moved_routes: List[List[int]],
) -> Tuple[List[List[TruckStop]], List[List[DroneLeg]]]:
    owner, _ = _owner_and_pos(moved_routes)

    old_by_city: Dict[int, List[int]] = {}
    old_depot_by_truck: Dict[int, List[int]] = {}
    for t_idx, route in enumerate(base_trucks):
        old_depot_by_truck[t_idx] = route[0].drone_customers[:]
        for stop in route[1:]:
            old_by_city[stop.city] = stop.drone_customers[:]

    new_trucks: List[List[TruckStop]] = []
    for t_idx, route in enumerate(moved_routes):
        depot = [c for c in old_depot_by_truck.get(t_idx, []) if owner.get(c) == t_idx]
        new_route = [TruckStop(city=0, drone_customers=depot)]
        for city in route:
            customers = [c for c in old_by_city.get(city, []) if owner.get(c) == t_idx]
            new_route.append(TruckStop(city=city, drone_customers=customers))
        new_trucks.append(new_route)

    new_trips = copy.deepcopy(base_trips)
    _cleanup_trips(new_trips)
    return new_trucks, new_trips


def _deterministic_repair(
    trucks: List[List[TruckStop]],
    trips: List[List[DroneLeg]],
    routes: List[List[int]],
    data: ProblemData,
    preferred_customers: Optional[List[int]] = None,
    eval_cache: Optional[Dict[str, SimulationResult]] = None,
) -> Tuple[List[List[TruckStop]], List[List[DroneLeg]]]:
    owner, pos = _owner_and_pos(routes)
    invalid = _invalid_customers(trucks, routes, data)
    if preferred_customers:
        pref = [c for c in preferred_customers if c in invalid]
        tail = [c for c in invalid if c not in pref]
        invalid = pref + tail

    cur_trucks = copy.deepcopy(trucks)
    cur_trips = copy.deepcopy(trips)

    for customer in invalid:
        t_idx = owner[customer]
        c_pos = pos[customer]

        # Hierarchical repair order from the paper:
        # 1) direct truck assignment at depot, 2) reuse existing sync point,
        # 3) create a new drone trip.
        direct = _try_direct_truck_assignment(cur_trucks, cur_trips, t_idx, customer, data, eval_cache)
        if direct is not None:
            cur_trucks, cur_trips = direct
            continue

        reuse = _best_reuse_existing_sync(cur_trucks, cur_trips, t_idx, c_pos, customer, data, eval_cache)
        if reuse is not None:
            cur_trucks, cur_trips = reuse
            continue

        created = _best_new_trip(cur_trucks, cur_trips, t_idx, c_pos, customer, data, eval_cache)
        if created is not None:
            cur_trucks, cur_trips = created
            continue

        cascaded = _depot_cascade_by_release(
            cur_trucks,
            cur_trips,
            routes,
            t_idx,
            customer,
            data,
            eval_cache,
        )
        if cascaded is not None:
            cur_trucks, cur_trips = cascaded

    # Final safety fallback: all packages loaded at depot for each truck.
    final_ev = _evaluate_cached(_raw_from_parsed(cur_trucks, cur_trips), data, eval_cache)
    if final_ev.feasible:
        return cur_trucks, cur_trips

    fallback_trucks = copy.deepcopy(cur_trucks)
    fallback_trips: List[List[DroneLeg]] = []
    for t_idx, route in enumerate(routes):
        fallback_trucks[t_idx][0].drone_customers = route[:]
        for s_idx in range(1, len(fallback_trucks[t_idx])):
            fallback_trucks[t_idx][s_idx].drone_customers = []
    return fallback_trucks, fallback_trips


def generate_truck_drone_neighbors(
    raw_solution: Any,
    data: ProblemData,
    use_eval_cache: bool = True,
    neighborhood_filter: Optional[List[str]] = None,
) -> List[TruckDroneNeighbor]:
    base_trucks, base_trips = normalize_solution(raw_solution)
    base_routes = _routes_from_trucks(base_trucks)
    eval_cache: Optional[Dict[str, SimulationResult]] = {} if use_eval_cache else None

    neighborhood_builders = [
        ("(1,0)", move_1_0),
        ("(1,1)", move_1_1),
        ("2-opt", two_opt),
        ("(2,1)", move_2_1),
    ]
    allowed = set(neighborhood_filter) if neighborhood_filter else None

    out: List[TruckDroneNeighbor] = []
    for name, builder in neighborhood_builders:
        if allowed is not None and name not in allowed:
            continue
        for moved_routes, moved_customers in builder(base_routes):
            cand_trucks, cand_trips = _build_candidate_structure(base_trucks, base_trips, moved_routes)
            rep_trucks, rep_trips = _deterministic_repair(
                cand_trucks,
                cand_trips,
                moved_routes,
                data,
                preferred_customers=moved_customers,
                eval_cache=eval_cache,
            )
            cand_raw = _raw_from_parsed(rep_trucks, rep_trips)
            ev = _evaluate_cached(cand_raw, data, eval_cache)
            if not ev.feasible:
                continue
            out.append(
                TruckDroneNeighbor(
                    neighborhood=name,
                    moved_customers=moved_customers,
                    solution=cand_raw,
                    fitness=ev.system_completion_time,
                    truck_completion_sum=sum(ev.truck_time.values()),
                )
            )
    return out


def best_truck_drone_neighbor(raw_solution: Any, data: ProblemData) -> Optional[TruckDroneNeighbor]:
    neighbors = generate_truck_drone_neighbors(raw_solution, data, use_eval_cache=True)
    if not neighbors:
        return None
    return min(neighbors, key=lambda n: (n.fitness, n.truck_completion_sum))


def repair_solution_from_truck_routes(
    raw_solution: Any,
    moved_routes: List[List[int]],
    data: ProblemData,
    preferred_customers: Optional[List[int]] = None,
    use_eval_cache: bool = True,
) -> Optional[Any]:
    """Repair drone assignments after externally modified truck routes.

    Returns a feasible solution when repair succeeds; otherwise None.
    """
    base_trucks, base_trips = normalize_solution(raw_solution)
    eval_cache: Optional[Dict[str, SimulationResult]] = {} if use_eval_cache else None
    cand_trucks, cand_trips = _build_candidate_structure(base_trucks, base_trips, moved_routes)
    rep_trucks, rep_trips = _deterministic_repair(
        cand_trucks,
        cand_trips,
        moved_routes,
        data,
        preferred_customers=preferred_customers,
        eval_cache=eval_cache,
    )
    repaired = _raw_from_parsed(rep_trucks, rep_trips)
    ev = _evaluate_cached(repaired, data, eval_cache)
    if not ev.feasible:
        return None
    return repaired
