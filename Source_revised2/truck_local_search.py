from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple
import copy
import random

from .fitness import evaluate_fitness
from .problem_data import ProblemData
from .solution import DroneLeg, DroneTrip, Solution, TruckRoute, TruckStop
from .validator import validate_solution


EPS = 1e-9


@dataclass
class TruckNeighbor:
    neighborhood: str
    moved_customers: List[int]
    solution: Solution
    objective: float
    truck_completion_sum: float
    move_detail: str


def _clone_solution(solution: Solution) -> Solution:
    return Solution.from_legacy(solution.to_legacy())


def _to_solution(solution: Solution | Any) -> Solution:
    if isinstance(solution, Solution):
        return solution
    return Solution.from_legacy(solution)


def _routes_from_solution(solution: Solution) -> List[List[int]]:
    routes: List[List[int]] = []
    for route in solution.truck_routes:
        seq = [stop.city for stop in route.stops if stop.city != 0]
        routes.append(seq)
    return routes


def _is_within_drone_range(data: ProblemData, city: int) -> bool:
    if city == 0:
        return True
    total = 2.0 * data.drone_time_matrix[0][city] + data.unloading_time
    return total <= data.drone_limit_time + EPS


def move_1_0_routes(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int], str]]:
    out: List[Tuple[List[List[int]], List[int], str]] = []
    for i in range(len(routes)):
        for j in range(len(routes[i])):
            customer = routes[i][j]
            for k in range(len(routes)):
                for l in range(len(routes[k]) + 1):
                    if i == k and (l == j or l == j + 1):
                        continue

                    cand = [r[:] for r in routes]
                    cand[i].pop(j)
                    insert_at = l
                    if i == k and l > j:
                        insert_at -= 1
                    cand[k].insert(insert_at, customer)

                    detail = f"({i},{j})->({k},{l})"
                    out.append((cand, [customer], detail))
    return out


def move_1_1_routes(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int], str]]:
    out: List[Tuple[List[List[int]], List[int], str]] = []
    for i in range(len(routes)):
        for j in range(len(routes[i])):
            for k in range(i, len(routes)):
                start_l = (j + 1) if i == k else 0
                for l in range(start_l, len(routes[k])):
                    customer_a = routes[i][j]
                    customer_b = routes[k][l]
                    if customer_a == customer_b:
                        continue

                    cand = [r[:] for r in routes]
                    cand[i][j], cand[k][l] = cand[k][l], cand[i][j]

                    detail = f"swap({i},{j})<->({k},{l})"
                    out.append((cand, [customer_a, customer_b], detail))
    return out


def move_2_0_routes(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int], str]]:
    out: List[Tuple[List[List[int]], List[int], str]] = []
    for i in range(len(routes)):
        if len(routes[i]) < 2:
            continue
        for j in range(len(routes[i]) - 1):
            block = routes[i][j : j + 2]
            for k in range(len(routes)):
                for l in range(len(routes[k]) + 1):
                    # Same-route no-op insertions: before first, between pair, after pair.
                    if i == k and l in (j, j + 1, j + 2):
                        continue

                    cand = [r[:] for r in routes]
                    moved_block = cand[i][j : j + 2]
                    del cand[i][j : j + 2]

                    insert_at = l
                    if i == k and l > j + 1:
                        insert_at -= 2
                    cand[k][insert_at:insert_at] = moved_block

                    detail = f"block2({i},{j}:{j+1})->({k},{l})"
                    out.append((cand, block[:], detail))
    return out


def move_2_1_routes(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int], str]]:
    out: List[Tuple[List[List[int]], List[int], str]] = []
    for i in range(len(routes)):
        if len(routes[i]) < 2:
            continue
        for j in range(len(routes[i]) - 1):
            block = routes[i][j : j + 2]

            for k in range(len(routes)):
                for l in range(len(routes[k])):
                    # Single city cannot be one of the moved block positions on same route.
                    if i == k and l in (j, j + 1):
                        continue

                    cand = [r[:] for r in routes]
                    customer = routes[k][l]

                    if i == k:
                        route = routes[i]
                        new_route: List[int] = []
                        for idx, city in enumerate(route):
                            if idx == l:
                                new_route.extend(block)
                            elif idx == j:
                                new_route.append(customer)
                            elif idx == j + 1:
                                continue
                            else:
                                new_route.append(city)
                        cand[i] = new_route
                    else:
                        moved_block = cand[i][j : j + 2]
                        del cand[i][j : j + 2]

                        moved_city = cand[k][l]
                        del cand[k][l]

                        cand[i][j:j] = [moved_city]
                        cand[k][l:l] = moved_block

                    detail = f"exchange2-1(({i},{j}:{j+1})<->({k},{l}))"
                    out.append((cand, [block[0], block[1], customer], detail))
    return out


def move_2_opt_routes(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int], str]]:
    out: List[Tuple[List[List[int]], List[int], str]] = []
    # Intra-route 2-opt: reverse a segment within one truck route.
    for i in range(len(routes)):
        n = len(routes[i])
        if n < 2:
            continue
        for a in range(n - 1):
            for b in range(a + 1, n):
                cand = [r[:] for r in routes]
                seg = cand[i][a : b + 1]
                cand[i][a : b + 1] = list(reversed(seg))
                detail = f"2opt_rev({i},{a}:{b})"
                out.append((cand, seg[:], detail))

    # Inter-route 2-opt* : swap tails of two truck routes.
    for i in range(len(routes)):
        for k in range(i + 1, len(routes)):
            if not routes[i] or not routes[k]:
                continue
            for a in range(len(routes[i])):
                for b in range(len(routes[k])):
                    tail_i = routes[i][a + 1 :]
                    tail_k = routes[k][b + 1 :]
                    if not tail_i and not tail_k:
                        continue

                    cand = [r[:] for r in routes]
                    cand[i] = routes[i][: a + 1] + tail_k
                    cand[k] = routes[k][: b + 1] + tail_i
                    moved = tail_i + tail_k
                    detail = f"2opt_star(({i},{a})<->({k},{b}))"
                    out.append((cand, moved, detail))
    return out


def move_swap_two_segments_routes(
    routes: List[List[int]],
    rng: Optional[random.Random] = None,
    max_moves_per_route: Optional[int] = None,
) -> List[Tuple[List[List[int]], List[int], str]]:
    """
    Diversification move inspired by Neighborhood.swap_two_array in test_similarity.py:
    swap 2 segments on the same truck route, one in the first half and one in the second half.
    """
    out: List[Tuple[List[List[int]], List[int], str]] = []
    for t_idx, route in enumerate(routes):
        n = len(route)
        if n < 4:
            continue
        mid = n // 2
        if mid < 2 or mid >= n:
            continue

        min_len_candidates = list(range(2, mid + 1))
        if not min_len_candidates:
            continue
        if rng is None:
            min_len_values = min_len_candidates
        else:
            min_len_values = [rng.choice(min_len_candidates)]

        generated = 0
        stop_route = False
        for min_len in min_len_values:
            for a in range(0, mid):
                b_min = a + min_len - 1
                if b_min >= mid:
                    continue
                for b in range(b_min, mid):
                    for c in range(mid, n):
                        d_min = c + min_len - 1
                        if d_min >= n:
                            continue
                        for d in range(d_min, n):
                            seg1 = route[a : b + 1]
                            seg2 = route[c : d + 1]
                            if not seg1 or not seg2:
                                continue

                            cand = [r[:] for r in routes]
                            cand_route = route[:a] + seg2 + route[b + 1 : c] + seg1 + route[d + 1 :]
                            cand[t_idx] = cand_route

                            moved = seg1 + seg2
                            detail = f"swap2seg({t_idx},{a}:{b}<->{c}:{d})"
                            out.append((cand, moved, detail))
                            generated += 1

                            if max_moves_per_route is not None and generated >= max_moves_per_route:
                                stop_route = True
                                break
                        if stop_route:
                            break
                    if stop_route:
                        break
                if stop_route:
                    break
            if stop_route:
                break
    return out


def _owner_and_pos(routes: List[List[int]]) -> Tuple[Dict[int, int], Dict[int, int]]:
    owner: Dict[int, int] = {}
    pos: Dict[int, int] = {}
    for t_idx, route in enumerate(routes):
        for p, city in enumerate(route):
            owner[city] = t_idx
            pos[city] = p
    return owner, pos


def _cleanup_trips(trips: List[DroneTrip]) -> None:
    i = 0
    while i < len(trips):
        new_legs: List[DroneLeg] = []
        for leg in trips[i].legs:
            seen = set()
            uniq = []
            for c in leg.customers:
                if c not in seen:
                    seen.add(c)
                    uniq.append(c)
            if uniq:
                new_legs.append(DroneLeg(launch_city=leg.launch_city, customers=uniq))
        trips[i].legs = new_legs
        if not trips[i].legs:
            trips.pop(i)
        else:
            i += 1


def _cleanup_trucks(trucks: List[TruckRoute]) -> None:
    for route in trucks:
        for stop in route.stops:
            seen = set()
            uniq = []
            for c in stop.drone_customers:
                if c not in seen:
                    seen.add(c)
                    uniq.append(c)
            stop.drone_customers = uniq


def _remove_customer_everywhere(trucks: List[TruckRoute], trips: List[DroneTrip], customer: int) -> None:
    for route in trucks:
        for stop in route.stops:
            if customer in stop.drone_customers:
                stop.drone_customers = [c for c in stop.drone_customers if c != customer]
    for trip in trips:
        for leg in trip.legs:
            if customer in leg.customers:
                leg.customers = [c for c in leg.customers if c != customer]
    _cleanup_trucks(trucks)
    _cleanup_trips(trips)


def _trip_total_demand(trip: DroneTrip, data: ProblemData) -> float:
    return sum(data.demands[c] for leg in trip.legs for c in leg.customers)


def _launch_city_used(trips: List[DroneTrip], launch_city: int) -> bool:
    for trip in trips:
        for leg in trip.legs:
            if leg.launch_city == launch_city:
                return True
    return False


def _solution_eval(trucks: List[TruckRoute], trips: List[DroneTrip], data: ProblemData) -> Tuple[bool, float, float]:
    sol = Solution(truck_routes=trucks, drone_queue=trips)
    # Explicitly validate temporal constraints:
    # - flight + waiting energy limit
    # - delayed departure from depot by first synchronization
    # before using objective evaluation.
    val = validate_solution(sol, data)
    if not val.feasible:
        return False, float("inf"), float("inf")
    ev = evaluate_fitness(sol, data)
    if not ev.feasible:
        return False, float("inf"), float("inf")
    return True, ev.objective, sum(ev.truck_return_time.values())


def _customer_drop_occurrences(trucks: List[TruckRoute], customer: int) -> List[Tuple[int, int]]:
    out: List[Tuple[int, int]] = []
    for t_idx, route in enumerate(trucks):
        for s_idx, stop in enumerate(route.stops):
            if customer in stop.drone_customers:
                out.append((t_idx, s_idx))
    return out


def _invalid_customers(trucks: List[TruckRoute], routes: List[List[int]], data: ProblemData) -> List[int]:
    owner, pos = _owner_and_pos(routes)
    invalid: List[int] = []
    for customer in owner.keys():
        occ = _customer_drop_occurrences(trucks, customer)
        if len(occ) != 1:
            invalid.append(customer)
            continue

        t_idx, s_idx = occ[0]
        if t_idx != owner[customer]:
            invalid.append(customer)
            continue

        if s_idx > 0:
            customer_stop_pos = pos[customer] + 1
            if s_idx > customer_stop_pos:
                invalid.append(customer)
                continue
            launch_city = trucks[t_idx].stops[s_idx].city
            if not _is_within_drone_range(data, launch_city):
                invalid.append(customer)
                continue
    return sorted(set(invalid), key=lambda c: (owner[c], pos[c]))


def _build_candidate_structure(base_solution: Solution, moved_routes: List[List[int]]) -> Tuple[List[TruckRoute], List[DroneTrip]]:
    owner, _ = _owner_and_pos(moved_routes)

    old_by_city: Dict[int, List[int]] = {}
    old_depot_by_truck: Dict[int, List[int]] = {}
    for t_idx, route in enumerate(base_solution.truck_routes):
        old_depot_by_truck[t_idx] = route.stops[0].drone_customers[:] if route.stops else []
        for stop in route.stops[1:]:
            old_by_city[stop.city] = stop.drone_customers[:]

    new_trucks: List[TruckRoute] = []
    for t_idx, route in enumerate(moved_routes):
        depot_customers = [c for c in old_depot_by_truck.get(t_idx, []) if owner.get(c) == t_idx]
        stops = [TruckStop(city=0, drone_customers=depot_customers)]
        for city in route:
            city_customers = [c for c in old_by_city.get(city, []) if owner.get(c) == t_idx]
            stops.append(TruckStop(city=city, drone_customers=city_customers))
        new_trucks.append(TruckRoute(stops=stops))

    new_trips = copy.deepcopy(base_solution.drone_queue)
    _cleanup_trips(new_trips)
    _cleanup_trucks(new_trucks)
    return new_trucks, new_trips


def _trip_min_release(trip: DroneTrip, data: ProblemData) -> float:
    values = [data.release_dates[c] for leg in trip.legs for c in leg.customers]
    return min(values) if values else float("-inf")


def _leg_min_release(leg: DroneLeg, data: ProblemData) -> float:
    values = [data.release_dates[c] for c in leg.customers]
    return min(values) if values else float("-inf")


def _city_stop_index_map(trucks: List[TruckRoute]) -> List[Dict[int, int]]:
    maps: List[Dict[int, int]] = []
    for route in trucks:
        m: Dict[int, int] = {}
        for s_idx, stop in enumerate(route.stops):
            m[stop.city] = s_idx
        maps.append(m)
    return maps


def _set_unique_truck_assignment(trucks: List[TruckRoute], owner: int, stop_idx: int, customer: int) -> None:
    for route in trucks:
        for stop in route.stops:
            if customer in stop.drone_customers:
                stop.drone_customers = [c for c in stop.drone_customers if c != customer]
    trucks[owner].stops[stop_idx].drone_customers.append(customer)
    _cleanup_trucks(trucks)


def _normalize_truck_package_assignments(
    trucks: List[TruckRoute],
    routes: List[List[int]],
    data: ProblemData,
) -> None:
    owner, pos = _owner_and_pos(routes)
    city_stop = _city_stop_index_map(trucks)

    # Remove orphan package references that no longer belong to any truck route.
    for route in trucks:
        for stop in route.stops:
            stop.drone_customers = [c for c in stop.drone_customers if c in owner]

    for customer in owner.keys():
        occurrences = _customer_drop_occurrences(trucks, customer)
        valid: List[Tuple[int, int]] = []
        for t_idx, s_idx in occurrences:
            if t_idx != owner[customer]:
                continue
            if s_idx == 0:
                valid.append((t_idx, s_idx))
                continue
            customer_stop_pos = pos[customer] + 1
            if s_idx <= customer_stop_pos:
                launch_city = trucks[t_idx].stops[s_idx].city
                if _is_within_drone_range(data, launch_city):
                    valid.append((t_idx, s_idx))

        if valid:
            # Keep assignment closest to customer (latest feasible stop).
            chosen = max(valid, key=lambda x: x[1])
            _set_unique_truck_assignment(trucks, chosen[0], chosen[1], customer)
            continue

        # If nothing feasible remains, fallback to depot load of owner truck.
        owner_t = owner[customer]
        _set_unique_truck_assignment(trucks, owner_t, 0, customer)

    _cleanup_trucks(trucks)


def _block_duplicate_package_resupply(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    data: ProblemData,
) -> None:
    owner, pos = _owner_and_pos(routes)
    city_stop = _city_stop_index_map(trucks)

    # Keep only one resupply occurrence per customer in drone queue.
    occurrences: Dict[int, List[Tuple[int, int, int]]] = {}
    for tr_idx, trip in enumerate(trips):
        for leg_idx, leg in enumerate(trip.legs):
            for customer in leg.customers:
                occurrences.setdefault(customer, []).append((tr_idx, leg_idx, leg.launch_city))

    for customer, occ in occurrences.items():
        if len(occ) <= 1:
            continue
        if customer not in owner:
            continue

        # Keep occurrence on trip with largest min-release (requested tie-break spirit).
        keep = max(occ, key=lambda x: (_trip_min_release(trips[x[0]], data), -x[0], -x[1]))
        keep_launch = keep[2]
        keep_owner = owner[customer]
        keep_stop_idx = city_stop[keep_owner].get(keep_launch, 0)
        max_stop = pos[customer] + 1
        if keep_stop_idx > max_stop:
            keep_stop_idx = 0
        if keep_stop_idx > 0 and not _is_within_drone_range(data, trucks[keep_owner].stops[keep_stop_idx].city):
            keep_stop_idx = 0

        for tr_idx, leg_idx, _ in occ:
            if (tr_idx, leg_idx) == (keep[0], keep[1]):
                continue
            trips[tr_idx].legs[leg_idx].customers = [c for c in trips[tr_idx].legs[leg_idx].customers if c != customer]

        _set_unique_truck_assignment(trucks, keep_owner, keep_stop_idx, customer)

    _cleanup_trips(trips)
    _cleanup_trucks(trucks)


def _trip_meets_truck_at_other_launch(
    trip: DroneTrip,
    owner_map: Dict[int, int],
    owner: int,
    launch_city: int,
) -> bool:
    for leg in trip.legs:
        leg_owner = owner_map.get(leg.launch_city)
        if leg_owner == owner and leg.launch_city != launch_city:
            return True
    return False


def _move_customer_to_specific_launch(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    data: ProblemData,
    owner: int,
    customer: int,
    launch_stop_idx: int,
    forbidden_trip_idx: Optional[int] = None,
) -> Optional[Tuple[List[TruckRoute], List[DroneTrip]]]:
    owner_map, pos = _owner_and_pos(routes)
    if customer not in owner_map:
        return None
    if launch_stop_idx <= 0 or launch_stop_idx >= len(trucks[owner].stops):
        return None
    if launch_stop_idx > pos[customer] + 1:
        return None

    launch_city = trucks[owner].stops[launch_stop_idx].city
    if not _is_within_drone_range(data, launch_city):
        return None

    cand_trucks = copy.deepcopy(trucks)
    cand_trips = copy.deepcopy(trips)
    _remove_customer_everywhere(cand_trucks, cand_trips, customer)
    cand_trucks[owner].stops[launch_stop_idx].drone_customers.append(customer)

    demand = data.demands[customer]
    launch_taken = False
    for tr_idx, trip in enumerate(cand_trips):
        for leg in trip.legs:
            if leg.launch_city != launch_city:
                continue
            launch_taken = True
            if forbidden_trip_idx is not None and tr_idx == forbidden_trip_idx:
                continue
            if _trip_total_demand(trip, data) + demand > data.drone_capacity + EPS:
                continue
            if _trip_meets_truck_at_other_launch(trip, owner_map, owner, launch_city):
                continue
            leg.customers.append(customer)
            _cleanup_trucks(cand_trucks)
            _cleanup_trips(cand_trips)
            return cand_trucks, cand_trips

    if launch_taken:
        return None

    # No trip uses this launch city yet, create a new one-leg trip.
    cand_trips.append(DroneTrip(legs=[DroneLeg(launch_city=launch_city, customers=[customer])]))
    _cleanup_trucks(cand_trucks)
    _cleanup_trips(cand_trips)
    return cand_trucks, cand_trips


def _move_customer_to_immediate_next_stop(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    data: ProblemData,
    owner: int,
    customer: int,
    from_launch_stop_idx: int,
    forbidden_trip_idx: Optional[int] = None,
) -> Optional[Tuple[List[TruckRoute], List[DroneTrip]]]:
    # User-requested strict policy: move conflicting package to the NEXT truck stop.
    next_idx = from_launch_stop_idx + 1
    return _move_customer_to_specific_launch(
        trucks,
        trips,
        routes,
        data,
        owner,
        customer,
        next_idx,
        forbidden_trip_idx=forbidden_trip_idx,
    )


def _resolve_duplicate_launch_conflicts(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    data: ProblemData,
) -> Tuple[bool, List[int], List[TruckRoute], List[DroneTrip]]:
    owner_map, _ = _owner_and_pos(routes)
    city_stop = _city_stop_index_map(trucks)
    affected: List[int] = []

    cur_trucks = copy.deepcopy(trucks)
    cur_trips = copy.deepcopy(trips)

    while True:
        launch_occ: Dict[int, List[Tuple[int, int]]] = {}
        for tr_idx, trip in enumerate(cur_trips):
            for leg_idx, leg in enumerate(trip.legs):
                launch_occ.setdefault(leg.launch_city, []).append((tr_idx, leg_idx))

        conflict_launch = None
        conflict_occ = None
        for launch_city, occ in launch_occ.items():
            if len(occ) > 1:
                conflict_launch = launch_city
                conflict_occ = occ
                break
        if conflict_launch is None:
            break

        keep = max(conflict_occ, key=lambda x: _trip_min_release(cur_trips[x[0]], data))
        owner = owner_map.get(conflict_launch)
        if owner is None:
            return False, affected, cur_trucks, cur_trips
        from_stop_idx = city_stop[owner].get(conflict_launch)
        if from_stop_idx is None:
            return False, affected, cur_trucks, cur_trips

        for tr_idx, leg_idx in conflict_occ:
            if (tr_idx, leg_idx) == keep:
                continue
            customers = cur_trips[tr_idx].legs[leg_idx].customers[:]
            for customer in customers:
                moved = _move_customer_to_immediate_next_stop(
                    cur_trucks,
                    cur_trips,
                    routes,
                    data,
                    owner,
                    customer,
                    from_stop_idx,
                    forbidden_trip_idx=tr_idx,
                )
                if moved is None:
                    return False, affected, cur_trucks, cur_trips
                cur_trucks, cur_trips = moved
                affected.append(customer)

        _cleanup_trips(cur_trips)
        _cleanup_trucks(cur_trucks)

    return True, affected, cur_trucks, cur_trips


def _resolve_same_trip_same_truck_conflicts(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    data: ProblemData,
) -> Tuple[bool, List[int], List[TruckRoute], List[DroneTrip]]:
    owner_map, _ = _owner_and_pos(routes)
    city_stop = _city_stop_index_map(trucks)
    affected: List[int] = []

    cur_trucks = copy.deepcopy(trucks)
    cur_trips = copy.deepcopy(trips)

    changed = True
    while changed:
        changed = False
        for tr_idx, trip in enumerate(cur_trips):
            owner_to_legs: Dict[int, List[int]] = {}
            for leg_idx, leg in enumerate(trip.legs):
                owner = owner_map.get(leg.launch_city)
                if owner is None:
                    continue
                owner_to_legs.setdefault(owner, []).append(leg_idx)

            conflict_owner = None
            conflict_leg_indices = None
            for owner, leg_indices in owner_to_legs.items():
                if len(leg_indices) > 1:
                    conflict_owner = owner
                    conflict_leg_indices = leg_indices
                    break
            if conflict_owner is None:
                continue

            keep_leg_idx = max(conflict_leg_indices, key=lambda idx: _leg_min_release(trip.legs[idx], data))
            for leg_idx in conflict_leg_indices:
                if leg_idx == keep_leg_idx:
                    continue
                launch_city = cur_trips[tr_idx].legs[leg_idx].launch_city
                from_stop_idx = city_stop[conflict_owner].get(launch_city)
                if from_stop_idx is None:
                    return False, affected, cur_trucks, cur_trips
                customers = cur_trips[tr_idx].legs[leg_idx].customers[:]
                for customer in customers:
                    moved = _move_customer_to_immediate_next_stop(
                        cur_trucks,
                        cur_trips,
                        routes,
                        data,
                        conflict_owner,
                        customer,
                        from_stop_idx,
                        forbidden_trip_idx=tr_idx,
                    )
                    if moved is None:
                        return False, affected, cur_trucks, cur_trips
                    cur_trucks, cur_trips = moved
                    affected.append(customer)

            _cleanup_trips(cur_trips)
            _cleanup_trucks(cur_trucks)
            changed = True
            break

    return True, affected, cur_trucks, cur_trips


def _force_affected_to_depot_with_release_rule(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    data: ProblemData,
    affected_customers: List[int],
) -> Tuple[List[TruckRoute], List[DroneTrip]]:
    owner_map, _ = _owner_and_pos(routes)
    cur_trucks = copy.deepcopy(trucks)
    cur_trips = copy.deepcopy(trips)

    for customer in sorted(set(affected_customers), key=lambda c: data.release_dates[c], reverse=True):
        owner = owner_map.get(customer)
        if owner is None:
            continue
        cascaded = _depot_cascade_by_release(cur_trucks, cur_trips, routes, owner, customer, data)
        if cascaded is not None:
            cur_trucks, cur_trips = cascaded
            continue
        _remove_customer_everywhere(cur_trucks, cur_trips, customer)
        if customer not in cur_trucks[owner].stops[0].drone_customers:
            cur_trucks[owner].stops[0].drone_customers.append(customer)

    _cleanup_trucks(cur_trucks)
    _cleanup_trips(cur_trips)
    return cur_trucks, cur_trips


def _try_direct_depot(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    owner: int,
    customer: int,
    data: ProblemData,
) -> Optional[Tuple[List[TruckRoute], List[DroneTrip]]]:
    cand_trucks = copy.deepcopy(trucks)
    cand_trips = copy.deepcopy(trips)
    _remove_customer_everywhere(cand_trucks, cand_trips, customer)

    depot_customers = cand_trucks[owner].stops[0].drone_customers[:]
    max_release = max((data.release_dates[c] for c in depot_customers), default=float("inf"))
    if data.release_dates[customer] > max_release + EPS:
        return None

    if customer not in cand_trucks[owner].stops[0].drone_customers:
        cand_trucks[owner].stops[0].drone_customers.append(customer)
    _cleanup_trucks(cand_trucks)
    _cleanup_trips(cand_trips)

    ok, _, _ = _solution_eval(cand_trucks, cand_trips, data)
    if not ok:
        return None
    return cand_trucks, cand_trips


def _best_reuse_existing_sync(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    owner: int,
    customer_pos: int,
    customer: int,
    data: ProblemData,
) -> Optional[Tuple[List[TruckRoute], List[DroneTrip]]]:
    best: Optional[Tuple[List[TruckRoute], List[DroneTrip], float, float]] = None
    demand = data.demands[customer]

    for s_idx in range(1, customer_pos + 2):
        launch_city = trucks[owner].stops[s_idx].city
        if not _is_within_drone_range(data, launch_city):
            continue

        for tr_idx, trip in enumerate(trips):
            for leg_idx, leg in enumerate(trip.legs):
                if leg.launch_city != launch_city:
                    continue
                if _trip_total_demand(trip, data) + demand > data.drone_capacity + EPS:
                    continue

                cand_trucks = copy.deepcopy(trucks)
                cand_trips = copy.deepcopy(trips)
                _remove_customer_everywhere(cand_trucks, cand_trips, customer)
                cand_trucks[owner].stops[s_idx].drone_customers.append(customer)
                cand_trips[tr_idx].legs[leg_idx].customers.append(customer)
                _cleanup_trucks(cand_trucks)
                _cleanup_trips(cand_trips)

                ok, obj, truck_sum = _solution_eval(cand_trucks, cand_trips, data)
                if not ok:
                    continue
                if best is None or (obj, truck_sum) < (best[2], best[3]):
                    best = (cand_trucks, cand_trips, obj, truck_sum)

    if best is None:
        return None
    return best[0], best[1]


def _best_new_trip(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    owner: int,
    customer_pos: int,
    customer: int,
    data: ProblemData,
) -> Optional[Tuple[List[TruckRoute], List[DroneTrip]]]:
    if data.demands[customer] > data.drone_capacity + EPS:
        return None

    best: Optional[Tuple[List[TruckRoute], List[DroneTrip], float, float]] = None
    for require_used_launch in [True, False]:
        for s_idx in range(customer_pos + 1, 0, -1):
            launch_city = trucks[owner].stops[s_idx].city
            if not _is_within_drone_range(data, launch_city):
                continue
            used = _launch_city_used(trips, launch_city)
            if require_used_launch and not used:
                continue
            if (not require_used_launch) and used:
                continue

            cand_trucks = copy.deepcopy(trucks)
            cand_trips = copy.deepcopy(trips)
            _remove_customer_everywhere(cand_trucks, cand_trips, customer)
            cand_trucks[owner].stops[s_idx].drone_customers.append(customer)
            cand_trips.append(DroneTrip(legs=[DroneLeg(launch_city=launch_city, customers=[customer])]))
            _cleanup_trucks(cand_trucks)
            _cleanup_trips(cand_trips)

            ok, obj, truck_sum = _solution_eval(cand_trucks, cand_trips, data)
            if not ok:
                continue
            if best is None or (obj, truck_sum) < (best[2], best[3]):
                best = (cand_trucks, cand_trips, obj, truck_sum)

        if best is not None:
            break

    if best is None:
        return None
    return best[0], best[1]


def _depot_cascade_by_release(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    owner: int,
    customer: int,
    data: ProblemData,
) -> Optional[Tuple[List[TruckRoute], List[DroneTrip]]]:
    threshold = data.release_dates[customer]
    same_truck_earlier = [c for c in routes[owner] if data.release_dates[c] < threshold]
    affected = set(same_truck_earlier + [customer])

    cand_trucks = copy.deepcopy(trucks)
    cand_trips = copy.deepcopy(trips)
    for c in affected:
        _remove_customer_everywhere(cand_trucks, cand_trips, c)
        if c not in cand_trucks[owner].stops[0].drone_customers:
            cand_trucks[owner].stops[0].drone_customers.append(c)
    _cleanup_trucks(cand_trucks)
    _cleanup_trips(cand_trips)

    ok, _, _ = _solution_eval(cand_trucks, cand_trips, data)
    if not ok:
        return None
    return cand_trucks, cand_trips


def _deterministic_repair(
    trucks: List[TruckRoute],
    trips: List[DroneTrip],
    routes: List[List[int]],
    data: ProblemData,
    preferred_customers: Optional[List[int]] = None,
) -> Tuple[List[TruckRoute], List[DroneTrip]]:
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

        direct = _try_direct_depot(cur_trucks, cur_trips, t_idx, customer, data)
        if direct is not None:
            cur_trucks, cur_trips = direct
            continue

        reuse = _best_reuse_existing_sync(cur_trucks, cur_trips, t_idx, c_pos, customer, data)
        if reuse is not None:
            cur_trucks, cur_trips = reuse
            continue

        created = _best_new_trip(cur_trucks, cur_trips, t_idx, c_pos, customer, data)
        if created is not None:
            cur_trucks, cur_trips = created
            continue

        cascade = _depot_cascade_by_release(cur_trucks, cur_trips, routes, t_idx, customer, data)
        if cascade is not None:
            cur_trucks, cur_trips = cascade

    ok, _, _ = _solution_eval(cur_trucks, cur_trips, data)
    if ok:
        return cur_trucks, cur_trips

    # Safety fallback.
    fallback_trucks = copy.deepcopy(cur_trucks)
    for t_idx, route in enumerate(routes):
        fallback_trucks[t_idx].stops[0].drone_customers = route[:]
        for s_idx in range(1, len(fallback_trucks[t_idx].stops)):
            fallback_trucks[t_idx].stops[s_idx].drone_customers = []
    return fallback_trucks, []


def repair_solution_after_truck_routes(
    base_solution: Solution | Any,
    moved_routes: List[List[int]],
    data: ProblemData,
    affected_customers: Optional[List[int]] = None,
) -> Optional[Solution]:
    """
    Deterministic feasibility repair after truck-route modifications.
    This operator is generic and can be reused by multiple truck neighborhoods.
    """
    base = _clone_solution(_to_solution(base_solution))
    pref = list(affected_customers or [])

    trucks, trips = _build_candidate_structure(base, moved_routes)
    _cleanup_trucks(trucks)
    _cleanup_trips(trips)
    _normalize_truck_package_assignments(trucks, moved_routes, data)
    _block_duplicate_package_resupply(trucks, trips, moved_routes, data)

    ok_dup_launch, affected1, trucks, trips = _resolve_duplicate_launch_conflicts(
        trucks,
        trips,
        moved_routes,
        data,
    )
    if not ok_dup_launch:
        pref.extend(affected1)
        trucks, trips = _force_affected_to_depot_with_release_rule(trucks, trips, moved_routes, data, pref)
    else:
        pref.extend(affected1)

    ok_same_trip, affected2, trucks, trips = _resolve_same_trip_same_truck_conflicts(
        trucks,
        trips,
        moved_routes,
        data,
    )
    if not ok_same_trip:
        pref.extend(affected2)
        trucks, trips = _force_affected_to_depot_with_release_rule(trucks, trips, moved_routes, data, pref)
    else:
        pref.extend(affected2)

    _cleanup_trucks(trucks)
    _cleanup_trips(trips)
    _normalize_truck_package_assignments(trucks, moved_routes, data)
    _block_duplicate_package_resupply(trucks, trips, moved_routes, data)

    trucks, trips = _deterministic_repair(
        trucks,
        trips,
        moved_routes,
        data,
        preferred_customers=pref,
    )
    _cleanup_trucks(trucks)
    _cleanup_trips(trips)
    _normalize_truck_package_assignments(trucks, moved_routes, data)
    _block_duplicate_package_resupply(trucks, trips, moved_routes, data)

    ok, _, _ = _solution_eval(trucks, trips, data)
    if ok:
        return Solution(truck_routes=trucks, drone_queue=trips)

    # Final fallback required by user:
    # affected packages -> depot and clear lower-release resupplies on same truck.
    if not pref:
        pref = _invalid_customers(trucks, moved_routes, data)
    if not pref:
        owner_map, _ = _owner_and_pos(moved_routes)
        pref = list(owner_map.keys())

    trucks2, trips2 = _force_affected_to_depot_with_release_rule(trucks, trips, moved_routes, data, pref)
    trucks2, trips2 = _deterministic_repair(
        trucks2,
        trips2,
        moved_routes,
        data,
        preferred_customers=pref,
    )
    _cleanup_trucks(trucks2)
    _cleanup_trips(trips2)
    _normalize_truck_package_assignments(trucks2, moved_routes, data)
    _block_duplicate_package_resupply(trucks2, trips2, moved_routes, data)

    ok2, _, _ = _solution_eval(trucks2, trips2, data)
    if not ok2:
        return None
    return Solution(truck_routes=trucks2, drone_queue=trips2)


def _generate_truck_neighbors_from_raw_moves(
    base_solution: Solution,
    raw_moves: List[Tuple[List[List[int]], List[int], str]],
    neighborhood_name: str,
    data: ProblemData,
    max_neighbors: Optional[int] = None,
) -> List[TruckNeighbor]:
    seen = set()
    out: List[TruckNeighbor] = []
    for moved_routes, moved_customers, detail in raw_moves:
        repaired = repair_solution_after_truck_routes(
            base_solution,
            moved_routes,
            data,
            affected_customers=moved_customers,
        )
        if repaired is None:
            continue

        ev = evaluate_fitness(repaired, data)
        if not ev.feasible:
            continue

        signature = repr(repaired.to_legacy())
        if signature in seen:
            continue
        seen.add(signature)

        out.append(
            TruckNeighbor(
                neighborhood=neighborhood_name,
                moved_customers=moved_customers[:],
                solution=repaired,
                objective=ev.objective,
                truck_completion_sum=sum(ev.truck_return_time.values()),
                move_detail=detail,
            )
        )

    out.sort(key=lambda n: (n.objective, n.truck_completion_sum, n.move_detail))
    if max_neighbors is not None and max_neighbors >= 0:
        return out[:max_neighbors]
    return out


def generate_truck_neighbors_move_1_0(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = None,
) -> List[TruckNeighbor]:
    base = _clone_solution(_to_solution(solution))
    base_routes = _routes_from_solution(base)
    raw_moves = move_1_0_routes(base_routes)
    return _generate_truck_neighbors_from_raw_moves(
        base,
        raw_moves,
        "(1,0)",
        data,
        max_neighbors=max_neighbors,
    )


def generate_truck_neighbors_move_1_1(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = None,
) -> List[TruckNeighbor]:
    base = _clone_solution(_to_solution(solution))
    base_routes = _routes_from_solution(base)
    raw_moves = move_1_1_routes(base_routes)
    return _generate_truck_neighbors_from_raw_moves(
        base,
        raw_moves,
        "(1,1)",
        data,
        max_neighbors=max_neighbors,
    )


def generate_truck_neighbors_move_2_0(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = None,
) -> List[TruckNeighbor]:
    base = _clone_solution(_to_solution(solution))
    base_routes = _routes_from_solution(base)
    raw_moves = move_2_0_routes(base_routes)
    return _generate_truck_neighbors_from_raw_moves(
        base,
        raw_moves,
        "(2,0)",
        data,
        max_neighbors=max_neighbors,
    )


def generate_truck_neighbors_move_2_1(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = None,
) -> List[TruckNeighbor]:
    base = _clone_solution(_to_solution(solution))
    base_routes = _routes_from_solution(base)
    raw_moves = move_2_1_routes(base_routes)
    return _generate_truck_neighbors_from_raw_moves(
        base,
        raw_moves,
        "(2,1)",
        data,
        max_neighbors=max_neighbors,
    )


def generate_truck_neighbors_move_2_opt(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = None,
) -> List[TruckNeighbor]:
    base = _clone_solution(_to_solution(solution))
    base_routes = _routes_from_solution(base)
    raw_moves = move_2_opt_routes(base_routes)
    return _generate_truck_neighbors_from_raw_moves(
        base,
        raw_moves,
        "(2-opt)",
        data,
        max_neighbors=max_neighbors,
    )


def generate_truck_diversification_neighbors(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = 200,
    rng: Optional[random.Random] = None,
    max_moves_per_route: Optional[int] = 80,
) -> List[TruckNeighbor]:
    base = _clone_solution(_to_solution(solution))
    base_routes = _routes_from_solution(base)
    raw_moves = move_swap_two_segments_routes(
        base_routes,
        rng=rng,
        max_moves_per_route=max_moves_per_route,
    )
    return _generate_truck_neighbors_from_raw_moves(
        base,
        raw_moves,
        "diversification",
        data,
        max_neighbors=max_neighbors,
    )


def diversification_truck(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = 200,
    rng: Optional[random.Random] = None,
    max_moves_per_route: Optional[int] = 80,
    accept_non_improving: bool = True,
) -> Tuple[Solution, bool]:
    """
    Return a diversified solution and whether it improves the current objective.
    - If accept_non_improving=True, returns best feasible diversified neighbor even when not improving.
    - If accept_non_improving=False, returns current solution when no improving diversified neighbor exists.
    """
    base = _clone_solution(_to_solution(solution))
    base_eval = evaluate_fitness(base, data)
    if not base_eval.feasible:
        return base, False
    base_key = (base_eval.objective, sum(base_eval.truck_return_time.values()))

    neighbors = generate_truck_diversification_neighbors(
        base,
        data,
        max_neighbors=max_neighbors,
        rng=rng,
        max_moves_per_route=max_moves_per_route,
    )
    if not neighbors:
        return base, False

    best = min(neighbors, key=lambda n: (n.objective, n.truck_completion_sum, n.move_detail))
    improved = (best.objective, best.truck_completion_sum) < base_key
    if improved or accept_non_improving:
        return _clone_solution(best.solution), improved
    return base, False


def _local_search_truck_by_generator(
    solution: Solution | Any,
    data: ProblemData,
    neighbor_generator: Callable[..., List[TruckNeighbor]],
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> Solution:
    current = _clone_solution(_to_solution(solution))
    cur_eval = evaluate_fitness(current, data)
    if not cur_eval.feasible:
        return current
    cur_key = (cur_eval.objective, sum(cur_eval.truck_return_time.values()))

    limit = (data.number_of_cities - 1) if max_iterations is None else max_iterations
    if limit < 1:
        return current

    for _ in range(limit):
        neighbors = neighbor_generator(current, data, max_neighbors=max_neighbors)
        improving = [n for n in neighbors if (n.objective, n.truck_completion_sum) < cur_key]
        if not improving:
            break
        best = min(improving, key=lambda n: (n.objective, n.truck_completion_sum, n.move_detail))
        current = _clone_solution(best.solution)
        cur_key = (best.objective, best.truck_completion_sum)
    return current


def local_search_truck_move_1_0(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> Solution:
    return _local_search_truck_by_generator(
        solution,
        data,
        neighbor_generator=generate_truck_neighbors_move_1_0,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    )


def local_search_truck_move_1_1(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> Solution:
    return _local_search_truck_by_generator(
        solution,
        data,
        neighbor_generator=generate_truck_neighbors_move_1_1,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    )


def local_search_truck_move_2_0(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> Solution:
    return _local_search_truck_by_generator(
        solution,
        data,
        neighbor_generator=generate_truck_neighbors_move_2_0,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    )


def local_search_truck_move_2_1(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> Solution:
    return _local_search_truck_by_generator(
        solution,
        data,
        neighbor_generator=generate_truck_neighbors_move_2_1,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    )


def local_search_truck_move_2_opt(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> Solution:
    return _local_search_truck_by_generator(
        solution,
        data,
        neighbor_generator=generate_truck_neighbors_move_2_opt,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    )


def local_search_truck_move_1_0_legacy(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> List[Any]:
    return local_search_truck_move_1_0(
        solution,
        data,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    ).to_legacy()


def local_search_truck_move_1_1_legacy(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> List[Any]:
    return local_search_truck_move_1_1(
        solution,
        data,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    ).to_legacy()


def local_search_truck_move_2_0_legacy(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> List[Any]:
    return local_search_truck_move_2_0(
        solution,
        data,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    ).to_legacy()


def local_search_truck_move_2_1_legacy(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> List[Any]:
    return local_search_truck_move_2_1(
        solution,
        data,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    ).to_legacy()


def local_search_truck_move_2_opt_legacy(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 300,
) -> List[Any]:
    return local_search_truck_move_2_opt(
        solution,
        data,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    ).to_legacy()


def diversification_truck_legacy(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = 200,
    rng: Optional[random.Random] = None,
    max_moves_per_route: Optional[int] = 80,
    accept_non_improving: bool = True,
) -> Tuple[List[Any], bool]:
    sol, improved = diversification_truck(
        solution,
        data,
        max_neighbors=max_neighbors,
        rng=rng,
        max_moves_per_route=max_moves_per_route,
        accept_non_improving=accept_non_improving,
    )
    return sol.to_legacy(), improved
