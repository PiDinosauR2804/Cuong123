from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional, Tuple

from .fitness import evaluate_fitness
from .problem_data import ProblemData
from .solution import DroneLeg, DroneTrip, Solution
from .validator import validate_solution


EPS = 1e-9
DIFFERENTIAL_RATE_RELEASE_TIME = 1.0
B_RATIO = 0.7
C_RATIO = 0.1


@dataclass
class DroneNeighbor:
    move: str
    solution: Solution
    objective: float
    truck_sum: float


def _clone_solution(solution: Solution) -> Solution:
    return Solution.from_legacy(solution.to_legacy())


def _solution_signature(solution: Solution) -> str:
    return repr(solution.to_legacy())


def _objective_key(solution: Solution, data: ProblemData) -> Optional[Tuple[float, float]]:
    result = evaluate_fitness(solution, data)
    if not result.feasible:
        return None
    return result.objective, sum(result.truck_return_time.values())


def _is_better(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    if a[0] < b[0] - EPS:
        return True
    if abs(a[0] - b[0]) <= EPS and a[1] < b[1] - EPS:
        return True
    return False


def _owner_and_pos(solution: Solution) -> Tuple[Dict[int, int], List[Dict[int, int]]]:
    owner: Dict[int, int] = {}
    pos: List[Dict[int, int]] = []
    for t_idx, route in enumerate(solution.truck_routes):
        pos_map: Dict[int, int] = {}
        for s_idx, stop in enumerate(route.stops):
            if s_idx == 0 or stop.city == 0:
                continue
            owner[stop.city] = t_idx
            if stop.city not in pos_map:
                pos_map[stop.city] = s_idx
        pos.append(pos_map)
    return owner, pos


def _trip_total_demand(trip: DroneTrip, data: ProblemData) -> float:
    total = 0.0
    for leg in trip.legs:
        for c in leg.customers:
            total += data.demands[c]
    return total


def _release_std(data: ProblemData) -> float:
    values = [data.release_dates[i] for i in range(1, data.number_of_cities)]
    if len(values) <= 1:
        return 0.0
    mean = sum(values) / len(values)
    var = sum((v - mean) * (v - mean) for v in values) / len(values)
    return math.sqrt(max(0.0, var))


def _trip_customers(trip: DroneTrip) -> List[int]:
    out: List[int] = []
    for leg in trip.legs:
        out.extend(leg.customers)
    return out


def _release_compatible_with_trip(
    data: ProblemData,
    trip: DroneTrip,
    incoming_customers: List[int],
    b_ratio: float = B_RATIO,
) -> bool:
    incoming = [c for c in incoming_customers if 0 <= c < data.number_of_cities and c != 0]
    if not incoming:
        return True
    base = _trip_customers(trip)
    if not base:
        return True

    incoming_max = max(data.release_dates[c] for c in incoming)
    base_max = max(data.release_dates[c] for c in base)
    base_min = min(data.release_dates[c] for c in base)
    std = _release_std(data)
    return (
        base_max * DIFFERENTIAL_RATE_RELEASE_TIME + std >= incoming_max - EPS
        and base_min * DIFFERENTIAL_RATE_RELEASE_TIME + b_ratio * std >= incoming_max - EPS
    )


def _depot_prefers_keep_customer(
    solution: Solution,
    data: ProblemData,
    owner_truck: int,
    customer: int,
    c_ratio: float = C_RATIO,
) -> bool:
    if owner_truck < 0 or owner_truck >= len(solution.truck_routes):
        return False
    if not (0 <= customer < data.number_of_cities) or customer == 0:
        return False

    depot_customers = [c for c in solution.truck_routes[owner_truck].stops[0].drone_customers if c != customer]
    if not depot_customers:
        return False

    std = _release_std(data)
    r_customer = data.release_dates[customer]
    r_max = max(data.release_dates[c] for c in depot_customers)
    r_min = min(data.release_dates[c] for c in depot_customers)
    return (
        r_max * DIFFERENTIAL_RATE_RELEASE_TIME + std >= r_customer - EPS
        and r_min * DIFFERENTIAL_RATE_RELEASE_TIME + c_ratio * std >= r_customer - EPS
    )


def _is_reachable_from_depot(data: ProblemData, city: int) -> bool:
    round_trip = 2.0 * data.drone_time_matrix[0][city] + data.unloading_time
    return round_trip <= data.drone_limit_time + EPS


def _cleanup(solution: Solution) -> None:
    for route in solution.truck_routes:
        for stop in route.stops:
            seen = set()
            uniq = []
            for c in stop.drone_customers:
                if c not in seen:
                    seen.add(c)
                    uniq.append(c)
            stop.drone_customers = uniq

    new_queue: List[DroneTrip] = []
    for trip in solution.drone_queue:
        new_legs: List[DroneLeg] = []
        for leg in trip.legs:
            seen = set()
            uniq = []
            for c in leg.customers:
                if c not in seen:
                    seen.add(c)
                    uniq.append(c)
            if uniq:
                new_legs.append(DroneLeg(launch_city=leg.launch_city, customers=uniq))
        if new_legs:
            new_queue.append(DroneTrip(legs=new_legs))
    solution.drone_queue = new_queue


def _remove_customer_everywhere(solution: Solution, customer: int) -> None:
    for route in solution.truck_routes:
        for stop in route.stops:
            if customer in stop.drone_customers:
                stop.drone_customers = [c for c in stop.drone_customers if c != customer]
    for trip in solution.drone_queue:
        for leg in trip.legs:
            if customer in leg.customers:
                leg.customers = [c for c in leg.customers if c != customer]
    _cleanup(solution)


def _add_customer_to_depot(solution: Solution, owner_truck: int, customer: int) -> None:
    depot = solution.truck_routes[owner_truck].stops[0]
    if customer not in depot.drone_customers:
        depot.drone_customers.append(customer)


def _find_customer_owner(owner_map: Dict[int, int], customer: int) -> Optional[int]:
    return owner_map.get(customer)


def _assign_customer_to_launch(
    solution: Solution,
    data: ProblemData,
    customer: int,
    launch_city: int,
) -> bool:
    owner_map, pos_map_by_truck = _owner_and_pos(solution)
    owner_customer = _find_customer_owner(owner_map, customer)
    owner_launch = owner_map.get(launch_city)
    if owner_customer is None or owner_launch is None:
        return False
    if owner_customer != owner_launch:
        return False

    pos_launch = pos_map_by_truck[owner_customer].get(launch_city)
    pos_customer = pos_map_by_truck[owner_customer].get(customer)
    if pos_launch is None or pos_customer is None or pos_launch > pos_customer:
        return False

    _remove_customer_everywhere(solution, customer)

    # truck assignment
    target_stop = None
    for idx, stop in enumerate(solution.truck_routes[owner_customer].stops):
        if stop.city == launch_city:
            target_stop = idx
            break
    if target_stop is None:
        return False
    if customer not in solution.truck_routes[owner_customer].stops[target_stop].drone_customers:
        solution.truck_routes[owner_customer].stops[target_stop].drone_customers.append(customer)

    # drone assignment
    for trip in solution.drone_queue:
        for leg in trip.legs:
            if leg.launch_city == launch_city:
                if not _release_compatible_with_trip(data, trip, [customer], b_ratio=B_RATIO):
                    return False
                if customer not in leg.customers:
                    leg.customers.append(customer)
                _cleanup(solution)
                return True

    solution.drone_queue.append(DroneTrip(legs=[DroneLeg(launch_city=launch_city, customers=[customer])]))
    _cleanup(solution)
    return True


def _repair_duplicate_launch(solution: Solution, data: ProblemData) -> None:
    owner_map, _ = _owner_and_pos(solution)
    launch_to_leg: Dict[int, Tuple[int, int]] = {}
    for trip_idx, trip in enumerate(solution.drone_queue):
        for leg_idx, leg in enumerate(trip.legs):
            launch = leg.launch_city
            previous = launch_to_leg.get(launch)
            if previous is None:
                launch_to_leg[launch] = (trip_idx, leg_idx)
                continue

            keep_trip_idx, keep_leg_idx = previous
            if keep_trip_idx >= len(solution.drone_queue):
                continue
            keep_trip = solution.drone_queue[keep_trip_idx]
            if keep_leg_idx >= len(keep_trip.legs):
                continue
            keep_leg = keep_trip.legs[keep_leg_idx]

            for customer in list(leg.customers):
                if (
                    _trip_total_demand(keep_trip, data) + data.demands[customer] <= data.drone_capacity + EPS
                    and _release_compatible_with_trip(data, keep_trip, [customer], b_ratio=B_RATIO)
                ):
                    keep_leg.customers.append(customer)
                else:
                    owner = owner_map.get(customer)
                    if owner is not None:
                        _add_customer_to_depot(solution, owner, customer)
            leg.customers = []
    _cleanup(solution)


def _repair_trip_meet_same_truck(solution: Solution) -> None:
    owner_map, _ = _owner_and_pos(solution)
    for trip in solution.drone_queue:
        seen_trucks = set()
        kept_legs: List[DroneLeg] = []
        moved_to_depot: List[int] = []
        for leg in trip.legs:
            owner = owner_map.get(leg.launch_city)
            if owner is None:
                moved_to_depot.extend(leg.customers)
                continue
            if owner in seen_trucks:
                moved_to_depot.extend(leg.customers)
                continue
            seen_trucks.add(owner)
            kept_legs.append(leg)
        trip.legs = kept_legs
        for customer in moved_to_depot:
            owner = owner_map.get(customer)
            if owner is not None:
                _add_customer_to_depot(solution, owner, customer)
    _cleanup(solution)


def _repair_capacity_like_test_similarity(solution: Solution, data: ProblemData) -> None:
    owner_map, _ = _owner_and_pos(solution)
    for trip in solution.drone_queue:
        while _trip_total_demand(trip, data) > data.drone_capacity + EPS:
            # remove one package (latest release) and send back to depot.
            candidates: List[Tuple[float, int, int]] = []
            for leg_idx, leg in enumerate(trip.legs):
                for customer in leg.customers:
                    candidates.append((data.release_dates[customer], leg_idx, customer))
            if not candidates:
                break
            _, leg_idx, customer = max(candidates)
            leg = trip.legs[leg_idx]
            leg.customers.remove(customer)
            owner = owner_map.get(customer)
            if owner is not None:
                _add_customer_to_depot(solution, owner, customer)
    _cleanup(solution)


def _repair_queue_order(solution: Solution) -> None:
    owner_map, pos_map_by_truck = _owner_and_pos(solution)
    trip_pos_by_truck: List[Dict[int, int]] = []
    for trip in solution.drone_queue:
        pos_map: Dict[int, int] = {}
        for leg in trip.legs:
            owner = owner_map.get(leg.launch_city)
            if owner is None:
                continue
            if owner in pos_map:
                continue
            launch_pos = pos_map_by_truck[owner].get(leg.launch_city)
            if launch_pos is None:
                continue
            pos_map[owner] = launch_pos
        trip_pos_by_truck.append(pos_map)

    order = list(range(len(solution.drone_queue)))
    max_rounds = max(1, len(order) * 2)
    for _ in range(max_rounds):
        changed = False
        for truck in range(len(solution.truck_routes)):
            positions = [p for p, trip_idx in enumerate(order) if truck in trip_pos_by_truck[trip_idx]]
            if len(positions) <= 1:
                continue
            indices = [order[p] for p in positions]
            indices_sorted = sorted(indices, key=lambda i: trip_pos_by_truck[i][truck])
            if indices != indices_sorted:
                changed = True
                for p, sorted_idx in zip(positions, indices_sorted):
                    order[p] = sorted_idx
        if not changed:
            break

    solution.drone_queue = [solution.drone_queue[idx] for idx in order]
    _cleanup(solution)


def _repair_flight_limit(solution: Solution, data: ProblemData) -> None:
    owner_map, _ = _owner_and_pos(solution)
    for _ in range(3):
        v = validate_solution(solution, data)
        changed = False
        for trip_idx in range(len(solution.drone_queue)):
            flight_wait = v.trip_flight_wait_time.get(trip_idx, 0.0)
            if flight_wait <= data.drone_limit_time + EPS:
                continue
            trip = solution.drone_queue[trip_idx]
            while trip.legs and v.trip_flight_wait_time.get(trip_idx, 0.0) > data.drone_limit_time + EPS:
                # move one package from the latest leg back to depot
                last_leg = trip.legs[-1]
                if not last_leg.customers:
                    trip.legs.pop()
                    continue
                customer = max(last_leg.customers, key=lambda c: data.release_dates[c])
                last_leg.customers.remove(customer)
                owner = owner_map.get(customer)
                if owner is not None:
                    _add_customer_to_depot(solution, owner, customer)
                _cleanup(solution)
                v = validate_solution(solution, data)
                changed = True
        if not changed:
            break
    _cleanup(solution)


def _repair_solution(solution: Solution, data: ProblemData) -> None:
    for _ in range(4):
        sig_before = _solution_signature(solution)
        _cleanup(solution)
        _repair_trip_meet_same_truck(solution)
        _repair_capacity_like_test_similarity(solution, data)
        _repair_duplicate_launch(solution, data)
        _repair_queue_order(solution)
        _repair_flight_limit(solution, data)
        _cleanup(solution)
        if _solution_signature(solution) == sig_before:
            break


def _collect_depot_customers(solution: Solution) -> List[Tuple[int, int]]:
    out: List[Tuple[int, int]] = []
    for t_idx, route in enumerate(solution.truck_routes):
        if route.stops:
            for c in route.stops[0].drone_customers:
                out.append((t_idx, c))
    return out


def _candidate_launches_for_customer(solution: Solution, data: ProblemData, customer: int) -> List[int]:
    owner_map, pos_map_by_truck = _owner_and_pos(solution)
    owner = owner_map.get(customer)
    if owner is None:
        return []
    pos_customer = pos_map_by_truck[owner].get(customer)
    if pos_customer is None:
        return []
    route = solution.truck_routes[owner]
    candidates: List[int] = []
    for idx in range(pos_customer, 0, -1):
        city = route.stops[idx].city
        if city == 0:
            continue
        if _is_reachable_from_depot(data, city):
            candidates.append(city)
    return candidates


def _op_activate_from_depot(solution: Solution, data: ProblemData, max_branch: int = 12) -> List[Tuple[str, Solution]]:
    result: List[Tuple[str, Solution]] = []
    depot_customers = _collect_depot_customers(solution)
    depot_customers.sort(
        key=lambda x: (
            _depot_prefers_keep_customer(solution, data, x[0], x[1], c_ratio=C_RATIO),
            -data.release_dates[x[1]],
        )
    )

    count = 0
    for owner_truck, customer in depot_customers:
        keep_pref = _depot_prefers_keep_customer(solution, data, owner_truck, customer, c_ratio=C_RATIO)
        launches = _candidate_launches_for_customer(solution, data, customer)
        if not launches:
            continue
        launch_limit = 1 if keep_pref else 2
        for launch_city in launches[:launch_limit]:
            cand = _clone_solution(solution)
            ok = _assign_customer_to_launch(cand, data, customer, launch_city)
            if not ok:
                continue
            _repair_solution(cand, data)
            result.append((f"activate_depot:{customer}->{launch_city}", cand))
            count += 1
            if count >= max_branch:
                return result
    return result


def _op_relocate_existing(solution: Solution, data: ProblemData, max_branch: int = 30) -> List[Tuple[str, Solution]]:
    result: List[Tuple[str, Solution]] = []
    count = 0
    for trip in solution.drone_queue:
        for leg in trip.legs:
            for customer in leg.customers:
                launches = _candidate_launches_for_customer(solution, data, customer)
                for launch_city in launches[:3]:
                    if launch_city == leg.launch_city:
                        continue
                    cand = _clone_solution(solution)
                    ok = _assign_customer_to_launch(cand, data, customer, launch_city)
                    if not ok:
                        continue
                    _repair_solution(cand, data)
                    result.append((f"relocate:{customer}:{leg.launch_city}->{launch_city}", cand))
                    count += 1
                    if count >= max_branch:
                        return result
    return result


def _op_merge_trips(solution: Solution, data: ProblemData, max_branch: int = 12) -> List[Tuple[str, Solution]]:
    result: List[Tuple[str, Solution]] = []
    count = 0
    for i in range(len(solution.drone_queue)):
        for j in range(i + 1, len(solution.drone_queue)):
            cand = _clone_solution(solution)
            incoming_customers = _trip_customers(cand.drone_queue[j])
            if not _release_compatible_with_trip(data, cand.drone_queue[i], incoming_customers, b_ratio=B_RATIO):
                continue
            merged_legs = cand.drone_queue[i].legs + cand.drone_queue[j].legs
            cand.drone_queue[i] = DroneTrip(legs=merged_legs)
            cand.drone_queue.pop(j)
            _repair_solution(cand, data)
            result.append((f"merge:{i}+{j}", cand))
            count += 1
            if count >= max_branch:
                return result
    return result


def _op_reorder_queue(solution: Solution, data: ProblemData, max_branch: int = 20) -> List[Tuple[str, Solution]]:
    result: List[Tuple[str, Solution]] = []
    count = 0
    n = len(solution.drone_queue)
    for i in range(n):
        for j in range(n + 1):
            if j == i or j == i + 1:
                continue
            cand = _clone_solution(solution)
            trip = cand.drone_queue.pop(i)
            insert_pos = j - 1 if i < j else j
            cand.drone_queue.insert(insert_pos, trip)
            _repair_solution(cand, data)
            result.append((f"reorder:{i}->{j}", cand))
            count += 1
            if count >= max_branch:
                return result
    return result


def generate_drone_neighbors(
    solution: Solution | Any,
    data: ProblemData,
    max_neighbors: Optional[int] = 120,
) -> List[DroneNeighbor]:
    base_raw = solution if isinstance(solution, Solution) else Solution.from_legacy(solution)
    base = _clone_solution(base_raw)
    _repair_solution(base, data)

    candidates: Dict[str, DroneNeighbor] = {}
    moves: List[Tuple[str, Solution]] = []
    moves.extend(_op_activate_from_depot(base, data))
    moves.extend(_op_relocate_existing(base, data))
    moves.extend(_op_merge_trips(base, data))
    moves.extend(_op_reorder_queue(base, data))

    for move_name, cand in moves:
        key = _objective_key(cand, data)
        if key is None:
            continue
        sig = _solution_signature(cand)
        prev = candidates.get(sig)
        neighbor = DroneNeighbor(
            move=move_name,
            solution=cand,
            objective=key[0],
            truck_sum=key[1],
        )
        if prev is None or (neighbor.objective, neighbor.truck_sum) < (prev.objective, prev.truck_sum):
            candidates[sig] = neighbor

    ranked = sorted(candidates.values(), key=lambda n: (n.objective, n.truck_sum, n.move))
    if max_neighbors is not None and max_neighbors >= 0:
        return ranked[:max_neighbors]
    return ranked


def local_search_drone(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 120,
) -> Solution:
    current = solution if isinstance(solution, Solution) else Solution.from_legacy(solution)
    current = _clone_solution(current)
    _repair_solution(current, data)
    current_key = _objective_key(current, data)
    if current_key is None:
        return current

    limit = (data.number_of_cities - 1) if max_iterations is None else max_iterations
    if limit < 1:
        return current

    for _ in range(limit):
        neighbors = generate_drone_neighbors(current, data, max_neighbors=max_neighbors)
        improving = [n for n in neighbors if _is_better((n.objective, n.truck_sum), current_key)]
        if not improving:
            break
        best = min(improving, key=lambda n: (n.objective, n.truck_sum, n.move))
        current = _clone_solution(best.solution)
        current_key = (best.objective, best.truck_sum)
    return current


def local_search_drone_legacy(
    solution: Solution | Any,
    data: ProblemData,
    max_iterations: Optional[int] = None,
    max_neighbors: Optional[int] = 120,
) -> List[Any]:
    return local_search_drone(
        solution,
        data,
        max_iterations=max_iterations,
        max_neighbors=max_neighbors,
    ).to_legacy()
