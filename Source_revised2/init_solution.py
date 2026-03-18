from __future__ import annotations

from typing import Any, Dict, List, Tuple

from .drone_local_search import generate_drone_neighbors, local_search_drone
from .fitness import evaluate_fitness
from .problem_data import ProblemData
from .solution import Solution, TruckRoute, TruckStop


EPS = 1e-9


def compute_phase1_earliest_departure(data: ProblemData) -> Dict[int, float]:
    """
    e_i = max(truck_time(depot, i), release_i + drone_time(depot, i))
    """
    e: Dict[int, float] = {0: 0.0}
    for city in range(1, data.number_of_cities):
        e[city] = max(
            data.truck_time_matrix[0][city],
            data.release_dates[city] + data.drone_time_matrix[0][city],
        )
    return e


def _route_completion_time(route: List[int], data: ProblemData, e: Dict[int, float]) -> float:
    """
    Completion time of one truck route under rule:
    truck can leave city i only when current_time >= e_i.
    """
    if not route:
        return 0.0

    current = 0.0
    prev = 0
    for city in route:
        current += data.truck_time_matrix[prev][city]
        if current < e[city]:
            current = e[city]
        prev = city
    current += data.truck_time_matrix[prev][0]
    return current


def phase1_truck_completion_times(
    routes: List[List[int]],
    data: ProblemData,
    e: Dict[int, float] | None = None,
) -> List[float]:
    e_values = e if e is not None else compute_phase1_earliest_departure(data)
    return [_route_completion_time(route, data, e_values) for route in routes]


def phase1_makespan(
    routes: List[List[int]],
    data: ProblemData,
    e: Dict[int, float] | None = None,
) -> float:
    if not routes:
        return 0.0
    return max(phase1_truck_completion_times(routes, data, e))


def _two_opt_route(route: List[int], i: int, k: int) -> List[int]:
    return route[:i] + list(reversed(route[i : k + 1])) + route[k + 1 :]


def phase1_local_search_2opt(
    routes: List[List[int]],
    data: ProblemData,
    e: Dict[int, float] | None = None,
    max_iterations: int | None = None,
) -> List[List[int]]:
    """
    Local search with 2-opt on truck routes.
    Stop when no improvement or iterations reach number of customers.
    Objective and e_i constraints are unchanged from phase 1.
    """
    if not routes:
        return routes

    e_values = e if e is not None else compute_phase1_earliest_departure(data)
    current_routes = [route[:] for route in routes]
    current_completion = phase1_truck_completion_times(current_routes, data, e_values)
    current_makespan = max(current_completion) if current_completion else 0.0
    current_sum = sum(current_completion)

    n_customers = max(0, data.number_of_cities - 1)
    limit = n_customers if max_iterations is None else max_iterations

    iteration = 0
    while iteration < limit:
        iteration += 1
        best_move = None
        best_makespan = current_makespan
        best_sum = current_sum
        best_new_time = None

        for truck_idx, route in enumerate(current_routes):
            if len(route) < 2:
                continue

            max_other = max(
                (current_completion[j] for j in range(len(current_routes)) if j != truck_idx),
                default=0.0,
            )
            sum_other = current_sum - current_completion[truck_idx]

            for i in range(len(route) - 1):
                for k in range(i + 1, len(route)):
                    new_route = _two_opt_route(route, i, k)
                    if new_route == route:
                        continue

                    new_time = _route_completion_time(new_route, data, e_values)
                    cand_makespan = max(max_other, new_time)
                    cand_sum = sum_other + new_time

                    better = False
                    if cand_makespan < best_makespan - EPS:
                        better = True
                    elif abs(cand_makespan - best_makespan) <= EPS and cand_sum < best_sum - EPS:
                        better = True
                    elif (
                        abs(cand_makespan - best_makespan) <= EPS
                        and abs(cand_sum - best_sum) <= EPS
                        and best_new_time is not None
                        and new_time < best_new_time - EPS
                    ):
                        better = True

                    if better:
                        best_move = (truck_idx, new_route, new_time)
                        best_makespan = cand_makespan
                        best_sum = cand_sum
                        best_new_time = new_time

        if best_move is None:
            break

        truck_idx, new_route, new_time = best_move
        current_routes[truck_idx] = new_route
        current_completion[truck_idx] = new_time
        current_makespan = max(current_completion) if current_completion else 0.0
        current_sum = sum(current_completion)

    return current_routes


def build_phase1_truck_routes(data: ProblemData) -> Tuple[List[List[int]], Dict[int, float]]:
    """
    Phase 1 heuristic:
    1) Seed each truck with one customer having largest e_i (descending).
    2) Insert remaining customers by global best insertion that minimizes
       team makespan under e_i departure constraints.
    3) Refine by 2-opt local search (max n iterations, n = number of customers).
    """
    number_truck = max(1, data.number_truck)
    customers = list(range(1, data.number_of_cities))
    e = compute_phase1_earliest_departure(data)

    customers.sort(key=lambda c: e[c], reverse=True)

    routes: List[List[int]] = [[] for _ in range(number_truck)]
    seed_count = min(number_truck, len(customers))
    for truck_idx in range(seed_count):
        routes[truck_idx].append(customers[truck_idx])

    remaining = customers[seed_count:]
    completion = phase1_truck_completion_times(routes, data, e)

    while remaining:
        best_key = None
        best_choice = None

        for customer in remaining:
            for truck_idx in range(number_truck):
                route = routes[truck_idx]
                max_other = max(
                    (completion[j] for j in range(number_truck) if j != truck_idx),
                    default=0.0,
                )
                sum_other = sum(completion) - completion[truck_idx]

                for pos in range(len(route) + 1):
                    new_route = route[:pos] + [customer] + route[pos:]
                    new_time = _route_completion_time(new_route, data, e)
                    candidate_makespan = max(max_other, new_time)
                    candidate_sum = sum_other + new_time

                    key = (
                        candidate_makespan,
                        candidate_sum,
                        new_time,
                        len(new_route),
                        truck_idx,
                        pos,
                        customer,
                    )
                    if best_key is None or key < best_key:
                        best_key = key
                        best_choice = (customer, truck_idx, pos, new_time)

        customer, truck_idx, pos, new_time = best_choice
        routes[truck_idx].insert(pos, customer)
        completion[truck_idx] = new_time
        remaining.remove(customer)

    routes = phase1_local_search_2opt(
        routes,
        data,
        e,
        max_iterations=max(0, data.number_of_cities - 1),
    )

    return routes, e


def build_initial_solution_phase1(data: ProblemData) -> Solution:
    """
    Build a truck-only initial solution:
    - depot stop keeps all route customers as depot-loaded packages
    - each customer stop has no pending drone package yet
    - drone queue is empty
    """
    routes, _ = build_phase1_truck_routes(data)

    truck_routes: List[TruckRoute] = []
    for route in routes:
        stops = [TruckStop(city=0, drone_customers=route[:])]
        for city in route:
            stops.append(TruckStop(city=city, drone_customers=[]))
        truck_routes.append(TruckRoute(stops=stops))

    return Solution(truck_routes=truck_routes, drone_queue=[])


def build_initial_solution_phase1_legacy(data: ProblemData) -> List[Any]:
    return build_initial_solution_phase1(data).to_legacy()


def build_initial_solution(
    data: ProblemData,
    apply_drone_local_search: bool = True,
    drone_ls_iterations: int | None = None,
    drone_ls_max_neighbors: int | None = 120,
    prefer_nonempty_drone: bool = True,
    nonempty_makespan_tolerance: float = float("inf"),
) -> Solution:
    """
    Build initial solution in the same high-level order as test_similarity.py:
    1) Build truck-first solution (phase 1).
    2) Then improve drone plan by local search over drone queue.
    """
    phase1_solution = build_initial_solution_phase1(data)
    solution = phase1_solution

    if apply_drone_local_search and data.number_drone > 0:
        solution = local_search_drone(
            phase1_solution,
            data,
            max_iterations=drone_ls_iterations,
            max_neighbors=drone_ls_max_neighbors,
        )

        if prefer_nonempty_drone and not solution.drone_queue:
            # Optional fallback: keep a drone-seeded solution even if makespan is
            # slightly worse (bounded by tolerance), useful for downstream search.
            base_eval = evaluate_fitness(phase1_solution, data)
            if not base_eval.feasible:
                return solution
            budget = base_eval.objective + max(0.0, nonempty_makespan_tolerance)
            current = Solution.from_legacy(phase1_solution.to_legacy())
            best_nonempty = None
            best_key = None

            walk_steps = data.number_of_cities - 1 if drone_ls_iterations is None else max(1, drone_ls_iterations)
            for _ in range(max(1, walk_steps)):
                neighbors = generate_drone_neighbors(current, data, max_neighbors=drone_ls_max_neighbors)
                if not neighbors:
                    break
                chosen = neighbors[0].solution
                current = Solution.from_legacy(chosen.to_legacy())
                if not current.drone_queue:
                    continue
                fit_eval = evaluate_fitness(current, data)
                if not fit_eval.feasible:
                    continue
                if fit_eval.objective <= budget + EPS:
                    key = (fit_eval.objective, len(current.drone_queue))
                    if best_key is None or key < best_key:
                        best_key = key
                        best_nonempty = Solution.from_legacy(current.to_legacy())

            if best_nonempty is not None:
                solution = best_nonempty

    return solution


def build_initial_solution_legacy(
    data: ProblemData,
    apply_drone_local_search: bool = True,
    drone_ls_iterations: int | None = None,
    drone_ls_max_neighbors: int | None = 120,
    prefer_nonempty_drone: bool = True,
    nonempty_makespan_tolerance: float = float("inf"),
) -> List[Any]:
    return build_initial_solution(
        data,
        apply_drone_local_search=apply_drone_local_search,
        drone_ls_iterations=drone_ls_iterations,
        drone_ls_max_neighbors=drone_ls_max_neighbors,
        prefer_nonempty_drone=prefer_nonempty_drone,
        nonempty_makespan_tolerance=nonempty_makespan_tolerance,
    ).to_legacy()
