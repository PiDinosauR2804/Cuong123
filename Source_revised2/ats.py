from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from .drone_local_search import local_search_drone
from .fitness import FitnessResult, evaluate_fitness
from .init_solution import build_initial_solution
from .problem_data import ProblemData
from .solution import Solution
from .truck_local_search import (
    TruckNeighbor,
    diversification_truck,
    generate_truck_neighbors_move_1_0,
    generate_truck_neighbors_move_1_1,
    generate_truck_neighbors_move_2_0,
    generate_truck_neighbors_move_2_1,
    generate_truck_neighbors_move_2_opt,
)


EPS = 1e-9


@dataclass
class AtsParams:
    # Stop inner loop after nimp consecutive non-improving iterations.
    nimp: int = 30
    # Number of consecutive non-improving segments before diversification.
    seg: int = 8
    # Number of consecutive diversification rounds without p_best improvement to stop.
    div: int = 3

    # Adaptive weights parameters (as in ATS paper-style formulation).
    gamma1: float = 0.5
    gamma2: float = 0.3
    gamma3: float = 0.1
    gamma4: float = 0.3

    seed: Optional[int] = None

    # Neighborhood size caps.
    truck_max_neighbors: Optional[int] = 300
    drone_max_neighbors: Optional[int] = 120

    # Apply drone LS after selected truck neighbor.
    use_drone_refine: bool = True
    drone_ls_iterations: Optional[int] = None

    # Diversification controls.
    diversification_max_neighbors: Optional[int] = 200
    diversification_max_moves_per_route: Optional[int] = 80
    diversification_accept_non_improving: bool = True

    # Optional hard stop by number of ATS segments.
    max_segments: Optional[int] = None

    # Optional hard stop by wall-clock runtime (seconds).
    max_runtime_sec: Optional[float] = None


@dataclass
class AtsResult:
    best_solution: Solution
    best_eval: FitnessResult
    current_solution: Solution
    current_eval: FitnessResult
    initial_solution: Solution
    initial_eval: FitnessResult
    best_multi_visit_solution: Optional[Solution]
    best_multi_visit_eval: Optional[FitnessResult]
    segments_run: int
    diversification_rounds: int
    weights: Dict[str, float] = field(default_factory=dict)
    segment_trace: List[Dict[str, Any]] = field(default_factory=list)
    time_limit_reached: bool = False
    elapsed_sec: float = 0.0


def _to_solution(solution: Solution | Any) -> Solution:
    if isinstance(solution, Solution):
        return Solution.from_legacy(solution.to_legacy())
    return Solution.from_legacy(solution)


def _is_better(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    if a[0] < b[0] - EPS:
        return True
    if abs(a[0] - b[0]) <= EPS and a[1] < b[1] - EPS:
        return True
    return False


def _key_from_eval(ev: FitnessResult) -> Tuple[float, float]:
    return ev.objective, sum(ev.truck_return_time.values())


def _roulette_select(items: List[str], weights: Dict[str, float], rng: random.Random) -> str:
    total = sum(max(0.0, weights.get(it, 0.0)) for it in items)
    if total <= EPS:
        return rng.choice(items)
    x = rng.random() * total
    acc = 0.0
    for it in items:
        acc += max(0.0, weights.get(it, 0.0))
        if acc >= x:
            return it
    return items[-1]


def _neighbor_tabu_key(nei: TruckNeighbor) -> Tuple[str, Tuple[int, ...]]:
    moved = [int(c) for c in nei.moved_customers]
    if nei.neighborhood == "(1,0)":
        key = tuple(moved[:1])
    elif nei.neighborhood in ("(1,1)", "(2-opt)"):
        key = tuple(sorted(moved[:2]))
    elif nei.neighborhood in ("(2,0)", "(2,1)"):
        key = tuple(sorted(moved[:3]))
    else:
        key = tuple(sorted(moved[:4]))
    return nei.neighborhood, key


def _is_tabu(
    nei: TruckNeighbor,
    tabu: Dict[str, Dict[Tuple[int, ...], int]],
    iteration: int,
    best_eval: FitnessResult,
) -> bool:
    category, key = _neighbor_tabu_key(nei)
    expire = tabu.get(category, {}).get(key, -1)
    aspiration = nei.objective < best_eval.objective - EPS
    return expire > iteration and not aspiration


def _mark_tabu(
    nei: TruckNeighbor,
    tabu: Dict[str, Dict[Tuple[int, ...], int]],
    iteration: int,
    tenure: int,
) -> None:
    category, key = _neighbor_tabu_key(nei)
    tabu.setdefault(category, {})[key] = iteration + tenure


def adaptive_tabu_search(
    data: ProblemData,
    params: Optional[AtsParams] = None,
    initial_solution: Solution | Any | None = None,
    verbose: bool = False,
) -> AtsResult:
    """
    ATS structure (paper-style):
    1) Build initial solution.
    2) Iterate by adaptive roulette neighborhood selection with tabu memory.
    3) Segment-based stagnation detection and diversification.
    4) Keep global best solution p_best.
    """
    cfg = params if params is not None else AtsParams()

    rng = random.Random(cfg.seed)
    if cfg.seed is not None:
        random.seed(cfg.seed)

    if initial_solution is None:
        current = build_initial_solution(
            data,
            apply_drone_local_search=cfg.use_drone_refine,
            drone_ls_iterations=cfg.drone_ls_iterations,
            drone_ls_max_neighbors=cfg.drone_max_neighbors,
        )
    else:
        current = _to_solution(initial_solution)
        if cfg.use_drone_refine:
            current = local_search_drone(
                current,
                data,
                max_iterations=cfg.drone_ls_iterations,
                max_neighbors=cfg.drone_max_neighbors,
            )

    current_eval = evaluate_fitness(current, data)
    if not current_eval.feasible:
        # Safe fallback to standard initializer if caller passes an infeasible seed.
        current = build_initial_solution(
            data,
            apply_drone_local_search=cfg.use_drone_refine,
            drone_ls_iterations=cfg.drone_ls_iterations,
            drone_ls_max_neighbors=cfg.drone_max_neighbors,
        )
        current_eval = evaluate_fitness(current, data)

    if not current_eval.feasible:
        raise RuntimeError("ATS cannot start from a feasible initial solution.")

    initial = Solution.from_legacy(current.to_legacy())
    initial_eval = current_eval

    best = Solution.from_legacy(current.to_legacy())
    best_eval = current_eval
    best_multi_visit_solution: Optional[Solution] = None
    best_multi_visit_eval: Optional[FitnessResult] = None

    neighborhood_generators: Dict[str, Callable[..., List[TruckNeighbor]]] = {
        "(1,0)": generate_truck_neighbors_move_1_0,
        "(1,1)": generate_truck_neighbors_move_1_1,
        "(2,0)": generate_truck_neighbors_move_2_0,
        "(2,1)": generate_truck_neighbors_move_2_1,
        "(2-opt)": generate_truck_neighbors_move_2_opt,
    }
    neighborhoods = list(neighborhood_generators.keys())
    weights: Dict[str, float] = {n: 1.0 / len(neighborhoods) for n in neighborhoods}

    no_improve_segments = 0
    no_improve_div = 0
    segments_run = 0
    diversification_rounds = 0
    segment_trace: List[Dict[str, Any]] = []
    started_at = time.perf_counter()
    time_limit_reached = False

    def _is_timeout() -> bool:
        if cfg.max_runtime_sec is None:
            return False
        if cfg.max_runtime_sec <= 0:
            return False
        return (time.perf_counter() - started_at) >= float(cfg.max_runtime_sec)

    def _routes_of(sol: Solution) -> List[List[int]]:
        return [[stop.city for stop in route.stops if stop.city != 0] for route in sol.truck_routes]

    def _queue_of(sol: Solution) -> List[List[List[Any]]]:
        out: List[List[List[Any]]] = []
        for trip in sol.drone_queue:
            legs: List[List[Any]] = []
            for leg in trip.legs:
                legs.append([leg.launch_city, list(leg.customers)])
            out.append(legs)
        return out

    def _multi_visit_trip_count(sol: Solution) -> int:
        return sum(1 for trip in sol.drone_queue if trip.is_multi_visit)

    def _update_best_multi_visit(sol: Solution, ev: FitnessResult) -> None:
        nonlocal best_multi_visit_solution, best_multi_visit_eval
        if not ev.feasible:
            return
        if not sol.has_multi_visit_trip():
            return
        if (
            best_multi_visit_eval is None
            or _is_better(_key_from_eval(ev), _key_from_eval(best_multi_visit_eval))
        ):
            best_multi_visit_solution = Solution.from_legacy(sol.to_legacy())
            best_multi_visit_eval = ev

    _update_best_multi_visit(current, current_eval)

    while no_improve_div < cfg.div:
        if _is_timeout():
            time_limit_reached = True
            break
        if cfg.max_segments is not None and segments_run >= cfg.max_segments:
            break
        segments_run += 1
        if verbose:
            print(
                f"[ATS] segment={segments_run} p_best={best_eval.objective:.3f} "
                f"stall_seg={no_improve_segments} stall_div={no_improve_div}"
            )

        score: Dict[str, float] = {n: 0.0 for n in neighborhoods}
        used: Dict[str, int] = {n: 0 for n in neighborhoods}
        tenure = int(rng.uniform(2.0 * math.log(max(2, data.number_of_cities)), max(2, data.number_of_cities)))
        tenure = max(1, tenure)
        tabu: Dict[str, Dict[Tuple[int, ...], int]] = {n: {} for n in neighborhoods}

        no_improve_iter = 0
        iteration = 0
        best_before_segment = _key_from_eval(best_eval)

        while no_improve_iter < cfg.nimp:
            if _is_timeout():
                time_limit_reached = True
                break
            chosen = _roulette_select(neighborhoods, weights, rng)
            generator = neighborhood_generators[chosen]
            cand_neighbors = generator(current, data, max_neighbors=cfg.truck_max_neighbors)

            admissible = [n for n in cand_neighbors if not _is_tabu(n, tabu, iteration, best_eval)]
            if not admissible:
                no_improve_iter += 1
                iteration += 1
                continue

            selected = min(admissible, key=lambda n: (n.objective, n.truck_completion_sum, n.move_detail))
            candidate = Solution.from_legacy(selected.solution.to_legacy())

            if cfg.use_drone_refine:
                candidate = local_search_drone(
                    candidate,
                    data,
                    max_iterations=cfg.drone_ls_iterations,
                    max_neighbors=cfg.drone_max_neighbors,
                )

            cand_eval = evaluate_fitness(candidate, data)
            if not cand_eval.feasible:
                no_improve_iter += 1
                iteration += 1
                continue

            prev_current_key = _key_from_eval(current_eval)
            cand_key = _key_from_eval(cand_eval)
            best_key = _key_from_eval(best_eval)

            if _is_better(cand_key, best_key):
                best = Solution.from_legacy(candidate.to_legacy())
                best_eval = cand_eval
                score[chosen] += cfg.gamma1
                no_improve_iter = 0
            elif _is_better(cand_key, prev_current_key):
                score[chosen] += cfg.gamma2
                no_improve_iter += 1
            else:
                score[chosen] += cfg.gamma3
                no_improve_iter += 1

            current = candidate
            current_eval = cand_eval
            _update_best_multi_visit(current, current_eval)
            used[chosen] += 1
            _mark_tabu(selected, tabu, iteration, tenure)
            iteration += 1

        if time_limit_reached:
            break
        if _is_timeout():
            time_limit_reached = True
            break

        for n in neighborhoods:
            if used[n] > 0:
                weights[n] = (1.0 - cfg.gamma4) * weights[n] + cfg.gamma4 * (score[n] / used[n])

        if _is_better(_key_from_eval(best_eval), best_before_segment):
            no_improve_segments = 0
        else:
            no_improve_segments += 1

        if no_improve_segments < cfg.seg:
            segment_trace.append(
                {
                    "segment": segments_run,
                    "stage": "post_segment",
                    "current_objective": current_eval.objective,
                    "best_objective": best_eval.objective,
                    "current_truck_sum": sum(current_eval.truck_return_time.values()),
                    "best_truck_sum": sum(best_eval.truck_return_time.values()),
                    "current_routes": _routes_of(current),
                    "current_queue": _queue_of(current),
                    "best_routes": _routes_of(best),
                    "best_queue": _queue_of(best),
                    "best_multi_visit_objective": (
                        None if best_multi_visit_eval is None else best_multi_visit_eval.objective
                    ),
                    "best_multi_visit_trip_count": (
                        0 if best_multi_visit_solution is None else _multi_visit_trip_count(best_multi_visit_solution)
                    ),
                    "weights": dict(weights),
                    "diversification_applied": False,
                    "diversification_improved_best": False,
                }
            )
            continue

        if _is_timeout():
            time_limit_reached = True
            break

        diversification_rounds += 1
        diversified, _ = diversification_truck(
            current,
            data,
            max_neighbors=cfg.diversification_max_neighbors,
            rng=rng,
            max_moves_per_route=cfg.diversification_max_moves_per_route,
            accept_non_improving=cfg.diversification_accept_non_improving,
        )
        if cfg.use_drone_refine:
            diversified = local_search_drone(
                diversified,
                data,
                max_iterations=cfg.drone_ls_iterations,
                max_neighbors=cfg.drone_max_neighbors,
            )
        div_eval = evaluate_fitness(diversified, data)

        pbest_improved_by_div = False
        if div_eval.feasible:
            current = diversified
            current_eval = div_eval
            _update_best_multi_visit(current, current_eval)
            if _is_better(_key_from_eval(div_eval), _key_from_eval(best_eval)):
                best = Solution.from_legacy(diversified.to_legacy())
                best_eval = div_eval
                pbest_improved_by_div = True

        if pbest_improved_by_div:
            no_improve_div = 0
        else:
            no_improve_div += 1
        no_improve_segments = 0
        segment_trace.append(
            {
                "segment": segments_run,
                "stage": "post_diversification",
                "current_objective": current_eval.objective,
                "best_objective": best_eval.objective,
                "current_truck_sum": sum(current_eval.truck_return_time.values()),
                "best_truck_sum": sum(best_eval.truck_return_time.values()),
                "current_routes": _routes_of(current),
                "current_queue": _queue_of(current),
                "best_routes": _routes_of(best),
                "best_queue": _queue_of(best),
                "best_multi_visit_objective": (
                    None if best_multi_visit_eval is None else best_multi_visit_eval.objective
                ),
                "best_multi_visit_trip_count": (
                    0 if best_multi_visit_solution is None else _multi_visit_trip_count(best_multi_visit_solution)
                ),
                "weights": dict(weights),
                "diversification_applied": True,
                "diversification_improved_best": pbest_improved_by_div,
            }
        )

    return AtsResult(
        best_solution=best,
        best_eval=best_eval,
        current_solution=current,
        current_eval=current_eval,
        initial_solution=initial,
        initial_eval=initial_eval,
        best_multi_visit_solution=best_multi_visit_solution,
        best_multi_visit_eval=best_multi_visit_eval,
        segments_run=segments_run,
        diversification_rounds=diversification_rounds,
        weights=weights,
        segment_trace=segment_trace,
        time_limit_reached=time_limit_reached,
        elapsed_sec=(time.perf_counter() - started_at),
    )


def adaptive_tabu_search_legacy(
    data: ProblemData,
    params: Optional[AtsParams] = None,
    initial_solution: Solution | Any | None = None,
    verbose: bool = False,
) -> Tuple[List[Any], float]:
    result = adaptive_tabu_search(
        data=data,
        params=params,
        initial_solution=initial_solution,
        verbose=verbose,
    )
    return result.best_solution.to_legacy(), result.best_eval.objective
