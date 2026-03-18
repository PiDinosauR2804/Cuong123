from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# Running through VS Code wrappers can lose the workspace root on sys.path.
PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Source_revised import (
    best_merge_two_trips_multi_visit,
    build_initial_solution_phase1,
    evaluate_solution,
    local_search_phase2,
    read_dat,
    repair_solution_from_truck_routes,
    tabu_search_phase1,
)
from Source_revised.move_truck_with_drone import TruckDroneNeighbor, generate_truck_drone_neighbors

EPS = 1e-9

@dataclass
class AtsParams:
    nimp: int = 30
    seg: int = 8
    div: int = 3
    gamma1: float = 0.5
    gamma2: float = 0.3
    gamma3: float = 0.1
    gamma4: float = 0.3
    seed: Optional[int] = None
    use_eval_cache: bool = True

def roulette_select(items: List[str], weights: Dict[str, float], rng: random.Random) -> str:
    total = sum(max(0.0, weights[i]) for i in items)
    if total <= 0:
        return rng.choice(items)
    r = rng.random() * total
    acc = 0.0
    for it in items:
        acc += max(0.0, weights[it])
        if acc >= r:
            return it
    return items[-1]

def _neighbor_key(nei: TruckDroneNeighbor) -> Tuple[str, Tuple[int, ...]]:
    moved = nei.moved_customers
    if nei.neighborhood == "(1,0)":
        key = tuple(moved[:1])
    elif nei.neighborhood in ("(1,1)", "2-opt"):
        key = tuple(sorted(moved[:2]))
    elif nei.neighborhood == "(2,1)":
        key = tuple(sorted(moved[:3]))
    else:
        key = tuple(sorted(moved))
    return nei.neighborhood, key

def _is_tabu(
    nei: TruckDroneNeighbor,
    tabu: Dict[str, Dict[Tuple[int, ...], int]],
    it: int,
    best_fitness: float,
) -> bool:
    cat, key = _neighbor_key(nei)
    expire = tabu.get(cat, {}).get(key, -1)
    aspiration = nei.fitness + EPS < best_fitness
    return expire > it and not aspiration

def _mark_tabu(
    nei: TruckDroneNeighbor,
    tabu: Dict[str, Dict[Tuple[int, ...], int]],
    it: int,
    tenure: int,
) -> None:
    cat, key = _neighbor_key(nei)
    tabu.setdefault(cat, {})[key] = it + tenure

def _truck_routes(raw_solution: Any) -> List[List[int]]:
    trucks = raw_solution[0]
    routes: List[List[int]] = []
    for route in trucks:
        routes.append([stop[0] for stop in route[1:]])
    return routes

def diversification(raw_solution: Any, data: Any, rng: random.Random) -> Tuple[Any, bool]:
    routes = _truck_routes(raw_solution)

    base_eval = evaluate_solution(raw_solution, data)
    if not base_eval.feasible:
        return raw_solution, False
    best_sol = raw_solution
    best_key = (base_eval.system_completion_time, sum(base_eval.truck_time.values()))
    best_multi_visit_count = base_eval.multi_visit_trip_count

    improved = False
    for t_idx, route in enumerate(routes):
        x = len(route)
        if x < 4:
            continue
        max_len = x // 2
        r = rng.randint(2, max_len)
        for seg_len in range(r, max_len + 1):
            for start in range(0, x - seg_len + 1):
                end = start + seg_len
                new_routes = [rt[:] for rt in routes]
                new_routes[t_idx][start:end] = reversed(new_routes[t_idx][start:end])
                cand = repair_solution_from_truck_routes(raw_solution, new_routes, data)
                if cand is None:
                    continue
                ev = evaluate_solution(cand, data)
                if not ev.feasible:
                    continue
                key = (ev.system_completion_time, sum(ev.truck_time.values()))
                if key < best_key:
                    best_key = key
                    best_multi_visit_count = ev.multi_visit_trip_count
                    best_sol = cand
                    improved = True

    # Multi-visit diversification: merge 2 trips into a feasible multi-leg trip.
    merged = best_merge_two_trips_multi_visit(best_sol, data)
    if merged is not None:
        merged_eval = evaluate_solution(merged, data)
        if merged_eval.feasible:
            merged_key = (merged_eval.system_completion_time, sum(merged_eval.truck_time.values()))
            if merged_key < best_key:
                best_key = merged_key
                best_multi_visit_count = merged_eval.multi_visit_trip_count
                best_sol = merged
                improved = True
            elif merged_key == best_key and merged_eval.multi_visit_trip_count > best_multi_visit_count:
                best_multi_visit_count = merged_eval.multi_visit_trip_count
                best_sol = merged
                improved = True
    return best_sol, improved

def adaptive_tabu_search(
    instance_path: str,
    params: AtsParams,
    data_override: Any = None,
    verbose: bool = True,
) -> Tuple[Any, float, Any, Any]:
    rng = random.Random(params.seed)
    if params.seed is not None:
        random.seed(params.seed)
    data = data_override if data_override is not None else read_dat(instance_path)
    p = build_initial_solution_phase1(data)
    if verbose:
        print("Initial solution build_initial_solution_phase1.")
        print(p)
    p = tabu_search_phase1(p, data)
    if verbose:
        print("Initial solution tabu_search_phase1.")
        print(p)
    p = local_search_phase2(p, data)
    if verbose:
        print("Initial solution local_search_phase2.")
        print(p)
    p_eval = evaluate_solution(p, data)
    if not p_eval.feasible:
        raise RuntimeError("Initial solution is infeasible")
    p_best = p
    p_best_eval = p_eval
    neighborhoods = ["(1,0)", "(1,1)", "2-opt", "(2,1)"]
    weights = {n: 1.0 / len(neighborhoods) for n in neighborhoods}
    no_improve_segments = 0
    no_improve_div = 0
    best_multi_visit_sol = None
    best_multi_visit_eval = None
    def is_multi_visit(eval_result):
        return getattr(eval_result, "multi_visit_trip_count", 0) > 0
    if is_multi_visit(p_eval):
        best_multi_visit_sol = p
        best_multi_visit_eval = p_eval
    while True:
        if verbose:
            print(
                f"Segment {no_improve_segments}, Diversification no-pbest-improve count {no_improve_div}, current best fit {p_best_eval.system_completion_time}"
            )
        if no_improve_div >= params.div:
            break
        score = {n: 0.0 for n in neighborhoods}
        used = {n: 0 for n in neighborhoods}
        tenure = int(rng.uniform(2 * math.log(max(2, data.number_of_cities)), max(2, data.number_of_cities)))
        tabu: Dict[str, Dict[Tuple[int, ...], int]] = {n: {} for n in neighborhoods}
        no_improve_iter = 0
        it = 0
        best_before_segment = p_best_eval.system_completion_time
        while no_improve_iter < params.nimp:
            chosen = roulette_select(neighborhoods, weights, rng)
            cand_neighbors = generate_truck_drone_neighbors(
                p,
                data,
                use_eval_cache=params.use_eval_cache,
                neighborhood_filter=[chosen],
            )
            if not cand_neighbors:
                no_improve_iter += 1
                it += 1
                continue
            feasible_non_tabu = [
                n for n in cand_neighbors if not _is_tabu(n, tabu, it, p_best_eval.system_completion_time)
            ]
            if not feasible_non_tabu:
                no_improve_iter += 1
                it += 1
                continue
            p_prime_nei = min(feasible_non_tabu, key=lambda n: (n.fitness, n.truck_completion_sum))
            p_prime = local_search_phase2(p_prime_nei.solution, data)
            p_prime_eval = evaluate_solution(p_prime, data)
            if not p_prime_eval.feasible:
                no_improve_iter += 1
                it += 1
                continue
            if is_multi_visit(p_prime_eval):
                if (
                    best_multi_visit_eval is None
                    or p_prime_eval.system_completion_time < best_multi_visit_eval.system_completion_time
                ):
                    best_multi_visit_sol = p_prime
                    best_multi_visit_eval = p_prime_eval
            prev_current_fit = p_eval.system_completion_time
            if p_prime_eval.system_completion_time + EPS < p_best_eval.system_completion_time:
                p_best = p_prime
                p_best_eval = p_prime_eval
                no_improve_iter = 0
                score[chosen] += params.gamma1
            elif p_prime_eval.system_completion_time + EPS < prev_current_fit:
                score[chosen] += params.gamma2
                no_improve_iter += 1
            else:
                score[chosen] += params.gamma3
                no_improve_iter += 1
            p = p_prime
            p_eval = p_prime_eval
            used[chosen] += 1
            _mark_tabu(p_prime_nei, tabu, it, tenure)
            it += 1
        for n in neighborhoods:
            if used[n] > 0:
                weights[n] = (1.0 - params.gamma4) * weights[n] + params.gamma4 * (score[n] / used[n])
        if p_best_eval.system_completion_time + EPS < best_before_segment:
            no_improve_segments = 0
        else:
            no_improve_segments += 1
        if no_improve_segments < params.seg:
            continue
        diversified, improved = diversification(p, data, rng)
        div_eval = evaluate_solution(diversified, data)
        pbest_improved_by_div = False
        if improved and div_eval.feasible:
            p = diversified
            p_eval = div_eval
            if div_eval.system_completion_time + EPS < p_best_eval.system_completion_time:
                p_best = diversified
                p_best_eval = div_eval
                pbest_improved_by_div = True
            if is_multi_visit(div_eval):
                if (
                    best_multi_visit_eval is None
                    or div_eval.system_completion_time < best_multi_visit_eval.system_completion_time
                ):
                    best_multi_visit_sol = diversified
                    best_multi_visit_eval = div_eval
        if pbest_improved_by_div:
            no_improve_div = 0
        else:
            no_improve_div += 1
        no_improve_segments = 0
    return p_best, p_best_eval.system_completion_time, best_multi_visit_sol, best_multi_visit_eval

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Adaptive Tabu Search for truck-drone resupply")
    parser.add_argument("--instance", required=True, help="Path to .dat instance")
    parser.add_argument("--nimp", type=int, default=60)
    parser.add_argument("--seg", type=int, default=4)
    parser.add_argument("--div", type=int, default=3)
    parser.add_argument("--gamma1", type=float, default=0.5)
    parser.add_argument("--gamma2", type=float, default=0.3)
    parser.add_argument("--gamma3", type=float, default=0.1)
    parser.add_argument("--gamma4", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--A", type=float, default=None, help="Override drone capacity (optional)")
    parser.add_argument("--L_d", type=float, default=None, help="Override drone limit time (optional)")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    # Always use cache, remove option
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.seed is not None:
        # Keep global random calls in Source_revised deterministic across runs.
        random.seed(args.seed)
    params = AtsParams(
        nimp=args.nimp,
        seg=args.seg,
        div=args.div,
        gamma1=args.gamma1,
        gamma2=args.gamma2,
        gamma3=args.gamma3,
        gamma4=args.gamma4,
        seed=args.seed,
        use_eval_cache=True,  # Always use cache
    )

    # Read data and optionally override drone capacity / endurance.
    data = read_dat(args.instance)
    if args.A is not None:
        data.drone_capacity = args.A
    if args.L_d is not None:
        data.drone_limit_time = args.L_d

    best_sol, best_fit, best_multi_visit_sol, best_multi_visit_eval = adaptive_tabu_search(
        args.instance,
        params,
        data_override=data,
        verbose=args.verbose,
    )
    print("Best fitness:", best_fit)
    print("Best solution:", best_sol)
    if best_multi_visit_sol is not None:
        print("Best multi visit solution found:")
        print("  multi_visit_trip_count:", best_multi_visit_eval.multi_visit_trip_count)
        print("  system_completion_time:", best_multi_visit_eval.system_completion_time)
        print("  solution:", best_multi_visit_sol)
    else:
        print("No multi visit solution was found during search.")

if __name__ == "__main__":
    main()
