from __future__ import annotations

import argparse
import ast
import copy
import csv
import math
import os
import random
import shutil
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import Data
import Function
import Neighborhood
import Neighborhood10
import Neighborhood11
from Source_revised2 import (
    Solution,
    build_initial_solution,
    evaluate_fitness,
    local_search_drone,
    read_data_file,
    repair_solution_after_truck_routes,
)


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT = PROJECT_ROOT / "50_batch.csv"
DEFAULT_OUTPUT = PROJECT_ROOT / "50_batch_repaired.csv"
DEFAULT_BACKUP = PROJECT_ROOT / "50_batch.csv.bak_recalc_obj"
DEFAULT_COMPARE = PROJECT_ROOT / "50_batch_compare.csv"


def _parse_legacy_solution(raw: str) -> Optional[Solution]:
    text = (raw or "").strip()
    if not text or text.lower() in {"none", "nan", "null"}:
        return None
    try:
        parsed = ast.literal_eval(text)
        return Solution.from_legacy(parsed)
    except Exception:
        return None


def _route_lists(solution: Solution) -> List[List[int]]:
    return [[stop.city for stop in route.stops if stop.city != 0] for route in solution.truck_routes]


def _float_or_none(raw: str) -> Optional[float]:
    text = (raw or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except Exception:
        return None


def _fmt_float(value: Optional[float]) -> str:
    if value is None:
        return ""
    return f"{value:.7f}".rstrip("0").rstrip(".")


def _seed_from_row(row: Dict[str, str], row_idx: int) -> int:
    run_text = (row.get("run") or "").strip()
    try:
        return int(float(run_text))
    except Exception:
        return row_idx


def _resolve_instance_path(row: Dict[str, str]) -> Path:
    raw = (row.get("instance") or "").strip()
    if not raw:
        raise FileNotFoundError("Missing 'instance' path in CSV row.")
    normalized = raw.replace("\\", "/")
    p = Path(normalized)
    candidate = p if p.is_absolute() else PROJECT_ROOT / normalized
    resolved = candidate.resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Instance file not found: {candidate}")
    return resolved


def _effective_workers(requested_workers: int, total_rows: int) -> int:
    if total_rows <= 1:
        return 1
    cpu_count = max(1, os.cpu_count() or 1)
    requested = max(1, int(requested_workers))
    return max(1, min(requested, cpu_count, total_rows))


def _error_result(idx: int, row: Dict[str, str], exc: Exception) -> Tuple[Dict[str, str], Dict[str, Any]]:
    updated = dict(row)
    compare = {
        "row_index": idx,
        "instance": row.get("instance", ""),
        "run": row.get("run", ""),
        "old_best_fitness": row.get("best_fitness", ""),
        "old_best_multi_visit_fitness": row.get("best_multi_visit_fitness", ""),
        "best_sol_initially_feasible": "",
        "best_sol_repair_mode": "",
        "best_multi_visit_initially_feasible": "",
        "best_multi_visit_repair_mode": "",
        "ats_engine": "",
        "ats_segments_run": "",
        "ats_loop_iterations": "",
        "ats_time_limit_reached": "",
        "ats_elapsed_sec": "",
        "new_best_fitness": row.get("best_fitness", ""),
        "new_best_multi_visit_fitness": row.get("best_multi_visit_fitness", ""),
        "best_multi_visit_source": "",
        "status": "ERROR",
        "error": f"{type(exc).__name__}: {exc}",
        "runtime_sec": "",
    }
    return updated, compare


def _repair_solution_for_rules(
    candidate: Optional[Solution],
    data,
    fallback_build_initial: bool,
) -> Tuple[Solution, Any, str, bool]:
    """
    Returns (solution, fitness_eval, repair_mode, was_initially_feasible)
    """
    if candidate is None:
        if not fallback_build_initial:
            raise ValueError("Candidate solution is missing.")
        fallback = build_initial_solution(
            data,
            apply_drone_local_search=True,
            drone_ls_iterations=None,
            drone_ls_max_neighbors=None,
        )
        fallback_eval = evaluate_fitness(fallback, data)
        if not fallback_eval.feasible:
            raise RuntimeError("Fallback initial solution is infeasible.")
        return fallback, fallback_eval, "fallback_build_initial_missing", False

    raw_eval = evaluate_fitness(candidate, data)
    if raw_eval.feasible:
        return candidate, raw_eval, "already_feasible", True

    repaired = repair_solution_after_truck_routes(candidate, _route_lists(candidate), data)
    if repaired is not None:
        repaired = local_search_drone(repaired, data, max_iterations=0, max_neighbors=None)
        repaired_eval = evaluate_fitness(repaired, data)
        if repaired_eval.feasible:
            return repaired, repaired_eval, "repair_solution_after_truck_routes", False

    quick_fix = local_search_drone(candidate, data, max_iterations=0, max_neighbors=None)
    quick_fix_eval = evaluate_fitness(quick_fix, data)
    if quick_fix_eval.feasible:
        return quick_fix, quick_fix_eval, "local_search_drone_0iter", False

    if not fallback_build_initial:
        raise RuntimeError("Cannot repair candidate solution to feasible state.")

    fallback = build_initial_solution(
        data,
        apply_drone_local_search=True,
        drone_ls_iterations=None,
        drone_ls_max_neighbors=None,
    )
    fallback_eval = evaluate_fitness(fallback, data)
    if not fallback_eval.feasible:
        raise RuntimeError("Fallback initial solution is infeasible.")
    return fallback, fallback_eval, "fallback_build_initial", False


def _choose_best_multi_visit(
    data,
    candidates: List[Tuple[Solution, Any, str]],
) -> Tuple[Optional[Solution], Optional[Any], str]:
    best_sol: Optional[Solution] = None
    best_eval: Optional[Any] = None
    best_source = ""

    for solution, fit_eval, source in candidates:
        if solution is None:
            continue
        if not solution.has_multi_visit_trip():
            continue

        ev = fit_eval if fit_eval is not None else evaluate_fitness(solution, data)
        if not ev.feasible:
            continue

        if best_eval is None:
            best_sol = solution
            best_eval = ev
            best_source = source
            continue

        current_key = (ev.objective, sum(ev.truck_return_time.values()))
        best_key = (best_eval.objective, sum(best_eval.truck_return_time.values()))
        if current_key < best_key:
            best_sol = solution
            best_eval = ev
            best_source = source

    return best_sol, best_eval, best_source


def _is_better_eval(candidate_eval: Any, incumbent_eval: Any) -> bool:
    candidate_key = (candidate_eval.objective, sum(candidate_eval.truck_return_time.values()))
    incumbent_key = (incumbent_eval.objective, sum(incumbent_eval.truck_return_time.values()))
    return candidate_key < incumbent_key


def _legacy_has_multi_visit(legacy_solution: Any) -> bool:
    if not isinstance(legacy_solution, list) or len(legacy_solution) < 2:
        return False
    queue = legacy_solution[1]
    if not isinstance(queue, list):
        return False
    return any(isinstance(trip, list) and len(trip) > 1 for trip in queue)


def _legacy_roulette_select(population: List[int], fitness_scores: List[float]) -> int:
    total = sum(max(0.0, float(x)) for x in fitness_scores)
    if total <= 0:
        return random.choice(population)
    pick = random.random() * total
    running = 0.0
    for idx, item in enumerate(population):
        running += max(0.0, float(fitness_scores[idx]))
        if running >= pick:
            return item
    return population[-1]


def _sync_legacy_data_for_instance(
    instance_path: Path,
    data,
    row: Dict[str, str],
    seed: int,
) -> None:
    Data.read_data_random(str(instance_path))
    Data.number_of_trucks = int(data.number_truck)
    Data.number_of_drones = int(data.number_drone)
    Data.drone_capacity = float(data.drone_capacity)
    Data.drone_limit_time = float(data.drone_limit_time)

    theta_text = (row.get("theta") or "").strip()
    if theta_text:
        try:
            Data.theta = int(float(theta_text))
        except Exception:
            pass

    random.seed(seed)


def _try_record_feasible_candidate(
    legacy_candidate: Any,
    data,
    best_solution: Solution,
    best_eval: Any,
    best_multi_visit_solution: Optional[Solution],
    best_multi_visit_eval: Optional[Any],
) -> Tuple[Solution, Any, Optional[Solution], Optional[Any]]:
    try:
        candidate_solution = Solution.from_legacy(legacy_candidate)
    except Exception:
        return best_solution, best_eval, best_multi_visit_solution, best_multi_visit_eval

    candidate_eval = evaluate_fitness(candidate_solution, data)
    if not candidate_eval.feasible:
        return best_solution, best_eval, best_multi_visit_solution, best_multi_visit_eval

    if _is_better_eval(candidate_eval, best_eval):
        best_solution = Solution.from_legacy(candidate_solution.to_legacy())
        best_eval = candidate_eval

    if candidate_solution.has_multi_visit_trip():
        if best_multi_visit_eval is None or _is_better_eval(candidate_eval, best_multi_visit_eval):
            best_multi_visit_solution = Solution.from_legacy(candidate_solution.to_legacy())
            best_multi_visit_eval = candidate_eval

    return best_solution, best_eval, best_multi_visit_solution, best_multi_visit_eval


def _run_test_similarity_segment_with_source2_feasibility(
    instance_path: Path,
    row: Dict[str, str],
    row_idx: int,
    initial_solution: Solution,
    data,
    max_runtime_sec: Optional[float],
) -> Dict[str, Any]:
    seed = _seed_from_row(row, row_idx)
    _sync_legacy_data_for_instance(instance_path, data, row, seed)

    initial_eval = evaluate_fitness(initial_solution, data)
    if not initial_eval.feasible:
        raise RuntimeError("Initial solution must be feasible before legacy ATS segment.")

    current_sol = copy.deepcopy(initial_solution.to_legacy())
    current_fitness, current_truck_time, current_sum_fitness = Function.fitness(current_sol)
    legacy_best_sol = copy.deepcopy(current_sol)
    legacy_best_fitness = current_fitness

    best_solution = Solution.from_legacy(initial_solution.to_legacy())
    best_eval = initial_eval
    best_multi_visit_solution: Optional[Solution] = None
    best_multi_visit_eval: Optional[Any] = None
    if best_solution.has_multi_visit_trip():
        best_multi_visit_solution = Solution.from_legacy(best_solution.to_legacy())
        best_multi_visit_eval = best_eval

    epsilon = (-1) * 0.00001
    solution_pack: List[Any] = []
    solution_pack_len = 0

    end_segment = int(Data.number_of_cities / math.log10(Data.number_of_cities)) * int(Data.theta)
    nei_set = [0, 1, 2, 3]
    weight = [1 / len(nei_set)] * len(nei_set)

    tabu_tenure = random.uniform(2 * math.log(Data.number_of_cities), Data.number_of_cities)
    tabu_tenure1 = tabu_tenure
    tabu_tenure2 = tabu_tenure
    tabu_tenure3 = tabu_tenure

    tabu_structure = [(-1) * (tabu_tenure + 1)] * Data.number_of_cities
    tabu_structure1 = [(-1) * (tabu_tenure + 1)] * Data.number_of_cities
    tabu_structure2 = [(-1) * (tabu_tenure + 1)] * Data.number_of_cities
    tabu_structure3 = [(-1) * (tabu_tenure + 1)] * Data.number_of_cities

    factor = Data.delta
    score = [0] * len(nei_set)
    used = [0] * len(nei_set)
    lenght_i = [0] * 6
    i = 0

    loop_iterations = 0
    started = time.time()
    time_limit_reached = False

    while i < end_segment:
        if max_runtime_sec is not None and (time.time() - started) >= max_runtime_sec:
            time_limit_reached = True
            break

        current_neighborhood = []
        prev_fitness = current_fitness
        choose = _legacy_roulette_select(nei_set, weight)

        if choose == 0:
            current_neighborhood1, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                name_of_truck_neiborhood=Neighborhood10.Neighborhood_one_opt_standard,
                solution=current_sol,
                number_of_potial_solution=1,
                number_of_loop_drone=2,
                tabu_list=tabu_structure,
                tabu_tenure=tabu_tenure,
                index_of_loop=lenght_i[1],
                best_fitness=legacy_best_fitness,
                kind_of_tabu_structure=1,
                need_truck_time=False,
                solution_pack=solution_pack,
                solution_pack_len=solution_pack_len,
                use_solution_pack=True,
                index_consider_elite_set=0,
            )
            current_neighborhood.append([1, current_neighborhood1])
        elif choose == 2:
            current_neighborhood5, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                name_of_truck_neiborhood=Neighborhood11.Neighborhood_two_opt_tue,
                solution=current_sol,
                number_of_potial_solution=1,
                number_of_loop_drone=2,
                tabu_list=tabu_structure3,
                tabu_tenure=tabu_tenure3,
                index_of_loop=lenght_i[5],
                best_fitness=legacy_best_fitness,
                kind_of_tabu_structure=5,
                need_truck_time=False,
                solution_pack=solution_pack,
                solution_pack_len=solution_pack_len,
                use_solution_pack=True,
                index_consider_elite_set=0,
            )
            current_neighborhood.append([5, current_neighborhood5])
        elif choose == 3:
            current_neighborhood4, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                name_of_truck_neiborhood=Neighborhood11.Neighborhood_move_2_1,
                solution=current_sol,
                number_of_potial_solution=1,
                number_of_loop_drone=2,
                tabu_list=tabu_structure2,
                tabu_tenure=tabu_tenure2,
                index_of_loop=lenght_i[4],
                best_fitness=legacy_best_fitness,
                kind_of_tabu_structure=4,
                need_truck_time=False,
                solution_pack=solution_pack,
                solution_pack_len=solution_pack_len,
                use_solution_pack=True,
                index_consider_elite_set=0,
            )
            current_neighborhood.append([4, current_neighborhood4])
        else:
            current_neighborhood3, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                name_of_truck_neiborhood=Neighborhood11.Neighborhood_move_1_1_ver2,
                solution=current_sol,
                number_of_potial_solution=1,
                number_of_loop_drone=2,
                tabu_list=tabu_structure1,
                tabu_tenure=tabu_tenure1,
                index_of_loop=lenght_i[3],
                best_fitness=legacy_best_fitness,
                kind_of_tabu_structure=3,
                need_truck_time=False,
                solution_pack=solution_pack,
                solution_pack_len=solution_pack_len,
                use_solution_pack=True,
                index_consider_elite_set=0,
            )
            current_neighborhood.append([3, current_neighborhood3])

        flag = False
        index = [0] * len(current_neighborhood)
        min_nei = [100000] * len(current_neighborhood)
        min_sum = [1000000000] * len(current_neighborhood)
        for j in range(len(current_neighborhood)):
            if current_neighborhood[j][0] in [1, 2]:
                for k in range(len(current_neighborhood[j][1])):
                    cfnode = current_neighborhood[j][1][k][1][0]
                    if cfnode - legacy_best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        legacy_best_fitness = cfnode
                        legacy_best_sol = current_neighborhood[j][1][k][0]
                        flag = True

                    elif cfnode - min_nei[j] < epsilon and tabu_structure[current_neighborhood[j][1][k][2]] + tabu_tenure <= lenght_i[1]:
                        min_nei[j] = cfnode
                        index[j] = k
                        min_sum[j] = current_neighborhood[j][1][k][1][2]

                    elif min_nei[j] - epsilon > cfnode and tabu_structure[current_neighborhood[j][1][k][2]] + tabu_tenure <= lenght_i[1]:
                        if min_sum[j] > current_neighborhood[j][1][k][1][2]:
                            min_nei[j] = cfnode
                            index[j] = k
                            min_sum[j] = current_neighborhood[j][1][k][1][2]
            elif current_neighborhood[j][0] == 3:
                for k in range(len(current_neighborhood[j][1])):
                    cfnode = current_neighborhood[j][1][k][1][0]
                    if cfnode - legacy_best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        legacy_best_fitness = cfnode
                        legacy_best_sol = current_neighborhood[j][1][k][0]
                        flag = True

                    elif cfnode - min_nei[j] < epsilon and tabu_structure1[current_neighborhood[j][1][k][2][0]] + tabu_tenure1 <= lenght_i[3] or tabu_structure1[current_neighborhood[j][1][k][2][1]] + tabu_tenure1 <= lenght_i[3]:
                        min_nei[j] = cfnode
                        index[j] = k
                        min_sum[j] = current_neighborhood[j][1][k][1][2]

                    elif cfnode < min_nei[j] - epsilon and tabu_structure1[current_neighborhood[j][1][k][2][0]] + tabu_tenure1 <= lenght_i[3] or tabu_structure1[current_neighborhood[j][1][k][2][1]] + tabu_tenure1 <= lenght_i[3]:
                        if min_sum[j] > current_neighborhood[j][1][k][1][2]:
                            min_nei[j] = cfnode
                            index[j] = k
                            min_sum[j] = current_neighborhood[j][1][k][1][2]
            elif current_neighborhood[j][0] == 4:
                for k in range(len(current_neighborhood[j][1])):
                    cfnode = current_neighborhood[j][1][k][1][0]
                    if cfnode - legacy_best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        legacy_best_fitness = cfnode
                        legacy_best_sol = current_neighborhood[j][1][k][0]
                        flag = True

                    elif cfnode - min_nei[j] < epsilon and tabu_structure2[current_neighborhood[j][1][k][2][0]] + tabu_tenure2 <= lenght_i[4] or tabu_structure2[current_neighborhood[j][1][k][2][1]] + tabu_tenure2 <= lenght_i[4] or tabu_structure2[current_neighborhood[j][1][k][2][2]] + tabu_tenure2 <= lenght_i[4]:
                        min_nei[j] = cfnode
                        index[j] = k
                        min_sum[j] = current_neighborhood[j][1][k][1][2]

                    elif cfnode < min_nei[j] - epsilon and tabu_structure2[current_neighborhood[j][1][k][2][0]] + tabu_tenure2 <= lenght_i[4] or tabu_structure2[current_neighborhood[j][1][k][2][1]] + tabu_tenure2 <= lenght_i[4] or tabu_structure2[current_neighborhood[j][1][k][2][2]] + tabu_tenure2 <= lenght_i[4]:
                        if min_sum[j] > current_neighborhood[j][1][k][1][2]:
                            min_nei[j] = cfnode
                            index[j] = k
                            min_sum[j] = current_neighborhood[j][1][k][1][2]
            elif current_neighborhood[j][0] == 5:
                for k in range(len(current_neighborhood[j][1])):
                    cfnode = current_neighborhood[j][1][k][1][0]
                    if cfnode - legacy_best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        legacy_best_fitness = cfnode
                        legacy_best_sol = current_neighborhood[j][1][k][0]
                        flag = True

                    elif cfnode - min_nei[j] < epsilon and tabu_structure3[current_neighborhood[j][1][k][2][0]] + tabu_tenure3 <= lenght_i[5] or tabu_structure3[current_neighborhood[j][1][k][2][1]] + tabu_tenure3 <= lenght_i[5]:
                        min_nei[j] = cfnode
                        index[j] = k
                        min_sum[j] = current_neighborhood[j][1][k][1][2]

                    elif cfnode < min_nei[j] - epsilon and tabu_structure3[current_neighborhood[j][1][k][2][0]] + tabu_tenure3 <= lenght_i[5] or tabu_structure3[current_neighborhood[j][1][k][2][1]] + tabu_tenure3 <= lenght_i[5]:
                        if min_sum[j] > current_neighborhood[j][1][k][1][2]:
                            min_nei[j] = cfnode
                            index[j] = k
                            min_sum[j] = current_neighborhood[j][1][k][1][2]

        index_best_nei = 0
        best_fit_in_cur_loop = min_nei[0]
        for j in range(1, len(min_nei)):
            if min_nei[j] < best_fit_in_cur_loop:
                index_best_nei = j
                best_fit_in_cur_loop = min_nei[j]

        if current_neighborhood[index_best_nei][0] in [1, 2]:
            lenght_i[1] += 1
        if current_neighborhood[index_best_nei][0] == 3:
            lenght_i[3] += 1
        if current_neighborhood[index_best_nei][0] == 4:
            lenght_i[4] += 1
        if current_neighborhood[index_best_nei][0] == 5:
            lenght_i[5] += 1

        if len(current_neighborhood[index_best_nei][1]) == 0:
            i += 1
            continue

        current_sol = current_neighborhood[index_best_nei][1][index[index_best_nei]][0]
        current_fitness = current_neighborhood[index_best_nei][1][index[index_best_nei]][1][0]
        current_truck_time = current_neighborhood[index_best_nei][1][index[index_best_nei]][1][1]
        current_sum_fitness = current_neighborhood[index_best_nei][1][index[index_best_nei]][1][2]

        if current_neighborhood[index_best_nei][0] in [1, 2]:
            tabu_structure[current_neighborhood[index_best_nei][1][index[index_best_nei]][2]] = lenght_i[1] - 1
        if current_neighborhood[index_best_nei][0] == 3:
            tabu_structure1[current_neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = lenght_i[3] - 1
            tabu_structure1[current_neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = lenght_i[3] - 1
        if current_neighborhood[index_best_nei][0] == 4:
            tabu_structure2[current_neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = lenght_i[4] - 1
            tabu_structure2[current_neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = lenght_i[4] - 1
            tabu_structure2[current_neighborhood[index_best_nei][1][index[index_best_nei]][2][2]] = lenght_i[4] - 1
        if current_neighborhood[index_best_nei][0] == 5:
            tabu_structure3[current_neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = lenght_i[5] - 1
            tabu_structure3[current_neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = lenght_i[5] - 1

        used[choose] += 1
        if flag is True:
            score[choose] += Data.alpha[0]
        elif current_fitness - prev_fitness < epsilon:
            score[choose] += Data.alpha[1]
        else:
            score[choose] += Data.alpha[2]

        for j in range(len(nei_set)):
            if used[j] == 0:
                continue
            weight[j] = (1 - factor) * weight[j] + factor * score[j] / used[j]

        if flag is True:
            i = 0
        else:
            i += 1

        loop_iterations += 1

        best_solution, best_eval, best_multi_visit_solution, best_multi_visit_eval = _try_record_feasible_candidate(
            legacy_best_sol,
            data,
            best_solution,
            best_eval,
            best_multi_visit_solution,
            best_multi_visit_eval,
        )
        best_solution, best_eval, best_multi_visit_solution, best_multi_visit_eval = _try_record_feasible_candidate(
            current_sol,
            data,
            best_solution,
            best_eval,
            best_multi_visit_solution,
            best_multi_visit_eval,
        )

    elapsed_sec = time.time() - started
    return {
        "best_solution": best_solution,
        "best_eval": best_eval,
        "best_multi_visit_solution": best_multi_visit_solution,
        "best_multi_visit_eval": best_multi_visit_eval,
        "segments_run": 1,
        "time_limit_reached": time_limit_reached,
        "elapsed_sec": elapsed_sec,
        "loop_iterations": loop_iterations,
    }


def process_row(
    row: Dict[str, str],
    row_idx: int,
    nimp: int,
    truck_max_neighbors: Optional[int],
    drone_max_neighbors: Optional[int],
    max_segments: int,
    max_runtime_sec: Optional[float],
) -> Tuple[Dict[str, str], Dict[str, Any]]:
    started = time.time()
    updated = dict(row)

    old_best_fitness = row.get("best_fitness", "")
    old_best_sol = row.get("best_sol", "")
    old_mv_fitness = row.get("best_multi_visit_fitness", "")
    old_mv_sol = row.get("best_multi_visit_sol", "")

    compare: Dict[str, Any] = {
        "row_index": row_idx,
        "instance": row.get("instance", ""),
        "run": row.get("run", ""),
        "old_best_fitness": old_best_fitness,
        "old_best_multi_visit_fitness": old_mv_fitness,
        "best_sol_initially_feasible": "",
        "best_sol_repair_mode": "",
        "best_multi_visit_initially_feasible": "",
        "best_multi_visit_repair_mode": "",
        "ats_engine": "",
        "ats_segments_run": "",
        "ats_loop_iterations": "",
        "ats_time_limit_reached": "",
        "ats_elapsed_sec": "",
        "new_best_fitness": "",
        "new_best_multi_visit_fitness": "",
        "best_multi_visit_source": "",
        "status": "INIT",
        "error": "",
        "runtime_sec": "",
    }

    instance_path = _resolve_instance_path(row)
    data = read_data_file(instance_path)

    row_capacity = _float_or_none(row.get("drone_capacity", ""))
    row_limit = _float_or_none(row.get("drone_limit_time", ""))
    if row_capacity is not None:
        data.drone_capacity = row_capacity
    if row_limit is not None:
        data.drone_limit_time = row_limit

    raw_best = _parse_legacy_solution(old_best_sol)
    raw_mv = _parse_legacy_solution(old_mv_sol)

    best_sol, best_eval, best_mode, best_initially_feasible = _repair_solution_for_rules(
        raw_best,
        data,
        fallback_build_initial=True,
    )
    compare["best_sol_initially_feasible"] = best_initially_feasible
    compare["best_sol_repair_mode"] = best_mode

    if raw_mv is None:
        mv_fixed = None
        mv_fixed_eval = None
        mv_mode = "missing"
        mv_initially_feasible = ""
    else:
        try:
            mv_fixed, mv_fixed_eval, mv_mode, mv_initially_feasible = _repair_solution_for_rules(
                raw_mv,
                data,
                fallback_build_initial=False,
            )
        except Exception:
            mv_fixed = best_sol
            mv_fixed_eval = best_eval
            mv_mode = "fallback_best_sol_after_failed_repair"
            mv_initially_feasible = False

    compare["best_multi_visit_initially_feasible"] = mv_initially_feasible
    compare["best_multi_visit_repair_mode"] = mv_mode

    mv_candidates: List[Tuple[Solution, Any, str]] = []
    if mv_fixed is not None:
        mv_candidates.append((mv_fixed, mv_fixed_eval, "fixed_input_best_multi_visit_sol"))

    if max_segments == 0:
        compare["ats_engine"] = "none"
        compare["ats_segments_run"] = 0
        compare["ats_loop_iterations"] = 0
        compare["ats_time_limit_reached"] = False
        compare["ats_elapsed_sec"] = 0.0
        final_best_sol = best_sol
        final_best_eval = best_eval
        mv_candidates.append((final_best_sol, final_best_eval, "repaired_best_sol_if_multi_visit"))
    else:
        compare["ats_engine"] = "test_similarity"
        if max_segments < 0:
            raise ValueError("max_segments < 0 is not supported in test_similarity ATS mode.")

        segments_target = max(1, int(max_segments))
        final_best_sol = Solution.from_legacy(best_sol.to_legacy())
        final_best_eval = best_eval
        seed_solution = Solution.from_legacy(best_sol.to_legacy())

        total_elapsed = 0.0
        total_segments_run = 0
        total_loop_iterations = 0
        time_limit_reached = False

        for segment_index in range(segments_target):
            remaining_runtime = None
            if max_runtime_sec is not None:
                remaining_runtime = max_runtime_sec - total_elapsed
                if remaining_runtime <= 0:
                    time_limit_reached = True
                    break

            legacy_result = _run_test_similarity_segment_with_source2_feasibility(
                instance_path=instance_path,
                row=row,
                row_idx=row_idx + segment_index,
                initial_solution=seed_solution,
                data=data,
                max_runtime_sec=remaining_runtime,
            )

            total_segments_run += int(legacy_result["segments_run"])
            total_loop_iterations += int(legacy_result["loop_iterations"])
            total_elapsed += float(legacy_result["elapsed_sec"])

            candidate_best_sol = legacy_result["best_solution"]
            candidate_best_eval = legacy_result["best_eval"]
            if _is_better_eval(candidate_best_eval, final_best_eval):
                final_best_sol = Solution.from_legacy(candidate_best_sol.to_legacy())
                final_best_eval = candidate_best_eval

            if legacy_result["best_multi_visit_solution"] is not None and legacy_result["best_multi_visit_eval"] is not None:
                mv_candidates.append(
                    (
                        legacy_result["best_multi_visit_solution"],
                        legacy_result["best_multi_visit_eval"],
                        "test_similarity_best_multi_visit",
                    )
                )

            seed_solution = Solution.from_legacy(candidate_best_sol.to_legacy())
            if bool(legacy_result["time_limit_reached"]):
                time_limit_reached = True
                break

        compare["ats_segments_run"] = total_segments_run
        compare["ats_loop_iterations"] = total_loop_iterations
        compare["ats_time_limit_reached"] = time_limit_reached
        compare["ats_elapsed_sec"] = round(total_elapsed, 3)
        mv_candidates.append((final_best_sol, final_best_eval, "test_similarity_best_sol_if_multi_visit"))

    final_mv_sol, final_mv_eval, mv_source = _choose_best_multi_visit(data, mv_candidates)

    updated["best_fitness"] = _fmt_float(final_best_eval.objective)
    updated["best_sol"] = str(final_best_sol.to_legacy())

    if final_mv_sol is None or final_mv_eval is None:
        updated["best_multi_visit_fitness"] = ""
        updated["best_multi_visit_sol"] = "None"
    else:
        updated["best_multi_visit_fitness"] = _fmt_float(final_mv_eval.objective)
        updated["best_multi_visit_sol"] = str(final_mv_sol.to_legacy())

    compare["new_best_fitness"] = updated["best_fitness"]
    compare["new_best_multi_visit_fitness"] = updated["best_multi_visit_fitness"]
    compare["best_multi_visit_source"] = mv_source
    compare["status"] = "OK"
    compare["runtime_sec"] = round(time.time() - started, 3)

    return updated, compare


def _process_row_task(
    args: Tuple[int, Dict[str, str], int, Optional[int], Optional[int], int, Optional[float]]
) -> Tuple[int, Dict[str, str], Dict[str, Any]]:
    idx, row, nimp, truck_max_neighbors, drone_max_neighbors, max_segments, max_runtime_sec = args
    updated, compare = process_row(
        row=row,
        row_idx=idx,
        nimp=nimp,
        truck_max_neighbors=truck_max_neighbors,
        drone_max_neighbors=drone_max_neighbors,
        max_segments=max_segments,
        max_runtime_sec=max_runtime_sec,
    )
    return idx, updated, compare


def run(
    input_csv: Path,
    output_csv: Path,
    backup_csv: Path,
    compare_csv: Path,
    limit: int = 0,
    start_row: int = 1,
    end_row: int = 0,
    workers: int = 1,
    nimp: int = 30,
    max_segments: int = 1,
    max_runtime_sec: Optional[float] = None,
    truck_max_neighbors: Optional[int] = 300,
    drone_max_neighbors: Optional[int] = 120,
    patch_csv: Optional[Path] = None,
    allow_inplace_limited_output: bool = False,
) -> None:
    if not input_csv.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    with input_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        all_rows = list(reader)

    indexed_rows: List[Tuple[int, Dict[str, str]]] = list(enumerate(all_rows, start=1))
    source_total = len(indexed_rows)

    safe_start = max(1, int(start_row))
    safe_end = source_total if int(end_row) <= 0 else min(source_total, int(end_row))
    if safe_start > safe_end:
        indexed_rows = []
    else:
        indexed_rows = indexed_rows[safe_start - 1 : safe_end]

    if limit and limit > 0:
        indexed_rows = indexed_rows[:limit]

    is_partial_selection = len(indexed_rows) != source_total
    if output_csv.resolve() == input_csv.resolve() and is_partial_selection and not allow_inplace_limited_output:
        raise ValueError(
            "Refusing to overwrite input CSV with a partial subset (limit/start/end row selection). "
            "Use a different --output-csv, or pass --allow-inplace-limited-output explicitly."
        )

    updated_by_idx: Dict[int, Dict[str, str]] = {}
    compare_by_idx: Dict[int, Dict[str, Any]] = {}

    total = len(indexed_rows)
    requested_workers = max(1, int(workers))
    worker_count = _effective_workers(requested_workers, total)
    print(
        f"workers_requested={requested_workers} "
        f"workers_effective={worker_count} rows={total} cpu_count={max(1, os.cpu_count() or 1)}"
    )
    print(
        f"selection start_row={safe_start} end_row={safe_end if source_total else 0} "
        f"limit={max(0, int(limit))} selected_rows={total} source_rows={source_total}"
    )

    tasks = [
        (idx, row, nimp, truck_max_neighbors, drone_max_neighbors, max_segments, max_runtime_sec)
        for idx, row in indexed_rows
    ]

    if worker_count == 1:
        for done, (idx, row) in enumerate(indexed_rows, start=1):
            try:
                updated, compare = process_row(
                    row=row,
                    row_idx=idx,
                    nimp=nimp,
                    truck_max_neighbors=truck_max_neighbors,
                    drone_max_neighbors=drone_max_neighbors,
                    max_segments=max_segments,
                    max_runtime_sec=max_runtime_sec,
                )
            except Exception as exc:  # noqa: BLE001
                updated, compare = _error_result(idx, row, exc)
            updated_by_idx[idx] = updated
            compare_by_idx[idx] = compare
            print(
                f"[{done}/{total}] {compare['status']} row_index={idx} "
                f"instance={compare['instance']} run={compare['run']}"
            )
    else:
        with ProcessPoolExecutor(max_workers=worker_count) as executor:
            future_map = {executor.submit(_process_row_task, task): task for task in tasks}
            done = 0
            for future in as_completed(future_map):
                idx, row, _, _, _, _, _ = future_map[future]
                try:
                    _, updated, compare = future.result()
                except Exception as exc:  # noqa: BLE001
                    updated, compare = _error_result(idx, row, exc)
                updated_by_idx[idx] = updated
                compare_by_idx[idx] = compare
                done += 1
                print(
                    f"[{done}/{total}] {compare['status']} row_index={idx} "
                    f"instance={compare['instance']} run={compare['run']}"
                )

    updated_rows = [updated_by_idx[i] for i in sorted(updated_by_idx.keys())]
    compare_rows = [compare_by_idx[i] for i in sorted(compare_by_idx.keys())]

    if output_csv.resolve() == input_csv.resolve():
        backup_csv.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(input_csv, backup_csv)
        print(f"backup_csv={backup_csv}")

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(updated_rows)

    compare_fieldnames: List[str] = []
    for row in compare_rows:
        for key in row.keys():
            if key not in compare_fieldnames:
                compare_fieldnames.append(key)
    compare_csv.parent.mkdir(parents=True, exist_ok=True)
    with compare_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=compare_fieldnames)
        writer.writeheader()
        writer.writerows(compare_rows)

    if patch_csv is not None:
        patch_rows: List[Dict[str, Any]] = []
        for idx in sorted(updated_by_idx.keys()):
            updated = updated_by_idx[idx]
            compare = compare_by_idx[idx]
            patch_rows.append(
                {
                    "row_index": idx,
                    "status": compare.get("status", ""),
                    "error": compare.get("error", ""),
                    "best_fitness": updated.get("best_fitness", ""),
                    "best_sol": updated.get("best_sol", ""),
                    "best_multi_visit_fitness": updated.get("best_multi_visit_fitness", ""),
                    "best_multi_visit_sol": updated.get("best_multi_visit_sol", ""),
                    "ats_engine": compare.get("ats_engine", ""),
                    "ats_segments_run": compare.get("ats_segments_run", ""),
                    "ats_loop_iterations": compare.get("ats_loop_iterations", ""),
                    "ats_time_limit_reached": compare.get("ats_time_limit_reached", ""),
                    "ats_elapsed_sec": compare.get("ats_elapsed_sec", ""),
                    "runtime_sec": compare.get("runtime_sec", ""),
                }
            )
        patch_fieldnames = [
            "row_index",
            "status",
            "error",
            "best_fitness",
            "best_sol",
            "best_multi_visit_fitness",
            "best_multi_visit_sol",
            "ats_engine",
            "ats_segments_run",
            "ats_loop_iterations",
            "ats_time_limit_reached",
            "ats_elapsed_sec",
            "runtime_sec",
        ]
        patch_csv.parent.mkdir(parents=True, exist_ok=True)
        with patch_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=patch_fieldnames)
            writer.writeheader()
            writer.writerows(patch_rows)
        print(f"patch_csv={patch_csv}")

    ok = sum(1 for r in compare_rows if r.get("status") == "OK")
    print(f"output_csv={output_csv}")
    print(f"compare_csv={compare_csv}")
    print(f"rows={len(updated_rows)} ok={ok} error={len(updated_rows) - ok} source_rows={source_total}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Repair invalid best_sol / best_multi_visit_sol by Source_revised2 rules, "
            "optionally improve by 1+ legacy ATS segment(s) from test_similarity.py, "
            "while only recording best solutions that are feasible by Source_revised2."
        )
    )
    parser.add_argument("--input-csv", default=str(DEFAULT_INPUT), help="Input CSV path.")
    parser.add_argument("--output-csv", default=str(DEFAULT_OUTPUT), help="Output CSV path.")
    parser.add_argument("--backup-csv", default=str(DEFAULT_BACKUP), help="Backup path when overwriting input.")
    parser.add_argument("--compare-csv", default=str(DEFAULT_COMPARE), help="Compare/audit CSV path.")
    parser.add_argument("--patch-csv", default="", help="Optional patch CSV output (row_index + updated fields).")
    parser.add_argument("--limit", type=int, default=0, help="Only process first N rows (0 = all).")
    parser.add_argument("--start-row", type=int, default=1, help="1-based start row index in input CSV.")
    parser.add_argument("--end-row", type=int, default=0, help="1-based end row index (0 = last row).")
    parser.add_argument("--workers", type=int, default=1, help="Parallel worker processes (1 = sequential).")
    parser.add_argument("--nimp", type=int, default=30, help="Deprecated in test_similarity ATS mode (kept for CLI compatibility).")
    parser.add_argument(
        "--max-segments",
        type=int,
        default=1,
        help="ATS max segments (0 = repair only, -1 = unlimited until stopping rules/time limit).",
    )
    parser.add_argument(
        "--max-runtime-sec",
        type=float,
        default=0.0,
        help="ATS wall-clock time limit per row in seconds (<=0 disables).",
    )
    parser.add_argument(
        "--truck-max-neighbors",
        type=int,
        default=300,
        help="Deprecated in test_similarity ATS mode (kept for CLI compatibility).",
    )
    parser.add_argument(
        "--drone-max-neighbors",
        type=int,
        default=120,
        help="Deprecated in test_similarity ATS mode (kept for CLI compatibility).",
    )
    parser.add_argument(
        "--allow-inplace-limited-output",
        action="store_true",
        help="Allow overwriting input CSV even when only a subset is selected (unsafe).",
    )
    args = parser.parse_args()

    truck_max_neighbors = None if int(args.truck_max_neighbors) < 0 else int(args.truck_max_neighbors)
    drone_max_neighbors = None if int(args.drone_max_neighbors) < 0 else int(args.drone_max_neighbors)
    max_runtime_sec = None if float(args.max_runtime_sec) <= 0 else float(args.max_runtime_sec)
    patch_csv = Path(args.patch_csv) if str(args.patch_csv).strip() else None

    run(
        input_csv=Path(args.input_csv),
        output_csv=Path(args.output_csv),
        backup_csv=Path(args.backup_csv),
        compare_csv=Path(args.compare_csv),
        patch_csv=patch_csv,
        limit=max(0, int(args.limit)),
        start_row=max(1, int(args.start_row)),
        end_row=int(args.end_row),
        workers=max(1, int(args.workers)),
        nimp=max(1, int(args.nimp)),
        max_segments=int(args.max_segments),
        max_runtime_sec=max_runtime_sec,
        truck_max_neighbors=truck_max_neighbors,
        drone_max_neighbors=drone_max_neighbors,
        allow_inplace_limited_output=bool(args.allow_inplace_limited_output),
    )


if __name__ == "__main__":
    main()
