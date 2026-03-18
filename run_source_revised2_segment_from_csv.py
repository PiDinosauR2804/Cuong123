from __future__ import annotations

import argparse
import ast
import csv
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Dict, List, Optional

from Source_revised2 import (
    AtsParams,
    Solution,
    adaptive_tabu_search,
    build_initial_solution,
    evaluate_fitness,
    local_search_drone,
    read_data_file,
    repair_solution_after_truck_routes,
)


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT_CSV = (
    r"c:\Users\HoangHaPC\Downloads\multivisit-experiment-summary-from-existing-run (3)\c201_batch_data.csv"
)


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


def _resolve_instance_path(row: Dict[str, str]) -> Optional[Path]:
    candidates: List[Path] = []

    raw_instance = (row.get("instance") or "").strip()
    if raw_instance:
        normalized = raw_instance.replace("\\", "/")
        p = Path(normalized)
        candidates.append(p if p.is_absolute() else PROJECT_ROOT / normalized)

    group = (row.get("instance_group") or "").strip().lower()
    name = (row.get("instance_name") or "").strip()
    if name:
        if group in {"cluster", "random"}:
            candidates.append(
                PROJECT_ROOT
                / f"test_data/data_demand_random_50_batch_all1_equal_{group}"
                / name
            )
        candidates.append(PROJECT_ROOT / "test_data/data_demand_random_50_batch_all1_equal_cluster" / name)
        candidates.append(PROJECT_ROOT / "test_data/data_demand_random_50_batch_all1_equal_random" / name)

    for candidate in candidates:
        try:
            resolved = candidate.resolve()
        except Exception:
            resolved = candidate
        if resolved.exists():
            return resolved
    return None


def _count_multi_visit(solution: Optional[Solution]) -> int:
    if solution is None:
        return 0
    return sum(1 for trip in solution.drone_queue if trip.is_multi_visit)


def _repair_seed_solution(seed: Solution, data) -> Solution:
    repaired = repair_solution_after_truck_routes(seed, _route_lists(seed), data)
    if repaired is None:
        repaired = local_search_drone(seed, data, max_iterations=0, max_neighbors=None)
    else:
        repaired = local_search_drone(repaired, data, max_iterations=0, max_neighbors=None)

    repaired_eval = evaluate_fitness(repaired, data)
    if repaired_eval.feasible:
        return repaired

    # Fallback: ensure ATS always starts from feasible state.
    return build_initial_solution(
        data,
        apply_drone_local_search=True,
        drone_ls_iterations=None,
        drone_ls_max_neighbors=None,
    )


def _float_or_none(raw: str) -> Optional[float]:
    text = (raw or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except Exception:
        return None


def process_row(row: Dict[str, str], nimp: int, verbose: bool = False) -> Dict[str, Any]:
    t0 = time.time()
    out: Dict[str, Any] = dict(row)
    out.update(
        {
            "source_status": row.get("status", ""),
            "source_runtime_sec": row.get("runtime_sec", ""),
            "resolved_instance": "",
            "seed_source": "",
            "seed_repair_mode": "",
            "seed_fixed_sol": "",
            "seed_feasible": "",
            "seed_objective": "",
            "ats_segments_run": "",
            "ats_best_fitness": "",
            "ats_best_sol": "",
            "ats_best_multi_visit_fitness": "",
            "ats_best_multi_visit_sol": "",
            "ats_best_multi_visit_trip_count": "",
            "status": "INIT",
            "runtime_sec": "",
        }
    )

    instance_path = _resolve_instance_path(row)
    if instance_path is None:
        out["status"] = "DATA_NOT_FOUND"
        out["runtime_sec"] = round(time.time() - t0, 3)
        return out
    out["resolved_instance"] = str(instance_path)

    try:
        data = read_data_file(instance_path)

        row_capacity = _float_or_none(row.get("drone_capacity", ""))
        row_ld = _float_or_none(row.get("drone_limit_time", ""))
        if row_capacity is not None:
            data.drone_capacity = row_capacity
        if row_ld is not None:
            data.drone_limit_time = row_ld

        seed = _parse_legacy_solution(row.get("best_sol", ""))
        seed_source = "best_sol"
        if seed is None:
            seed = _parse_legacy_solution(row.get("best_multi_visit_sol", ""))
            seed_source = "best_multi_visit_sol"

        if seed is None:
            seed = build_initial_solution(
                data,
                apply_drone_local_search=True,
                drone_ls_iterations=None,
                drone_ls_max_neighbors=None,
            )
            seed_source = "build_initial_solution"
            out["seed_repair_mode"] = "fallback_build_initial_solution"
        else:
            seed = _repair_seed_solution(seed, data)
            out["seed_repair_mode"] = "repair_solution_after_truck_routes"

        out["seed_source"] = seed_source
        out["seed_fixed_sol"] = str(seed.to_legacy())
        seed_eval = evaluate_fitness(seed, data)
        out["seed_feasible"] = seed_eval.feasible
        out["seed_objective"] = seed_eval.objective

        params = AtsParams(
            nimp=nimp,
            seg=8,
            div=3,
            seed=None,
            truck_max_neighbors=None,
            drone_max_neighbors=None,
            use_drone_refine=True,
            drone_ls_iterations=None,
            diversification_max_neighbors=None,
            diversification_max_moves_per_route=None,
            diversification_accept_non_improving=True,
            max_segments=1,
        )

        ats = adaptive_tabu_search(
            data=data,
            params=params,
            initial_solution=seed,
            verbose=verbose,
        )

        out["ats_segments_run"] = ats.segments_run
        out["ats_best_fitness"] = ats.best_eval.objective
        out["ats_best_sol"] = str(ats.best_solution.to_legacy())

        if ats.best_multi_visit_solution is not None and ats.best_multi_visit_eval is not None:
            out["ats_best_multi_visit_fitness"] = ats.best_multi_visit_eval.objective
            out["ats_best_multi_visit_sol"] = str(ats.best_multi_visit_solution.to_legacy())
            out["ats_best_multi_visit_trip_count"] = _count_multi_visit(ats.best_multi_visit_solution)
        else:
            out["ats_best_multi_visit_fitness"] = ""
            out["ats_best_multi_visit_sol"] = ""
            out["ats_best_multi_visit_trip_count"] = 0

        out["status"] = "OK"
    except Exception as exc:  # noqa: BLE001
        out["status"] = f"ERROR: {type(exc).__name__}: {exc}"

    out["runtime_sec"] = round(time.time() - t0, 3)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Repair legacy solutions from CSV and run 1 ATS segment (Source_revised2)."
    )
    parser.add_argument("--input-csv", default=DEFAULT_INPUT_CSV, help="Input CSV path.")
    parser.add_argument("--output-csv", default="", help="Output CSV path.")
    parser.add_argument("--limit", type=int, default=0, help="Process first N rows only (0 = all).")
    parser.add_argument("--nimp", type=int, default=30, help="ATS inner non-improving iteration limit.")
    parser.add_argument("--workers", type=int, default=10, help="Number of parallel worker threads.")
    parser.add_argument("--verbose", action="store_true", help="Enable ATS verbose logs.")
    args = parser.parse_args()

    input_csv = Path(args.input_csv)
    if not input_csv.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    with input_csv.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if args.limit and args.limit > 0:
        rows = rows[: args.limit]

    processed_by_index: Dict[int, Dict[str, Any]] = {}
    total = len(rows)

    workers = max(1, int(args.workers))
    with ThreadPoolExecutor(max_workers=workers) as executor:
        future_map = {}
        for idx, row in enumerate(rows, start=1):
            future = executor.submit(process_row, row, args.nimp, args.verbose)
            future_map[future] = (idx, row)

        done = 0
        for future in as_completed(future_map):
            idx, row = future_map[future]
            instance_name = row.get("instance_name", "")
            run_id = row.get("run", "")
            try:
                processed_by_index[idx] = future.result()
            except Exception as exc:  # noqa: BLE001
                processed_by_index[idx] = {
                    **row,
                    "status": f"ERROR: {type(exc).__name__}: {exc}",
                    "runtime_sec": "",
                }
            done += 1
            print(f"[{done}/{total}] done idx={idx} {instance_name} run={run_id}")

    processed: List[Dict[str, Any]] = [processed_by_index[i] for i in sorted(processed_by_index.keys())]

    if args.output_csv:
        out_csv = Path(args.output_csv)
        out_csv.parent.mkdir(parents=True, exist_ok=True)
    else:
        out_dir = PROJECT_ROOT / "Source_revised2" / "Result"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_csv = out_dir / f"source_revised2_c201_segment1_unlimited_{int(time.time())}.csv"

    fieldnames: List[str] = []
    for row in processed:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)

    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(processed)

    ok = sum(1 for r in processed if r.get("status") == "OK")
    print(f"output_csv={out_csv}")
    print(f"rows={len(processed)} ok={ok}")


if __name__ == "__main__":
    main()
