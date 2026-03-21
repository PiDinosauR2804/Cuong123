from __future__ import annotations

import argparse
import ast
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import datetime as dt
import hashlib
import json
import random
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Source_revised2 import (
    AtsParams,
    Solution,
    adaptive_tabu_search,
    build_initial_solution,
    evaluate_fitness,
    read_data_file,
)

_LEGACY_IMPORT_ERROR: Optional[Exception] = None
try:
    import Data  # type: ignore
    import Function  # type: ignore
except Exception as exc:  # noqa: BLE001
    Data = None  # type: ignore[assignment]
    Function = None  # type: ignore[assignment]
    _LEGACY_IMPORT_ERROR = exc


CSV_FIELDS: List[str] = [
    "group_id",
    "group_key",
    "job_id",
    "instance",
    "A",
    "L",
    "run_index",
    "seed",
    "status",
    "error",
    "init_method",
    "init_feasible",
    "initial_fitness",
    "best_fitness",
    "best_solution",
    "best_drone_trip_count",
    "best_multi_visit_trip_count",
    "best_multi_fitness",
    "best_multi_solution",
    "best_multi_drone_trip_count",
    "best_multi_multi_visit_trip_count",
    "ats_segments_run",
    "ats_diversification_rounds",
    "ats_time_limit_reached",
    "ats_elapsed_sec",
    "solver_nimp",
    "solver_seg",
    "solver_div",
    "solver_max_runtime_sec",
    "started_utc",
    "finished_utc",
    "wall_time_sec",
]

SUMMARY_BATCH_INIT_FIELDS: List[str] = [
    "job_id",
    "instance",
    "A",
    "L",
    "run_index",
    "best_fitness",
    "best_solution",
    "best_multi_fitness",
    "best_multi_solution",
    "best_drone_avg_trip_time",
    "best_multi_visit_trip_count",
    "best_drone_trip_count",
    "best_avg_customers_per_trip",
    "best_multi_drone_avg_trip_time",
    "best_multi_multi_visit_trip_count",
    "best_multi_drone_trip_count",
    "best_multi_avg_customers_per_trip",
    "best_solution_file",
    "best_multi_solution_file",
    "log",
]


def _utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def _safe_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except Exception:
        return None


def _is_abs_path(raw: str) -> bool:
    text = raw.strip()
    if not text:
        return False
    if text.startswith("/"):
        return True
    return bool(re.match(r"^[A-Za-z]:[\\/]", text))


def _strip_quotes(text: str) -> str:
    s = text.strip()
    if (len(s) >= 2 and s[0] == '"' and s[-1] == '"') or (len(s) >= 2 and s[0] == "'" and s[-1] == "'"):
        return s[1:-1]
    return s


def _parse_jobs_from_yaml(config_path: Path) -> List[Dict[str, Any]]:
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    lines = config_path.read_text(encoding="utf-8").splitlines()
    jobs: List[Dict[str, Any]] = []
    in_jobs = False
    current: Optional[Dict[str, Any]] = None

    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue

        if stripped == "jobs:":
            in_jobs = True
            current = None
            continue

        if not in_jobs:
            continue

        if stripped.startswith("- id:"):
            if current is not None:
                jobs.append(current)
            value = stripped.split(":", 1)[1].strip()
            current = {"id": int(float(value))}
            continue

        if current is None:
            continue

        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        key = key.strip()
        value = _strip_quotes(value.strip())

        if key == "instance":
            current["instance"] = value
        elif key in {"A", "L", "run_index"}:
            current[key] = int(float(value))

    if current is not None:
        jobs.append(current)

    clean_jobs: List[Dict[str, Any]] = []
    for job in jobs:
        missing = [k for k in ("id", "instance", "A", "L", "run_index") if k not in job]
        if missing:
            raise ValueError(f"Job {job.get('id', '<unknown>')} missing keys: {missing}")
        clean_jobs.append(
            {
                "id": int(job["id"]),
                "instance": str(job["instance"]),
                "A": int(job["A"]),
                "L": int(job["L"]),
                "run_index": int(job["run_index"]),
            }
        )

    clean_jobs.sort(key=lambda x: int(x["id"]))
    return clean_jobs


def _group_jobs_by_instance_A_L(jobs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, int, int], List[Dict[str, Any]]] = {}
    for job in jobs:
        key = (str(job["instance"]), int(job["A"]), int(job["L"]))
        grouped.setdefault(key, []).append(job)

    groups: List[Dict[str, Any]] = []
    sorted_keys = sorted(grouped.keys(), key=lambda x: (x[0], x[1], x[2]))
    for idx, key in enumerate(sorted_keys, start=1):
        instance, a_val, l_val = key
        group_jobs = sorted(grouped[key], key=lambda x: (int(x["run_index"]), int(x["id"])))
        run_indexes = [int(j["run_index"]) for j in group_jobs]
        groups.append(
            {
                "group_id": idx,
                "group_key": f"{instance}|A={a_val}|L={l_val}",
                "instance": instance,
                "A": int(a_val),
                "L": int(l_val),
                "run_indexes": run_indexes,
                "jobs": group_jobs,
            }
        )
    return groups


def _resolve_instance_path(instance_raw: str) -> Path:
    normalized = instance_raw.replace("\\", "/")
    if _is_abs_path(normalized):
        candidate = Path(normalized)
    else:
        candidate = PROJECT_ROOT / normalized
    return candidate.resolve()


def _seed_for_job(job: Dict[str, Any]) -> int:
    key = f"{job['instance']}|{job['A']}|{job['L']}|{job['run_index']}"
    digest = hashlib.sha256(key.encode("utf-8")).hexdigest()
    return int(digest[:8], 16)


def _to_one_line_solution(solution: Optional[Solution]) -> str:
    if solution is None:
        return "None"
    return json.dumps(solution.to_legacy(), ensure_ascii=False, separators=(",", ":"))


def _drone_trip_count(solution: Optional[Solution]) -> int:
    if solution is None:
        return 0
    return len(solution.drone_queue)


def _multi_visit_trip_count(solution: Optional[Solution]) -> int:
    if solution is None:
        return 0
    return sum(1 for trip in solution.drone_queue if trip.is_multi_visit)


def _parse_solution_text(raw: Any) -> Optional[Solution]:
    text = str(raw or "").strip()
    if not text or text.lower() in {"none", "null", "nan"}:
        return None

    parsers = [json.loads, ast.literal_eval]
    for parser in parsers:
        try:
            parsed = parser(text)
            return Solution.from_legacy(parsed)
        except Exception:
            continue
    return None


def _ensure_legacy_modules_available() -> None:
    if Data is None or Function is None:
        msg = (
            "Legacy modules Data/Function are unavailable. "
            "Install dependencies (e.g. numpy) before running solve mode."
        )
        if _LEGACY_IMPORT_ERROR is not None:
            raise ModuleNotFoundError(msg) from _LEGACY_IMPORT_ERROR
        raise ModuleNotFoundError(msg)


def _avg_customers_per_trip(solution: Optional[Solution]) -> float:
    if solution is None or not solution.drone_queue:
        return 0.0
    counts: List[int] = []
    for trip in solution.drone_queue:
        delivered = 0
        for leg in trip.legs:
            delivered += len(leg.customers)
        counts.append(delivered)
    if not counts:
        return 0.0
    return float(sum(counts) / len(counts))


def _avg_drone_trip_energy_time(solution: Optional[Solution], data) -> float:
    if solution is None:
        return 0.0
    ev = evaluate_fitness(solution, data)
    if not ev.feasible:
        return 0.0
    values = list(ev.drone_flight_wait_energy_time.values())
    if not values:
        return 0.0
    return float(sum(values) / len(values))


def _objective_or_none(solution: Optional[Solution], data) -> Optional[float]:
    if solution is None:
        return None
    ev = evaluate_fitness(solution, data)
    if not ev.feasible:
        return None
    return float(ev.objective)


def _build_initial_solution_from_test_similarity(
    instance_path: Path,
    data,
    seed: int,
) -> Tuple[Solution, Any, str, bool]:
    _ensure_legacy_modules_available()
    random.seed(seed)
    Data.read_data_random(str(instance_path))
    Data.number_of_trucks = int(data.number_truck)
    Data.number_of_drones = int(data.number_drone)
    Data.drone_capacity = float(data.drone_capacity)
    Data.drone_limit_time = float(data.drone_limit_time)

    try:
        legacy_init = Function.initial_solution7()
        init_solution = Solution.from_legacy(legacy_init)
        init_eval = evaluate_fitness(init_solution, data)
        if init_eval.feasible:
            return init_solution, init_eval, "test_similarity_initial_solution7", True
    except Exception:
        pass

    # Fallback to Source_revised2 initializer when legacy init is infeasible/fails.
    fallback = build_initial_solution(
        data,
        apply_drone_local_search=True,
        drone_ls_iterations=None,
        drone_ls_max_neighbors=120,
    )
    fallback_eval = evaluate_fitness(fallback, data)
    if not fallback_eval.feasible:
        raise RuntimeError("Cannot build a feasible initial solution.")
    return fallback, fallback_eval, "source_revised2_fallback_initializer", False


def run_one_job(
    job: Dict[str, Any],
    max_runtime_sec: float,
    nimp: int,
    seg: int,
    div: int,
    verbose: bool = False,
) -> Dict[str, Any]:
    wall_started = time.perf_counter()
    started_utc = _utc_now_iso()

    row: Dict[str, Any] = {
        "group_id": "",
        "group_key": "",
        "job_id": int(job["id"]),
        "instance": str(job["instance"]),
        "A": int(job["A"]),
        "L": int(job["L"]),
        "run_index": int(job["run_index"]),
        "seed": "",
        "status": "ERROR",
        "error": "",
        "init_method": "",
        "init_feasible": "",
        "initial_fitness": "",
        "best_fitness": "",
        "best_solution": "",
        "best_drone_trip_count": "",
        "best_multi_visit_trip_count": "",
        "best_multi_fitness": "",
        "best_multi_solution": "",
        "best_multi_drone_trip_count": "",
        "best_multi_multi_visit_trip_count": "",
        "ats_segments_run": "",
        "ats_diversification_rounds": "",
        "ats_time_limit_reached": "",
        "ats_elapsed_sec": "",
        "solver_nimp": int(nimp),
        "solver_seg": int(seg),
        "solver_div": int(div),
        "solver_max_runtime_sec": float(max_runtime_sec),
        "started_utc": started_utc,
        "finished_utc": "",
        "wall_time_sec": "",
    }

    try:
        seed = _seed_for_job(job)
        row["seed"] = int(seed)
        instance_path = _resolve_instance_path(str(job["instance"]))
        if not instance_path.exists():
            raise FileNotFoundError(f"Instance file not found: {instance_path}")

        data = read_data_file(instance_path)
        data.drone_capacity = float(job["A"])
        data.drone_limit_time = float(job["L"])

        # Solve-time budget starts from init solution phase (as requested).
        solve_started = time.perf_counter()
        init_solution, init_eval, init_method, init_feasible = _build_initial_solution_from_test_similarity(
            instance_path=instance_path,
            data=data,
            seed=seed,
        )
        row["init_method"] = init_method
        row["init_feasible"] = bool(init_feasible)
        row["initial_fitness"] = float(init_eval.objective)

        elapsed_after_init = time.perf_counter() - solve_started
        remaining_runtime = float(max_runtime_sec) - float(elapsed_after_init)

        if remaining_runtime <= 0:
            # Time budget is consumed during init; return current feasible bests.
            row["best_fitness"] = float(init_eval.objective)
            row["best_solution"] = _to_one_line_solution(init_solution)
            row["best_drone_trip_count"] = _drone_trip_count(init_solution)
            row["best_multi_visit_trip_count"] = _multi_visit_trip_count(init_solution)

            if init_solution.has_multi_visit_trip():
                row["best_multi_fitness"] = float(init_eval.objective)
                row["best_multi_solution"] = _to_one_line_solution(init_solution)
                row["best_multi_drone_trip_count"] = _drone_trip_count(init_solution)
                row["best_multi_multi_visit_trip_count"] = _multi_visit_trip_count(init_solution)
            else:
                row["best_multi_fitness"] = ""
                row["best_multi_solution"] = "None"
                row["best_multi_drone_trip_count"] = 0
                row["best_multi_multi_visit_trip_count"] = 0

            row["ats_segments_run"] = 0
            row["ats_diversification_rounds"] = 0
            row["ats_time_limit_reached"] = True
            row["ats_elapsed_sec"] = 0.0
            row["status"] = "OK"

            finished_utc = _utc_now_iso()
            row["finished_utc"] = finished_utc
            row["wall_time_sec"] = round(time.perf_counter() - wall_started, 3)
            return row

        params = AtsParams(
            nimp=int(nimp),
            seg=int(seg),
            div=int(div),
            seed=int(seed),
            max_runtime_sec=float(remaining_runtime),
            truck_max_neighbors=300,
            drone_max_neighbors=120,
            use_drone_refine=True,
        )
        ats_result = adaptive_tabu_search(
            data=data,
            params=params,
            initial_solution=init_solution,
            verbose=verbose,
        )

        row["best_fitness"] = float(ats_result.best_eval.objective)
        row["best_solution"] = _to_one_line_solution(ats_result.best_solution)
        row["best_drone_trip_count"] = _drone_trip_count(ats_result.best_solution)
        row["best_multi_visit_trip_count"] = _multi_visit_trip_count(ats_result.best_solution)

        if ats_result.best_multi_visit_solution is not None and ats_result.best_multi_visit_eval is not None:
            row["best_multi_fitness"] = float(ats_result.best_multi_visit_eval.objective)
            row["best_multi_solution"] = _to_one_line_solution(ats_result.best_multi_visit_solution)
            row["best_multi_drone_trip_count"] = _drone_trip_count(ats_result.best_multi_visit_solution)
            row["best_multi_multi_visit_trip_count"] = _multi_visit_trip_count(ats_result.best_multi_visit_solution)
        else:
            row["best_multi_fitness"] = ""
            row["best_multi_solution"] = "None"
            row["best_multi_drone_trip_count"] = 0
            row["best_multi_multi_visit_trip_count"] = 0

        row["ats_segments_run"] = int(ats_result.segments_run)
        row["ats_diversification_rounds"] = int(ats_result.diversification_rounds)
        row["ats_time_limit_reached"] = bool(ats_result.time_limit_reached)
        row["ats_elapsed_sec"] = round(float(ats_result.elapsed_sec), 3)
        row["status"] = "OK"
    except Exception as exc:  # noqa: BLE001
        row["status"] = "ERROR"
        row["error"] = f"{type(exc).__name__}: {exc}"

    finished_utc = _utc_now_iso()
    row["finished_utc"] = finished_utc
    row["wall_time_sec"] = round(time.perf_counter() - wall_started, 3)
    return row


def _run_one_job_task(args: Tuple[Dict[str, Any], float, int, int, int, bool, int, str]) -> Dict[str, Any]:
    job, max_runtime_sec, nimp, seg, div, verbose, group_id, group_key = args
    row = run_one_job(
        job=job,
        max_runtime_sec=max_runtime_sec,
        nimp=nimp,
        seg=seg,
        div=div,
        verbose=verbose,
    )
    row["group_id"] = int(group_id)
    row["group_key"] = str(group_key)
    return row


def run_group_jobs(
    group: Dict[str, Any],
    max_runtime_sec: float,
    nimp: int,
    seg: int,
    div: int,
    parallel_runs: int,
    verbose: bool = False,
) -> List[Dict[str, Any]]:
    group_jobs = list(group.get("jobs", []))
    group_id = int(group["group_id"])
    group_key = str(group.get("group_key", ""))
    if not group_jobs:
        return []

    workers = max(1, min(int(parallel_runs), len(group_jobs)))
    task_args = [
        (
            job,
            max_runtime_sec,
            nimp,
            seg,
            div,
            verbose,
            group_id,
            group_key,
        )
        for job in group_jobs
    ]

    rows: List[Dict[str, Any]] = []
    if workers == 1:
        for ta in task_args:
            rows.append(_run_one_job_task(ta))
    else:
        with ProcessPoolExecutor(max_workers=workers) as executor:
            future_map = {executor.submit(_run_one_job_task, ta): ta for ta in task_args}
            for future in as_completed(future_map):
                rows.append(future.result())

    rows.sort(key=lambda r: (int(r.get("run_index", 0)), int(r.get("job_id", 0))))
    return rows


def _ensure_parent(path: Path) -> None:
    if path.parent and not path.parent.exists():
        path.parent.mkdir(parents=True, exist_ok=True)


def _write_rows_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    _ensure_parent(path)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in CSV_FIELDS})


def _write_json(path: Path, content: Any) -> None:
    _ensure_parent(path)
    path.write_text(json.dumps(content, ensure_ascii=False, indent=2), encoding="utf-8")


def emit_matrix(config_path: Path, out_path: Path, max_jobs: int) -> int:
    jobs = _parse_jobs_from_yaml(config_path)
    groups = _group_jobs_by_instance_A_L(jobs)
    if max_jobs > 0:
        groups = groups[: max(0, int(max_jobs))]
    matrix = [{"group_id": int(group["group_id"])} for group in groups]
    _ensure_parent(out_path)
    out_path.write_text(json.dumps(matrix, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return len(matrix)


def merge_result_jsons(input_dir: Path, output_csv: Path) -> int:
    json_files = sorted(input_dir.rglob("*.json"))
    rows: List[Dict[str, Any]] = []
    for path in json_files:
        try:
            content = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(content, dict) and "job_id" in content:
                rows.append(content)
            elif isinstance(content, dict) and isinstance(content.get("rows"), list):
                for item in content["rows"]:
                    if isinstance(item, dict) and "job_id" in item:
                        rows.append(item)
            elif isinstance(content, list):
                for item in content:
                    if isinstance(item, dict) and "job_id" in item:
                        rows.append(item)
        except Exception:
            continue

    def _sort_key(r: Dict[str, Any]) -> Tuple[int, int, str]:
        try:
            jid = int(r.get("job_id", 0))
        except Exception:
            jid = 0
        try:
            gid = int(r.get("group_id", 0))
        except Exception:
            gid = 0
        return gid, jid, str(r.get("instance", ""))

    rows.sort(key=_sort_key)
    _write_rows_csv(output_csv, rows)
    return len(rows)


def build_batch_init_like_summary(input_csv: Path, output_csv: Path) -> int:
    if not input_csv.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    with input_csv.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    data_cache: Dict[Tuple[str, int, int], Any] = {}
    out_rows: List[Dict[str, Any]] = []

    for row in rows:
        instance = str(row.get("instance", "")).strip()
        a_val = int(float(row.get("A", 0) or 0))
        l_val = int(float(row.get("L", 0) or 0))

        key = (instance, a_val, l_val)
        data = data_cache.get(key)
        if data is None and instance:
            try:
                instance_path = _resolve_instance_path(instance)
                data = read_data_file(instance_path)
                data.drone_capacity = float(a_val)
                data.drone_limit_time = float(l_val)
                data_cache[key] = data
            except Exception:
                data = None

        best_sol = _parse_solution_text(row.get("best_solution", ""))
        best_multi_sol = _parse_solution_text(row.get("best_multi_solution", ""))

        best_fitness = _safe_float(row.get("best_fitness", ""))
        best_multi_fitness = _safe_float(row.get("best_multi_fitness", ""))

        if data is not None:
            obj = _objective_or_none(best_sol, data)
            if obj is not None:
                best_fitness = obj
            mv_obj = _objective_or_none(best_multi_sol, data)
            if mv_obj is not None:
                best_multi_fitness = mv_obj

        best_drone_trip_count = _drone_trip_count(best_sol)
        best_multi_visit_trip_count = _multi_visit_trip_count(best_sol)
        best_avg_customers_per_trip = _avg_customers_per_trip(best_sol)
        best_drone_avg_trip_time = _avg_drone_trip_energy_time(best_sol, data) if data is not None else 0.0

        best_multi_drone_trip_count = _drone_trip_count(best_multi_sol)
        best_multi_multi_visit_trip_count = _multi_visit_trip_count(best_multi_sol)
        best_multi_avg_customers_per_trip = _avg_customers_per_trip(best_multi_sol)
        best_multi_drone_avg_trip_time = _avg_drone_trip_energy_time(best_multi_sol, data) if data is not None else 0.0

        out_rows.append(
            {
                "job_id": row.get("job_id", ""),
                "instance": instance,
                "A": a_val,
                "L": l_val,
                "run_index": row.get("run_index", ""),
                "best_fitness": "" if best_fitness is None else best_fitness,
                "best_solution": row.get("best_solution", ""),
                "best_multi_fitness": "" if best_multi_fitness is None else best_multi_fitness,
                "best_multi_solution": row.get("best_multi_solution", ""),
                "best_drone_avg_trip_time": best_drone_avg_trip_time,
                "best_multi_visit_trip_count": best_multi_visit_trip_count,
                "best_drone_trip_count": best_drone_trip_count,
                "best_avg_customers_per_trip": best_avg_customers_per_trip,
                "best_multi_drone_avg_trip_time": best_multi_drone_avg_trip_time,
                "best_multi_multi_visit_trip_count": best_multi_multi_visit_trip_count,
                "best_multi_drone_trip_count": best_multi_drone_trip_count,
                "best_multi_avg_customers_per_trip": best_multi_avg_customers_per_trip,
                "best_solution_file": row.get("best_solution_file", ""),
                "best_multi_solution_file": row.get("best_multi_solution_file", ""),
                "log": row.get("error", "") if str(row.get("status", "")) != "OK" else "",
            }
        )

    _ensure_parent(output_csv)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_BATCH_INIT_FIELDS)
        writer.writeheader()
        for row in out_rows:
            writer.writerow({k: row.get(k, "") for k in SUMMARY_BATCH_INIT_FIELDS})
    return len(out_rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run jobs from jobs_missing_instances_A4A8_L60.yml using Source_revised2 ATS "
            "(nimp/seg/div + runtime limit) with feasible-only updates."
        )
    )
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "jobs_missing_instances_A4A8_L60.yml")
    parser.add_argument("--emit-matrix", type=Path, default=None, help="Write GitHub matrix JSON then exit.")
    parser.add_argument(
        "--max-jobs",
        type=int,
        default=0,
        help="Limit first N grouped jobs (instance,A,L) when emitting/running all (0=all).",
    )
    parser.add_argument("--job-id", type=int, default=0, help="Run one specific job id.")
    parser.add_argument("--group-id", type=int, default=0, help="Run one grouped job by (instance,A,L).")
    parser.add_argument("--run-all", action="store_true", help="Run all jobs from config.")
    parser.add_argument("--output-json", type=Path, default=None, help="Output JSON file for a single job.")
    parser.add_argument("--output-csv", type=Path, default=None, help="Output CSV path.")
    parser.add_argument("--merge-results-dir", type=Path, default=None, help="Merge all result JSON files under this dir to CSV.")
    parser.add_argument(
        "--output-batch-init-summary",
        type=Path,
        default=None,
        help="Optional summary CSV with full columns like batch_init.csv.",
    )
    parser.add_argument("--parallel-runs", type=int, default=2, help="Parallel processes per grouped job.")
    parser.add_argument("--max-runtime-sec", type=float, default=7200.0)
    parser.add_argument("--nimp", type=int, default=50)
    parser.add_argument("--seg", type=int, default=12)
    parser.add_argument("--div", type=int, default=3)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    config_path = args.config.resolve()

    if args.emit_matrix is not None:
        total = emit_matrix(config_path, args.emit_matrix.resolve(), args.max_jobs)
        print(f"matrix_jobs={total}")
        return

    if args.merge_results_dir is not None:
        if args.output_csv is None:
            raise ValueError("--output-csv is required with --merge-results-dir")
        total = merge_result_jsons(args.merge_results_dir.resolve(), args.output_csv.resolve())
        print(f"merged_rows={total}")
        print(f"output_csv={args.output_csv.resolve()}")
        if args.output_batch_init_summary is not None:
            count = build_batch_init_like_summary(args.output_csv.resolve(), args.output_batch_init_summary.resolve())
            print(f"batch_init_summary_rows={count}")
            print(f"batch_init_summary_csv={args.output_batch_init_summary.resolve()}")
        return

    jobs = _parse_jobs_from_yaml(config_path)
    groups = _group_jobs_by_instance_A_L(jobs)
    if args.max_jobs > 0 and args.run_all:
        groups = groups[: max(0, int(args.max_jobs))]

    all_rows: List[Dict[str, Any]] = []
    selected_group: Optional[Dict[str, Any]] = None

    if args.group_id > 0:
        matched = [g for g in groups if int(g["group_id"]) == int(args.group_id)]
        if not matched:
            raise ValueError(f"group_id={args.group_id} not found in {config_path}")
        selected_group = matched[0]
        groups_to_run = [selected_group]
    elif args.job_id > 0:
        selected_jobs = [j for j in jobs if int(j["id"]) == int(args.job_id)]
        if not selected_jobs:
            raise ValueError(f"job_id={args.job_id} not found in {config_path}")
        solo = selected_jobs[0]
        groups_to_run = [
            {
                "group_id": 0,
                "group_key": f"{solo['instance']}|A={solo['A']}|L={solo['L']}|single_job",
                "instance": solo["instance"],
                "A": solo["A"],
                "L": solo["L"],
                "run_indexes": [solo["run_index"]],
                "jobs": [solo],
            }
        ]
    elif args.run_all:
        groups_to_run = groups
    else:
        raise ValueError(
            "Please use one mode: --emit-matrix, --merge-results-dir, --group-id, --job-id, or --run-all"
        )

    for idx, group in enumerate(groups_to_run, start=1):
        print(
            f"[{idx}/{len(groups_to_run)}] group_id={group['group_id']} "
            f"instance={group['instance']} A={group['A']} L={group['L']} "
            f"runs={group.get('run_indexes', [])} parallel_runs={max(1, int(args.parallel_runs))}"
        )
        rows = run_group_jobs(
            group=group,
            max_runtime_sec=max(1.0, float(args.max_runtime_sec)),
            nimp=max(1, int(args.nimp)),
            seg=max(1, int(args.seg)),
            div=max(1, int(args.div)),
            parallel_runs=max(1, int(args.parallel_runs)),
            verbose=bool(args.verbose),
        )
        all_rows.extend(rows)
        for row in rows:
            print(
                f"  run_index={row.get('run_index')} status={row.get('status')} "
                f"best_fitness={row.get('best_fitness')} error={row.get('error', '')}"
            )

    all_rows.sort(key=lambda r: (int(r.get("group_id", 0)), int(r.get("run_index", 0)), int(r.get("job_id", 0))))

    if args.output_json is not None:
        out_payload: Any
        if selected_group is not None:
            out_payload = {
                "group_id": int(selected_group["group_id"]),
                "group_key": str(selected_group["group_key"]),
                "instance": str(selected_group["instance"]),
                "A": int(selected_group["A"]),
                "L": int(selected_group["L"]),
                "run_indexes": list(selected_group.get("run_indexes", [])),
                "rows": all_rows,
            }
        elif len(all_rows) == 1:
            out_payload = all_rows[0]
        else:
            out_payload = {"rows": all_rows}
        _write_json(args.output_json.resolve(), out_payload)
        print(f"output_json={args.output_json.resolve()}")

    if args.output_csv is not None:
        _write_rows_csv(args.output_csv.resolve(), all_rows)
        print(f"output_csv={args.output_csv.resolve()}")
        if args.output_batch_init_summary is not None:
            count = build_batch_init_like_summary(args.output_csv.resolve(), args.output_batch_init_summary.resolve())
            print(f"batch_init_summary_rows={count}")
            print(f"batch_init_summary_csv={args.output_batch_init_summary.resolve()}")


if __name__ == "__main__":
    main()
