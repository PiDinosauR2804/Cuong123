from __future__ import annotations

import argparse
import ast
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import datetime as dt
import hashlib
import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from Source_revised2 import (
    AtsParams,
    Solution,
    adaptive_tabu_search,
    build_initial_solution,
    evaluate_fitness,
    read_data_file,
)


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT = PROJECT_ROOT / "batch_init.csv"


DETAIL_FIELDS: List[str] = [
    "row_index",
    "job_id",
    "instance",
    "A",
    "L",
    "run_index",
    "parallel_runs",
    "attempts_ok",
    "attempts_error",
    "chosen_attempt",
    "status",
    "error",
    "attempt_index",
    "ats_seed",
    "seed_source",
    "seed_feasible",
    "seed_fitness",
    "selected_solution_source",
    "selected_fitness",
    "ats_time_limit_reached",
    "ats_elapsed_sec",
    "ats_segments_run",
    "ats_diversification_rounds",
    "nimp",
    "seg",
    "div",
    "max_runtime_sec",
    "started_utc",
    "finished_utc",
    "wall_time_sec",
]


def _utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def _resolve_instance_path(instance_raw: str) -> Path:
    normalized = str(instance_raw or "").strip().replace("\\", "/")
    p = Path(normalized)
    if p.is_absolute():
        return p.resolve()
    return (PROJECT_ROOT / normalized).resolve()


def _parse_solution_text(raw: Any) -> Optional[Solution]:
    text = str(raw or "").strip()
    if not text or text.lower() in {"none", "nan", "null"}:
        return None

    for parser in (json.loads, ast.literal_eval):
        try:
            parsed = parser(text)
            return Solution.from_legacy(parsed)
        except Exception:
            continue
    return None


def _solution_one_line(solution: Optional[Solution]) -> str:
    if solution is None:
        return ""
    return json.dumps(solution.to_legacy(), ensure_ascii=False, separators=(",", ":"))


def _drone_trip_count(solution: Optional[Solution]) -> int:
    if solution is None:
        return 0
    return len(solution.drone_queue)


def _multi_visit_trip_count(solution: Optional[Solution]) -> int:
    if solution is None:
        return 0
    return sum(1 for trip in solution.drone_queue if trip.is_multi_visit)


def _avg_customers_per_trip(solution: Optional[Solution]) -> float:
    if solution is None or not solution.drone_queue:
        return 0.0
    totals: List[int] = []
    for trip in solution.drone_queue:
        delivered = 0
        for leg in trip.legs:
            delivered += len(leg.customers)
        totals.append(delivered)
    if not totals:
        return 0.0
    return float(sum(totals) / len(totals))


def _avg_drone_trip_energy(ev: Any) -> float:
    vals = list(ev.drone_flight_wait_energy_time.values())
    if not vals:
        return 0.0
    return float(sum(vals) / len(vals))


def _fmt_float(value: Optional[float], digits: int = 9) -> str:
    if value is None:
        return ""
    s = f"{float(value):.{digits}f}"
    s = s.rstrip("0").rstrip(".")
    return s if s else "0"


def _to_float(value: Any) -> Optional[float]:
    try:
        text = str(value).strip()
        if not text:
            return None
        return float(text)
    except Exception:
        return None


def _attempt_seed(row: Dict[str, str], attempt_index: int) -> int:
    key = (
        f"{row.get('job_id','')}|{row.get('instance','')}|{row.get('A','')}|"
        f"{row.get('L','')}|{row.get('run_index','')}|attempt={attempt_index}"
    )
    digest = hashlib.sha256(key.encode("utf-8")).hexdigest()
    return int(digest[:8], 16)


def _score(ev: Any) -> Tuple[float, float]:
    return (float(ev.objective), float(sum(ev.truck_return_time.values())))


def _select_multi_solution(
    selected_sol: Solution,
    selected_eval: Any,
    ats_result: Any,
) -> Tuple[Optional[Solution], Optional[Any]]:
    candidates: List[Tuple[Solution, Any]] = []

    if selected_sol.has_multi_visit_trip():
        candidates.append((selected_sol, selected_eval))

    if ats_result.best_multi_visit_solution is not None and ats_result.best_multi_visit_eval is not None:
        candidates.append((ats_result.best_multi_visit_solution, ats_result.best_multi_visit_eval))

    if not candidates:
        return None, None

    best_sol, best_ev = candidates[0]
    for cand_sol, cand_ev in candidates[1:]:
        if _score(cand_ev) < _score(best_ev):
            best_sol, best_ev = cand_sol, cand_ev
    return best_sol, best_ev


def _append_log(old_log: str, note: str) -> str:
    base = str(old_log or "").strip()
    add = str(note or "").strip()
    if not base:
        return add
    if not add:
        return base
    return f"{base} | {add}"


def process_row(
    row_index: int,
    row: Dict[str, str],
    nimp: int,
    seg: int,
    div: int,
    max_runtime_sec: float,
    attempt_index: int = 1,
    parallel_runs: int = 1,
    ats_seed: Optional[int] = None,
) -> Tuple[Dict[str, str], Dict[str, Any]]:
    started_perf = time.perf_counter()
    started_utc = _utc_now_iso()

    detail: Dict[str, Any] = {
        "row_index": row_index,
        "job_id": row.get("job_id", ""),
        "instance": row.get("instance", ""),
        "A": row.get("A", ""),
        "L": row.get("L", ""),
        "run_index": row.get("run_index", ""),
        "parallel_runs": int(parallel_runs),
        "attempts_ok": "",
        "attempts_error": "",
        "chosen_attempt": "",
        "status": "ERROR",
        "error": "",
        "attempt_index": int(attempt_index),
        "ats_seed": "" if ats_seed is None else int(ats_seed),
        "seed_source": "",
        "seed_feasible": "",
        "seed_fitness": "",
        "selected_solution_source": "",
        "selected_fitness": "",
        "ats_time_limit_reached": "",
        "ats_elapsed_sec": "",
        "ats_segments_run": "",
        "ats_diversification_rounds": "",
        "nimp": int(nimp),
        "seg": int(seg),
        "div": int(div),
        "max_runtime_sec": float(max_runtime_sec),
        "started_utc": started_utc,
        "finished_utc": "",
        "wall_time_sec": "",
    }

    updated = dict(row)
    try:
        instance = str(row.get("instance", "")).strip()
        a_val = float(row.get("A", "0") or 0.0)
        l_val = float(row.get("L", "0") or 0.0)

        instance_path = _resolve_instance_path(instance)
        if not instance_path.exists():
            raise FileNotFoundError(f"Instance not found: {instance_path}")

        data = read_data_file(instance_path)
        data.drone_capacity = a_val
        data.drone_limit_time = l_val

        seed_sol = _parse_solution_text(row.get("best_solution", ""))
        seed_source = "input_best_solution"
        if seed_sol is None:
            seed_sol = build_initial_solution(data)
            seed_source = "fallback_build_initial_missing_seed"

        seed_eval = evaluate_fitness(seed_sol, data)
        if not seed_eval.feasible:
            seed_sol = build_initial_solution(data)
            seed_source = "fallback_build_initial_infeasible_seed"
            seed_eval = evaluate_fitness(seed_sol, data)
            if not seed_eval.feasible:
                raise RuntimeError("Cannot obtain feasible seed solution.")

        detail["seed_source"] = seed_source
        detail["seed_feasible"] = bool(seed_eval.feasible)
        detail["seed_fitness"] = float(seed_eval.objective)

        ats_result = adaptive_tabu_search(
            data=data,
            params=AtsParams(
                nimp=int(nimp),
                seg=int(seg),
                div=int(div),
                seed=ats_seed,
                max_runtime_sec=float(max_runtime_sec),
                truck_max_neighbors=300,
                drone_max_neighbors=120,
                use_drone_refine=True,
            ),
            initial_solution=seed_sol,
            verbose=False,
        )

        # As requested: even on timeout, record best-so-far solutions.
        selected_sol = ats_result.best_solution
        selected_ev = ats_result.best_eval
        if bool(ats_result.time_limit_reached):
            selected_source = "best_on_timeout"
        else:
            selected_source = "best_after_ats"

        multi_sol, multi_ev = _select_multi_solution(selected_sol, selected_ev, ats_result)

        updated["best_fitness"] = _fmt_float(float(selected_ev.objective))
        updated["best_solution"] = _solution_one_line(selected_sol)
        updated["best_multi_fitness"] = "" if multi_ev is None else _fmt_float(float(multi_ev.objective))
        updated["best_multi_solution"] = "" if multi_sol is None else _solution_one_line(multi_sol)

        updated["best_drone_avg_trip_time"] = _fmt_float(_avg_drone_trip_energy(selected_ev))
        updated["best_multi_visit_trip_count"] = str(_multi_visit_trip_count(selected_sol))
        updated["best_drone_trip_count"] = str(_drone_trip_count(selected_sol))
        updated["best_avg_customers_per_trip"] = _fmt_float(_avg_customers_per_trip(selected_sol))

        if multi_sol is None or multi_ev is None:
            updated["best_multi_drone_avg_trip_time"] = ""
            updated["best_multi_multi_visit_trip_count"] = ""
            updated["best_multi_drone_trip_count"] = ""
            updated["best_multi_avg_customers_per_trip"] = ""
        else:
            updated["best_multi_drone_avg_trip_time"] = _fmt_float(_avg_drone_trip_energy(multi_ev))
            updated["best_multi_multi_visit_trip_count"] = str(_multi_visit_trip_count(multi_sol))
            updated["best_multi_drone_trip_count"] = str(_drone_trip_count(multi_sol))
            updated["best_multi_avg_customers_per_trip"] = _fmt_float(_avg_customers_per_trip(multi_sol))

        log_note = (
            f"ATS(nimp={nimp},seg={seg},div={div},t={int(max_runtime_sec)}s,"
            f"selected={selected_source},timeout={bool(ats_result.time_limit_reached)},"
            f"attempt={attempt_index}/{parallel_runs},seed={'' if ats_seed is None else int(ats_seed)})"
        )
        updated["log"] = _append_log(updated.get("log", ""), log_note)

        detail["status"] = "OK"
        detail["selected_solution_source"] = selected_source
        detail["selected_fitness"] = float(selected_ev.objective)
        detail["ats_time_limit_reached"] = bool(ats_result.time_limit_reached)
        detail["ats_elapsed_sec"] = round(float(ats_result.elapsed_sec), 3)
        detail["ats_segments_run"] = int(ats_result.segments_run)
        detail["ats_diversification_rounds"] = int(ats_result.diversification_rounds)
    except Exception as exc:  # noqa: BLE001
        detail["status"] = "ERROR"
        detail["error"] = f"{type(exc).__name__}: {exc}"
        updated["log"] = _append_log(updated.get("log", ""), f"ERROR: {detail['error']}")

    detail["finished_utc"] = _utc_now_iso()
    detail["wall_time_sec"] = round(time.perf_counter() - started_perf, 3)
    return updated, detail


def _attempt_worker(
    args: Tuple[int, Dict[str, str], int, int, int, float, int, int, int]
) -> Tuple[Dict[str, str], Dict[str, Any]]:
    (
        row_index,
        row,
        nimp,
        seg,
        div,
        max_runtime_sec,
        attempt_index,
        parallel_runs,
        ats_seed,
    ) = args
    return process_row(
        row_index=row_index,
        row=row,
        nimp=nimp,
        seg=seg,
        div=div,
        max_runtime_sec=max_runtime_sec,
        attempt_index=attempt_index,
        parallel_runs=parallel_runs,
        ats_seed=ats_seed,
    )


def process_row_parallel(
    row_index: int,
    row: Dict[str, str],
    nimp: int,
    seg: int,
    div: int,
    max_runtime_sec: float,
    parallel_runs: int,
) -> Tuple[Dict[str, str], Dict[str, Any]]:
    runs = max(1, int(parallel_runs))
    if runs == 1:
        seed = _attempt_seed(row, 1)
        updated, detail = process_row(
            row_index=row_index,
            row=row,
            nimp=nimp,
            seg=seg,
            div=div,
            max_runtime_sec=max_runtime_sec,
            attempt_index=1,
            parallel_runs=1,
            ats_seed=seed,
        )
        detail["parallel_runs"] = 1
        detail["attempts_ok"] = 1 if detail.get("status") == "OK" else 0
        detail["attempts_error"] = 0 if detail.get("status") == "OK" else 1
        detail["chosen_attempt"] = 1 if detail.get("status") == "OK" else ""
        return updated, detail

    tasks: List[Tuple[int, Dict[str, str], int, int, int, float, int, int, int]] = []
    for attempt in range(1, runs + 1):
        tasks.append(
            (
                row_index,
                dict(row),
                int(nimp),
                int(seg),
                int(div),
                float(max_runtime_sec),
                attempt,
                runs,
                _attempt_seed(row, attempt),
            )
        )

    results: List[Tuple[Dict[str, str], Dict[str, Any]]] = []
    with ProcessPoolExecutor(max_workers=runs) as executor:
        future_map = {executor.submit(_attempt_worker, t): t for t in tasks}
        for future in as_completed(future_map):
            results.append(future.result())

    ok_results: List[Tuple[Dict[str, str], Dict[str, Any]]] = [r for r in results if r[1].get("status") == "OK"]
    err_count = len(results) - len(ok_results)

    if not ok_results:
        # Return first error attempt but annotate total failures.
        updated, detail = results[0]
        detail["parallel_runs"] = runs
        detail["attempts_ok"] = 0
        detail["attempts_error"] = err_count
        detail["chosen_attempt"] = ""
        updated["log"] = _append_log(updated.get("log", ""), f"parallel_runs={runs},all_attempts_failed")
        return updated, detail

    def _sel_key(item: Tuple[Dict[str, str], Dict[str, Any]]) -> Tuple[float, int]:
        _, det = item
        fit = _to_float(det.get("selected_fitness"))
        if fit is None:
            fit = float("inf")
        attempt = int(det.get("attempt_index", 10**9))
        return fit, attempt

    best_updated, best_detail = min(ok_results, key=_sel_key)

    # Best multi-solution is selected independently across successful attempts.
    best_multi_pick: Optional[Tuple[Dict[str, str], Dict[str, Any]]] = None
    for up, det in ok_results:
        mv_fit = _to_float(up.get("best_multi_fitness"))
        if mv_fit is None:
            continue
        if best_multi_pick is None:
            best_multi_pick = (up, det)
            continue
        cur_best = _to_float(best_multi_pick[0].get("best_multi_fitness"))
        if cur_best is None or mv_fit < cur_best:
            best_multi_pick = (up, det)

    chosen_attempt = int(best_detail.get("attempt_index", 1))
    result_updated = dict(best_updated)
    result_detail = dict(best_detail)
    result_detail["parallel_runs"] = runs
    result_detail["attempts_ok"] = len(ok_results)
    result_detail["attempts_error"] = err_count
    result_detail["chosen_attempt"] = chosen_attempt

    if best_multi_pick is not None:
        mv_up, mv_det = best_multi_pick
        result_updated["best_multi_fitness"] = mv_up.get("best_multi_fitness", "")
        result_updated["best_multi_solution"] = mv_up.get("best_multi_solution", "")
        result_updated["best_multi_drone_avg_trip_time"] = mv_up.get("best_multi_drone_avg_trip_time", "")
        result_updated["best_multi_multi_visit_trip_count"] = mv_up.get("best_multi_multi_visit_trip_count", "")
        result_updated["best_multi_drone_trip_count"] = mv_up.get("best_multi_drone_trip_count", "")
        result_updated["best_multi_avg_customers_per_trip"] = mv_up.get("best_multi_avg_customers_per_trip", "")
        result_updated["log"] = _append_log(
            result_updated.get("log", ""),
            f"parallel_runs={runs},chosen_attempt={chosen_attempt},"
            f"best_multi_from_attempt={mv_det.get('attempt_index', '')}",
        )
    else:
        result_updated["log"] = _append_log(
            result_updated.get("log", ""),
            f"parallel_runs={runs},chosen_attempt={chosen_attempt},best_multi_from_attempt=",
        )

    return result_updated, result_detail


def emit_matrix(input_csv: Path, output_json: Path, max_jobs: int) -> int:
    with input_csv.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    total = len(rows)
    end = total if max_jobs <= 0 else min(total, max(0, int(max_jobs)))
    matrix = [{"row_index": i} for i in range(1, end + 1)]
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(matrix, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return len(matrix)


def run_one_row(
    input_csv: Path,
    row_index: int,
    nimp: int,
    seg: int,
    div: int,
    max_runtime_sec: float,
    parallel_runs: int,
) -> Dict[str, Any]:
    with input_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    if row_index < 1 or row_index > len(rows):
        raise ValueError(f"row_index={row_index} is out of range 1..{len(rows)}")

    row = rows[row_index - 1]
    updated, detail = process_row_parallel(
        row_index=row_index,
        row=row,
        nimp=nimp,
        seg=seg,
        div=div,
        max_runtime_sec=max_runtime_sec,
        parallel_runs=parallel_runs,
    )
    return {
        "row_index": row_index,
        "status": detail.get("status", "ERROR"),
        "fieldnames": fieldnames,
        "updated_row": updated,
        "detail": detail,
    }


def merge_results(
    input_csv: Path,
    result_dir: Path,
    output_csv: Path,
    output_detail_csv: Path,
) -> Tuple[int, int, int]:
    with input_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        base_rows = list(reader)

    payloads: Dict[int, Dict[str, Any]] = {}
    for path in sorted(result_dir.rglob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                continue
            idx = int(data.get("row_index", 0))
            if idx <= 0:
                continue
            payloads[idx] = data
        except Exception:
            continue

    details: List[Dict[str, Any]] = []
    ok = 0
    err = 0
    merged_rows: List[Dict[str, Any]] = []
    for idx, base in enumerate(base_rows, start=1):
        payload = payloads.get(idx)
        if payload is None:
            miss = dict(base)
            miss["log"] = _append_log(miss.get("log", ""), "ERROR: missing job result")
            merged_rows.append(miss)
            details.append(
                {
                    "row_index": idx,
                    "job_id": base.get("job_id", ""),
                    "instance": base.get("instance", ""),
                    "A": base.get("A", ""),
                    "L": base.get("L", ""),
                    "run_index": base.get("run_index", ""),
                    "parallel_runs": "",
                    "attempts_ok": "",
                    "attempts_error": "",
                    "chosen_attempt": "",
                    "status": "ERROR",
                    "error": "missing job result",
                    "attempt_index": "",
                    "ats_seed": "",
                    "seed_source": "",
                    "seed_feasible": "",
                    "seed_fitness": "",
                    "selected_solution_source": "",
                    "selected_fitness": "",
                    "ats_time_limit_reached": "",
                    "ats_elapsed_sec": "",
                    "ats_segments_run": "",
                    "ats_diversification_rounds": "",
                    "nimp": "",
                    "seg": "",
                    "div": "",
                    "max_runtime_sec": "",
                    "started_utc": "",
                    "finished_utc": "",
                    "wall_time_sec": "",
                }
            )
            err += 1
            continue

        updated = payload.get("updated_row")
        detail = payload.get("detail")
        if not isinstance(updated, dict):
            updated = dict(base)
            updated["log"] = _append_log(updated.get("log", ""), "ERROR: invalid payload updated_row")
        merged_rows.append(updated)

        if isinstance(detail, dict):
            details.append(detail)
            if detail.get("status") == "OK":
                ok += 1
            else:
                err += 1
        else:
            err += 1

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in merged_rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})

    output_detail_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_detail_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=DETAIL_FIELDS)
        writer.writeheader()
        for row in sorted(details, key=lambda d: int(d.get("row_index", 0))):
            writer.writerow({k: row.get(k, "") for k in DETAIL_FIELDS})

    return len(merged_rows), ok, err


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Improve batch_init.csv solutions using Source_revised2 ATS "
            "(nimp=50, seg=4, div<=3, timeout 120min)."
        )
    )
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--emit-matrix", type=Path, default=None)
    parser.add_argument("--max-jobs", type=int, default=0)

    parser.add_argument("--row-index", type=int, default=0)
    parser.add_argument("--output-json", type=Path, default=None)

    parser.add_argument("--merge-results-dir", type=Path, default=None)
    parser.add_argument("--output-csv", type=Path, default=None)
    parser.add_argument("--output-detail-csv", type=Path, default=None)

    parser.add_argument("--nimp", type=int, default=50)
    parser.add_argument("--seg", type=int, default=4)
    parser.add_argument("--div", type=int, default=3)
    parser.add_argument("--max-runtime-sec", type=float, default=7200.0)
    parser.add_argument("--parallel-runs", type=int, default=1)
    args = parser.parse_args()

    input_csv = args.input_csv.resolve()
    if not input_csv.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    if args.emit_matrix is not None:
        count = emit_matrix(input_csv, args.emit_matrix.resolve(), int(args.max_jobs))
        print(f"matrix_jobs={count}")
        return

    if args.merge_results_dir is not None:
        if args.output_csv is None or args.output_detail_csv is None:
            raise ValueError("--output-csv and --output-detail-csv are required with --merge-results-dir")
        total, ok, err = merge_results(
            input_csv=input_csv,
            result_dir=args.merge_results_dir.resolve(),
            output_csv=args.output_csv.resolve(),
            output_detail_csv=args.output_detail_csv.resolve(),
        )
        print(f"merged_rows={total}")
        print(f"ok_rows={ok}")
        print(f"error_rows={err}")
        print(f"output_csv={args.output_csv.resolve()}")
        print(f"output_detail_csv={args.output_detail_csv.resolve()}")
        return

    if args.row_index <= 0:
        raise ValueError("Please provide --row-index, or use --emit-matrix / --merge-results-dir.")

    payload = run_one_row(
        input_csv=input_csv,
        row_index=int(args.row_index),
        nimp=max(1, int(args.nimp)),
        seg=max(1, int(args.seg)),
        div=max(1, int(args.div)),
        max_runtime_sec=max(1.0, float(args.max_runtime_sec)),
        parallel_runs=max(1, int(args.parallel_runs)),
    )

    if args.output_json is None:
        raise ValueError("--output-json is required with --row-index")
    out = args.output_json.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"row_index={payload['row_index']}")
    print(f"status={payload.get('status')}")
    print(f"output_json={out}")


if __name__ == "__main__":
    main()
