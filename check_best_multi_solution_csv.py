from __future__ import annotations

import argparse
import ast
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from Source_revised2 import Solution, evaluate_fitness, read_data_file

PROJECT_ROOT = Path(__file__).resolve().parent


def _parse_solution(raw: Any) -> Optional[Solution]:
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


def _to_float(raw: Any) -> Optional[float]:
    try:
        text = str(raw or "").strip()
        if not text:
            return None
        return float(text)
    except Exception:
        return None


def _fmt_float(v: Optional[float], digits: int = 9) -> str:
    if v is None:
        return ""
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return str(v)
    s = f"{float(v):.{digits}f}".rstrip("0").rstrip(".")
    return s if s else "0"


def _resolve_instance_path(instance_raw: str) -> Path:
    normalized = str(instance_raw or "").strip().replace("\\", "/")
    p = Path(normalized)
    if p.is_absolute():
        return p.resolve()
    return (PROJECT_ROOT / normalized).resolve()


def check_file(input_csv: Path, output_csv: Path) -> Dict[str, Any]:
    with input_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    add_cols = [
        "multi_sol_check_status",
        "multi_sol_check_error",
        "multi_sol_has_multi_visit",
        "multi_sol_recalc_feasible",
        "multi_sol_recalc_fitness",
        "multi_sol_old_fitness",
        "multi_sol_fitness_diff",
        "multi_sol_fitness_match",
    ]
    out_fields = fieldnames + [c for c in add_cols if c not in fieldnames]

    cache: Dict[Tuple[str, str, str], Any] = {}

    total = len(rows)
    with_sol = 0
    parse_fail = 0
    feasible = 0
    infeasible = 0
    match = 0
    mismatch = 0
    no_old_fit = 0

    out_rows = []
    for idx, row in enumerate(rows, start=1):
        out = dict(row)
        out["multi_sol_check_status"] = ""
        out["multi_sol_check_error"] = ""
        out["multi_sol_has_multi_visit"] = ""
        out["multi_sol_recalc_feasible"] = ""
        out["multi_sol_recalc_fitness"] = ""
        out["multi_sol_old_fitness"] = ""
        out["multi_sol_fitness_diff"] = ""
        out["multi_sol_fitness_match"] = ""

        try:
            sol_raw = row.get("best_multi_solution", "")
            old_fit = _to_float(row.get("best_multi_fitness", ""))
            out["multi_sol_old_fitness"] = _fmt_float(old_fit)

            sol = _parse_solution(sol_raw)
            if sol is None:
                out["multi_sol_check_status"] = "EMPTY_OR_PARSE_FAIL"
                parse_fail += 1
                out_rows.append(out)
                continue

            with_sol += 1
            out["multi_sol_has_multi_visit"] = str(bool(sol.has_multi_visit_trip()))

            key = (str(row.get("instance", "")).strip(), str(row.get("A", "")).strip(), str(row.get("L", "")).strip())
            data = cache.get(key)
            if data is None:
                p = _resolve_instance_path(key[0])
                data = read_data_file(p)
                data.drone_capacity = float(key[1] or 0)
                data.drone_limit_time = float(key[2] or 0)
                cache[key] = data

            ev = evaluate_fitness(sol, data)
            out["multi_sol_recalc_feasible"] = str(bool(ev.feasible))
            out["multi_sol_recalc_fitness"] = _fmt_float(ev.objective if ev.feasible else float("inf"))

            if ev.feasible:
                feasible += 1
                out["multi_sol_check_status"] = "FEASIBLE"
            else:
                infeasible += 1
                out["multi_sol_check_status"] = "INFEASIBLE"
                out["multi_sol_check_error"] = " | ".join(ev.violations[:5])

            if old_fit is None:
                out["multi_sol_fitness_match"] = "NO_OLD_FIT"
                no_old_fit += 1
            elif not ev.feasible:
                out["multi_sol_fitness_match"] = "OLD_SET_BUT_NOW_INFEASIBLE"
                mismatch += 1
            else:
                diff = float(ev.objective) - float(old_fit)
                out["multi_sol_fitness_diff"] = _fmt_float(diff)
                ok = abs(diff) <= 1e-6
                out["multi_sol_fitness_match"] = str(ok)
                if ok:
                    match += 1
                else:
                    mismatch += 1

        except Exception as exc:  # noqa: BLE001
            out["multi_sol_check_status"] = "ERROR"
            out["multi_sol_check_error"] = f"{type(exc).__name__}: {exc}"
            mismatch += 1

        out_rows.append(out)

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=out_fields)
        writer.writeheader()
        writer.writerows(out_rows)

    return {
        "rows_total": total,
        "rows_with_multi_solution": with_sol,
        "rows_empty_or_parse_fail": parse_fail,
        "rows_feasible": feasible,
        "rows_infeasible": infeasible,
        "rows_match": match,
        "rows_mismatch": mismatch,
        "rows_no_old_fitness": no_old_fit,
        "output_csv": str(output_csv.resolve()),
    }



def main() -> None:
    parser = argparse.ArgumentParser(description="Check best_multi_solution feasibility and recalc best_multi_fitness")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    args = parser.parse_args()

    summary = check_file(args.input_csv.resolve(), args.output_csv.resolve())
    for k, v in summary.items():
        print(f"{k}={v}")


if __name__ == "__main__":
    main()
