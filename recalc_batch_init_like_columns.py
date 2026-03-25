from __future__ import annotations

import argparse
import ast
import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from Source_revised2 import Solution, evaluate_fitness, read_data_file

PROJECT_ROOT = Path(__file__).resolve().parent


def _parse_solution_text(raw: Any) -> Optional[Solution]:
    text = str(raw or "").strip()
    if not text or text.lower() in {"none", "null", "nan"}:
        return None

    for parser in (json.loads, ast.literal_eval):
        try:
            parsed = parser(text)
            return Solution.from_legacy(parsed)
        except Exception:
            continue
    return None


def _resolve_instance_path(instance_raw: str) -> Path:
    normalized = str(instance_raw or "").strip().replace("\\", "/")
    p = Path(normalized)
    if p.is_absolute():
        return p.resolve()
    return (PROJECT_ROOT / normalized).resolve()


def _fmt_float(value: Optional[float], digits: int = 9) -> str:
    if value is None:
        return ""
    s = f"{float(value):.{digits}f}".rstrip("0").rstrip(".")
    return s if s else "0"


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


def _eval_solution(solution: Optional[Solution], data) -> Optional[Any]:
    if solution is None:
        return None
    ev = evaluate_fitness(solution, data)
    if not ev.feasible:
        return None
    return ev


def recalc_csv(input_csv: Path, output_csv: Path) -> Dict[str, Any]:
    with input_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    required_cols = [
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
    ]
    for col in required_cols:
        if col not in fieldnames:
            fieldnames.append(col)

    data_cache: Dict[Tuple[str, str, str], Any] = {}

    rows_total = len(rows)
    best_ok = 0
    best_multi_ok = 0

    for row in rows:
        instance = str(row.get("instance", "")).strip()
        a_txt = str(row.get("A", "") or "").strip()
        l_txt = str(row.get("L", "") or "").strip()

        key = (instance, a_txt, l_txt)
        data = data_cache.get(key)
        if data is None:
            p = _resolve_instance_path(instance)
            data = read_data_file(p)
            data.drone_capacity = float(a_txt or 0.0)
            data.drone_limit_time = float(l_txt or 0.0)
            data_cache[key] = data

        best_sol = _parse_solution_text(row.get("best_solution", ""))
        best_multi_sol = _parse_solution_text(row.get("best_multi_solution", ""))

        best_ev = _eval_solution(best_sol, data)
        best_multi_ev = _eval_solution(best_multi_sol, data)

        if best_ev is not None:
            row["best_fitness"] = _fmt_float(float(best_ev.objective))
            vals = list(best_ev.drone_flight_wait_energy_time.values())
            row["best_drone_avg_trip_time"] = _fmt_float(sum(vals) / len(vals) if vals else 0.0)
            best_ok += 1
        else:
            row["best_fitness"] = ""
            row["best_drone_avg_trip_time"] = ""

        row["best_multi_visit_trip_count"] = str(_multi_visit_trip_count(best_sol))
        row["best_drone_trip_count"] = str(_drone_trip_count(best_sol))
        row["best_avg_customers_per_trip"] = _fmt_float(_avg_customers_per_trip(best_sol))

        if best_multi_ev is not None:
            row["best_multi_fitness"] = _fmt_float(float(best_multi_ev.objective))
            mv_vals = list(best_multi_ev.drone_flight_wait_energy_time.values())
            row["best_multi_drone_avg_trip_time"] = _fmt_float(sum(mv_vals) / len(mv_vals) if mv_vals else 0.0)
            best_multi_ok += 1
        else:
            row["best_multi_fitness"] = ""
            row["best_multi_drone_avg_trip_time"] = ""

        row["best_multi_multi_visit_trip_count"] = str(_multi_visit_trip_count(best_multi_sol))
        row["best_multi_drone_trip_count"] = str(_drone_trip_count(best_multi_sol))
        row["best_multi_avg_customers_per_trip"] = _fmt_float(_avg_customers_per_trip(best_multi_sol))

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})

    return {
        "rows_total": rows_total,
        "best_solution_feasible_rows": best_ok,
        "best_multi_solution_feasible_rows": best_multi_ok,
        "output_csv": str(output_csv.resolve()),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Recalculate batch_init-like columns from best_solution and best_multi_solution"
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    args = parser.parse_args()

    summary = recalc_csv(args.input_csv.resolve(), args.output_csv.resolve())
    for k, v in summary.items():
        print(f"{k}={v}")


if __name__ == "__main__":
    main()
