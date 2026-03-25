from __future__ import annotations

import argparse
import ast
import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from Source_revised2 import Solution, evaluate_fitness, read_data_file

PROJECT_ROOT = Path(__file__).resolve().parent

OUTPUT_FIELDS: List[str] = [
    "instance",
    "A",
    "L",
    "best_solution",
    "best_multi_solution",
    "best_fitness",
    "best_multi_fitness",
]



def _find_value(row: Dict[str, Any], keys: Iterable[str]) -> str:
    for key in keys:
        if key in row and row.get(key) is not None:
            text = str(row.get(key)).strip()
            if text:
                return text
    return ""



def _to_float_text(raw: str) -> str:
    text = str(raw or "").strip()
    if not text:
        return ""
    try:
        val = float(text)
        if val.is_integer():
            return str(int(val))
        s = f"{val:.9f}".rstrip("0").rstrip(".")
        return s if s else "0"
    except Exception:
        return ""



def _fmt_float(value: Optional[float]) -> str:
    if value is None:
        return ""
    s = f"{float(value):.9f}".rstrip("0").rstrip(".")
    return s if s else "0"



def _parse_solution(raw: Any) -> Optional[Solution]:
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



def _solution_one_line(sol: Optional[Solution]) -> str:
    if sol is None:
        return ""
    return json.dumps(sol.to_legacy(), ensure_ascii=False, separators=(",", ":"))



def _resolve_instance_path(instance_raw: str) -> Path:
    normalized = str(instance_raw or "").strip().replace("\\", "/")
    p = Path(normalized)
    if p.is_absolute():
        return p.resolve()
    return (PROJECT_ROOT / normalized).resolve()



def _collect_input_files(folder_csv_dir: Path, extra_files: List[Path]) -> List[Path]:
    files: List[Path] = []
    if folder_csv_dir.exists():
        files.extend(sorted(folder_csv_dir.glob("*.csv")))
    for p in extra_files:
        if p.exists():
            files.append(p)
    # unique while preserving order
    seen = set()
    out: List[Path] = []
    for f in files:
        key = str(f.resolve()).lower()
        if key not in seen:
            seen.add(key)
            out.append(f.resolve())
    return out



def build_summary(output_csv: Path, folder_csv_dir: Path, extra_files: List[Path]) -> Dict[str, Any]:
    files = _collect_input_files(folder_csv_dir, extra_files)
    if not files:
        raise FileNotFoundError("No input CSV files found.")

    data_cache: Dict[Tuple[str, str, str], Any] = {}

    raw_rows = 0
    kept_rows = 0
    valid_best = 0
    valid_multi = 0

    out_rows: List[Dict[str, str]] = []

    for csv_file in files:
        with csv_file.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_rows += 1

                instance = _find_value(row, ["instance", "Instance", "resolved_instance", "data_file"])
                a_txt = _find_value(row, ["A", "a", "drone_capacity"])
                l_txt = _find_value(row, ["L", "l", "drone_limit_time"])
                a_norm = _to_float_text(a_txt)
                l_norm = _to_float_text(l_txt)

                best_sol_raw = _find_value(row, ["best_solution", "best_sol"])
                best_multi_raw = _find_value(row, ["best_multi_solution", "best_multi_visit_sol", "best_multivisit_solution"])

                best_sol = _parse_solution(best_sol_raw)
                best_multi_sol = _parse_solution(best_multi_raw)

                best_sol_valid: Optional[Solution] = None
                best_multi_valid: Optional[Solution] = None
                best_fit: Optional[float] = None
                best_multi_fit: Optional[float] = None

                data = None
                if instance and a_norm and l_norm:
                    key = (instance, a_norm, l_norm)
                    data = data_cache.get(key)
                    if data is None:
                        try:
                            p = _resolve_instance_path(instance)
                            data = read_data_file(p)
                            data.drone_capacity = float(a_norm)
                            data.drone_limit_time = float(l_norm)
                            data_cache[key] = data
                        except Exception:
                            data = None

                if data is not None and best_sol is not None:
                    ev = evaluate_fitness(best_sol, data)
                    if ev.feasible:
                        best_sol_valid = best_sol
                        best_fit = float(ev.objective)
                        valid_best += 1

                if data is not None and best_multi_sol is not None:
                    evm = evaluate_fitness(best_multi_sol, data)
                    if evm.feasible:
                        best_multi_valid = best_multi_sol
                        best_multi_fit = float(evm.objective)
                        valid_multi += 1

                out_rows.append(
                    {
                        "instance": instance,
                        "A": a_norm,
                        "L": l_norm,
                        "best_solution": _solution_one_line(best_sol_valid),
                        "best_multi_solution": _solution_one_line(best_multi_valid),
                        "best_fitness": _fmt_float(best_fit),
                        "best_multi_fitness": _fmt_float(best_multi_fit),
                    }
                )

    # Deduplicate exact rows.
    seen_rows = set()
    dedup_rows: List[Dict[str, str]] = []
    for row in out_rows:
        key = tuple(row.get(c, "") for c in OUTPUT_FIELDS)
        if key in seen_rows:
            continue
        seen_rows.add(key)
        dedup_rows.append(row)

    kept_rows = len(dedup_rows)

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        for row in dedup_rows:
            writer.writerow({k: row.get(k, "") for k in OUTPUT_FIELDS})

    return {
        "input_files": len(files),
        "raw_rows": raw_rows,
        "output_rows_dedup": kept_rows,
        "valid_best_solution_count": valid_best,
        "valid_best_multi_solution_count": valid_multi,
        "output_csv": str(output_csv.resolve()),
    }



def main() -> None:
    parser = argparse.ArgumentParser(description="Build summaryall.csv by validating and merging many CSV sources")
    parser.add_argument("--folder-csv-dir", type=Path, default=PROJECT_ROOT / "tổng hợp kết quả")
    parser.add_argument("--output-csv", type=Path, default=PROJECT_ROOT / "summaryall.csv")
    parser.add_argument("--extra-file", type=Path, action="append", default=[])
    args = parser.parse_args()

    summary = build_summary(
        output_csv=args.output_csv.resolve(),
        folder_csv_dir=args.folder_csv_dir.resolve(),
        extra_files=[p.resolve() for p in args.extra_file],
    )
    for k, v in summary.items():
        print(f"{k}={v}")


if __name__ == "__main__":
    main()
