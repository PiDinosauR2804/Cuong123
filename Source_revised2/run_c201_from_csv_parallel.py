from __future__ import annotations

import argparse
import csv
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Dict, List


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from run_source_revised2_segment_from_csv import (  # noqa: E402
    DEFAULT_INPUT_CSV,
    process_row,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run Source_revised2 ATS (1 segment, unlimited neighbors) from c201 CSV."
    )
    parser.add_argument(
        "--input-csv",
        default=DEFAULT_INPUT_CSV,
        help="Input CSV path.",
    )
    parser.add_argument(
        "--output-csv",
        default="",
        help="Output CSV path. Default: Source_revised2/Result/<timestamp>.csv",
    )
    parser.add_argument("--workers", type=int, default=10, help="Parallel worker threads.")
    parser.add_argument("--nimp", type=int, default=30, help="ATS inner non-improving iterations.")
    parser.add_argument("--limit", type=int, default=0, help="Process first N rows only (0 = all).")
    parser.add_argument("--verbose", action="store_true", help="Enable ATS verbose logs.")
    args = parser.parse_args()

    input_csv = Path(args.input_csv)
    if not input_csv.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_csv}")

    with input_csv.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if args.limit and args.limit > 0:
        rows = rows[: args.limit]

    workers = max(1, int(args.workers))
    total = len(rows)
    processed_by_index: Dict[int, Dict[str, Any]] = {}

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
        out_csv = out_dir / f"c201_segment1_parallel{workers}_{int(time.time())}.csv"

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
    print(f"rows={len(processed)} ok={ok} workers={workers}")


if __name__ == "__main__":
    main()
