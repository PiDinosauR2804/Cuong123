#!/usr/bin/env python3
import argparse
import csv
import os
import pathlib
import shutil
import subprocess
import sys
from typing import Dict


def normalized_output_stem(stem: str) -> str:
    s = (stem or "").strip()
    if s.endswith("_input"):
        s = s[: -len("_input")]
    return s or stem


def process_one(
    in_csv: pathlib.Path,
    out_csv: pathlib.Path,
    solver_bin: str,
    workers: int,
) -> int:
    with in_csv.open(newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        fieldnames = r.fieldnames or []
        rows = list(r)

    # No Python-side normalization.
    # Keep Function6.cpp as the single source of truth for normalize/validate/fitness.

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    env = os.environ.copy()
    env["SOLVER_BIN"] = solver_bin
    env["N_TRUCK"] = "2"
    env["N_DRONE"] = "1"
    env["PARALLEL_JOBS"] = str(workers)
    rc = subprocess.run(
        [sys.executable, "recompute_metrics_from_solutions_csv.py", str(out_csv)],
        env=env,
        text=True,
    ).returncode
    return rc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--solver-bin", default="C_Version/read_data_f6")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("inputs", nargs="+")
    args = ap.parse_args()

    out_dir = pathlib.Path(args.out_dir)
    overall_rc = 0

    for s in args.inputs:
        in_csv = pathlib.Path(s)
        out_csv = out_dir / (normalized_output_stem(in_csv.stem) + "_refine.csv")
        print(f"[RUN] {in_csv}")
        rc = process_one(in_csv, out_csv, args.solver_bin, args.workers)
        fail_rep = pathlib.Path("batch_init_ats_improved_with_multivisit_recompute_failures.csv")
        if fail_rep.exists():
            dst = out_dir / (in_csv.stem + "_recompute_failures.csv")
            shutil.move(str(fail_rep), str(dst))
            print(f"[REPORT] {dst}")
        print(f"[OUT] {out_csv} rc={rc}")
        if rc != 0:
            overall_rc = 1

    return overall_rc


if __name__ == "__main__":
    raise SystemExit(main())
