#!/usr/bin/env python3
import argparse
import csv
import json
import os
import pathlib
import re
import subprocess
from collections import defaultdict
from statistics import mean
from typing import Dict, List, Tuple, Optional


def to_float(x: str) -> Optional[float]:
    try:
        return float(str(x).strip())
    except Exception:
        return None


def norm_num(x: str) -> str:
    t = ("" if x is None else str(x)).strip()
    try:
        f = float(t)
        if abs(f - round(f)) < 1e-9:
            return str(int(round(f)))
        return str(f).rstrip("0").rstrip(".")
    except Exception:
        return t


def pick_output_name(src_name: str) -> str:
    m = re.match(r"^solve-f6-(.+)-three-folders-summary(?: \(\d+\))?\.csv$", src_name)
    if m:
        return f"best_{m.group(1)}.csv"
    stem = pathlib.Path(src_name).stem
    stem = re.sub(r"\s*\(\d+\)$", "", stem)
    return f"best_{stem}.csv"


def read_csv(path: pathlib.Path) -> Tuple[List[Dict[str, str]], List[str]]:
    with path.open(newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        rows = list(r)
        fields = list(r.fieldnames or [])
    return rows, fields


def write_csv(path: pathlib.Path, rows: List[Dict[str, str]], fieldnames: List[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def group_select(rows: List[Dict[str, str]]) -> Tuple[List[Dict[str, str]], List[str]]:
    fieldnames = list(rows[0].keys())
    for c in ["selection_source", "group_size", "avg_best_fitness_group", "p_multi_fitness_group"]:
        if c not in fieldnames:
            fieldnames.append(c)

    groups = defaultdict(list)
    for r in rows:
        inst = (r.get("instance") or "").strip()  # full path as requested
        a = norm_num(r.get("A"))
        l = norm_num(r.get("L"))
        groups[(inst, a, l)].append(r)

    out_rows: List[Dict[str, str]] = []
    for i, (_k, grp) in enumerate(sorted(groups.items(), key=lambda kv: kv[0]), start=1):
        best_vals = [to_float(r.get("best_fitness", "")) for r in grp]
        best_vals = [x for x in best_vals if x is not None]
        avg_best = mean(best_vals) if best_vals else None

        multi_candidates = []
        for r in grp:
            s = (r.get("best_multi_solution") or "").strip()
            f = to_float(r.get("best_multi_fitness", ""))
            if s and f is not None:
                multi_candidates.append((f, r))
        multi_candidates.sort(key=lambda x: x[0])
        p_multi = multi_candidates[0][1] if multi_candidates else None
        p_multi_fit = multi_candidates[0][0] if multi_candidates else None

        best_candidates = []
        for r in grp:
            s = (r.get("best_solution") or "").strip()
            f = to_float(r.get("best_fitness", ""))
            if s and f is not None:
                best_candidates.append((f, r))
        best_candidates.sort(key=lambda x: x[0])
        p_best = best_candidates[0][1] if best_candidates else None

        use_multi = (
            p_multi is not None
            and avg_best is not None
            and p_multi_fit is not None
            and p_multi_fit <= avg_best
        )

        base = dict(grp[0])
        base["job_id"] = str(i)
        base["run_index"] = "agg"
        base["selection_source"] = "p_multi" if use_multi else "best_solution"
        base["group_size"] = str(len(grp))
        base["avg_best_fitness_group"] = "" if avg_best is None else f"{avg_best:.12f}".rstrip("0").rstrip(".")
        base["p_multi_fitness_group"] = "" if p_multi_fit is None else f"{p_multi_fit:.12f}".rstrip("0").rstrip(".")

        if p_multi is not None:
            base["best_multi_solution"] = p_multi.get("best_multi_solution", "")
            base["best_multi_fitness"] = p_multi.get("best_multi_fitness", "")
        else:
            base["best_multi_solution"] = ""
            base["best_multi_fitness"] = ""

        if use_multi and p_multi is not None:
            base["best_solution"] = p_multi.get("best_multi_solution", "")
            base["best_fitness"] = p_multi.get("best_multi_fitness", "")
        elif p_best is not None:
            base["best_solution"] = p_best.get("best_solution", "")
            base["best_fitness"] = p_best.get("best_fitness", "")
        else:
            base["best_solution"] = ""
            base["best_fitness"] = ""

        for c in [
            "best_drone_avg_trip_time",
            "best_multi_visit_trip_count",
            "best_drone_trip_count",
            "best_avg_customers_per_trip",
            "best_multi_drone_avg_trip_time",
            "best_multi_multi_visit_trip_count",
            "best_multi_drone_trip_count",
            "best_multi_avg_customers_per_trip",
            "best_avg_sortie_time",
            "best_total_sortie_time",
            "best_avg_truck_wait_for_drone",
            "best_avg_drone_wait_for_truck",
            "best_multi_avg_sortie_time",
            "best_multi_total_sortie_time",
            "best_multi_avg_truck_wait_for_drone",
            "best_multi_avg_drone_wait_for_truck",
            "best_solution_file",
            "best_multi_solution_file",
            "log",
        ]:
            if c in base:
                base[c] = ""

        out_rows.append(base)

    return out_rows, fieldnames


def apply_monotonic(rows: List[Dict[str, str]]) -> Tuple[List[Dict[str, str]], List[str], int]:
    fieldnames = list(rows[0].keys())
    for c in ["rule_adjusted", "rule_from_A", "rule_from_L"]:
        if c not in fieldnames:
            fieldnames.append(c)

    by_inst = defaultdict(dict)
    for r in rows:
        inst = (r.get("instance") or "").strip()
        a = norm_num(r.get("A"))
        l = norm_num(r.get("L"))
        by_inst[inst][(a, l)] = r

    def cp(dst: Dict[str, str], src: Dict[str, str], sa: str, sl: str) -> None:
        dst["best_solution"] = src.get("best_solution", "")
        dst["best_fitness"] = src.get("best_fitness", "")
        dst["rule_adjusted"] = "1"
        dst["rule_from_A"] = sa
        dst["rule_from_L"] = sl

    def le(a: Optional[float], b: Optional[float]) -> bool:
        return a is not None and b is not None and a <= b

    adj = 0
    for _inst, m in by_inst.items():
        for k in [("4", "60"), ("4", "90"), ("4", "120"), ("8", "60"), ("8", "90"), ("8", "120")]:
            if k in m:
                m[k].setdefault("rule_adjusted", "0")
                m[k].setdefault("rule_from_A", "")
                m[k].setdefault("rule_from_L", "")

        if ("4", "90") in m and ("4", "60") in m and not le(to_float(m[("4", "90")].get("best_fitness", "")), to_float(m[("4", "60")].get("best_fitness", ""))):
            cp(m[("4", "90")], m[("4", "60")], "4", "60")
            adj += 1

        if ("4", "120") in m and ("4", "90") in m and not le(to_float(m[("4", "120")].get("best_fitness", "")), to_float(m[("4", "90")].get("best_fitness", ""))):
            cp(m[("4", "120")], m[("4", "90")], "4", "90")
            adj += 1

        if ("8", "60") in m and ("4", "60") in m and not le(to_float(m[("8", "60")].get("best_fitness", "")), to_float(m[("4", "60")].get("best_fitness", ""))):
            cp(m[("8", "60")], m[("4", "60")], "4", "60")
            adj += 1

        if ("8", "90") in m and ("4", "90") in m and ("8", "60") in m:
            f1 = to_float(m[("4", "90")].get("best_fitness", ""))
            f2 = to_float(m[("8", "60")].get("best_fitness", ""))
            src = None
            bf = None
            if f1 is not None and f2 is not None:
                src = ("4", "90") if f1 <= f2 else ("8", "60")
                bf = min(f1, f2)
            elif f1 is not None:
                src, bf = ("4", "90"), f1
            elif f2 is not None:
                src, bf = ("8", "60"), f2
            if src is not None and not le(to_float(m[("8", "90")].get("best_fitness", "")), bf):
                cp(m[("8", "90")], m[src], src[0], src[1])
                adj += 1

        if ("8", "120") in m and ("4", "120") in m and ("8", "90") in m:
            f1 = to_float(m[("4", "120")].get("best_fitness", ""))
            f2 = to_float(m[("8", "90")].get("best_fitness", ""))
            src = None
            bf = None
            if f1 is not None and f2 is not None:
                src = ("4", "120") if f1 <= f2 else ("8", "90")
                bf = min(f1, f2)
            elif f1 is not None:
                src, bf = ("4", "120"), f1
            elif f2 is not None:
                src, bf = ("8", "90"), f2
            if src is not None and not le(to_float(m[("8", "120")].get("best_fitness", "")), bf):
                cp(m[("8", "120")], m[src], src[0], src[1])
                adj += 1

    for r in rows:
        r.setdefault("rule_adjusted", "0")
        r.setdefault("rule_from_A", "")
        r.setdefault("rule_from_L", "")

    return rows, fieldnames, adj


def run_recompute(csv_path: pathlib.Path, recompute_script: pathlib.Path, solver_bin: str, n_truck: int, n_drone: int, jobs: int) -> Tuple[int, str]:
    env = dict(os.environ)
    env.update(
        {
            "FIX_DEMAND_EQUAL1": "1",
            "N_TRUCK": str(n_truck),
            "N_DRONE": str(n_drone),
            "SOLVER_BIN": solver_bin,
            "PARALLEL_JOBS": str(jobs),
        }
    )
    p = subprocess.run(
        ["python", str(recompute_script), str(csv_path)],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    return p.returncode, p.stdout or ""


def clear_invalid_best_multi(csv_path: pathlib.Path, failures_csv: pathlib.Path) -> int:
    if not failures_csv.exists():
        return 0
    fails = list(csv.DictReader(failures_csv.open(newline="", encoding="utf-8")))
    if not fails:
        return 0

    rows, fields = read_csv(csv_path)
    if not rows:
        return 0

    changed = 0
    for f in fails:
        idx = int(f.get("row_index", "-1") or -1)
        reason = (f.get("reason") or "").lower()
        if 0 <= idx < len(rows) and "best_multi_solution invalid" in reason:
            rows[idx]["best_multi_solution"] = ""
            rows[idx]["best_multi_fitness"] = ""
            for c in [
                "best_multi_drone_avg_trip_time",
                "best_multi_multi_visit_trip_count",
                "best_multi_drone_trip_count",
                "best_multi_avg_customers_per_trip",
                "best_multi_avg_sortie_time",
                "best_multi_total_sortie_time",
                "best_multi_avg_truck_wait_for_drone",
                "best_multi_avg_drone_wait_for_truck",
                "best_multi_solution_file",
            ]:
                if c in rows[idx]:
                    rows[idx][c] = ""
            changed += 1

    if changed:
        write_csv(csv_path, rows, fields)
    return changed


def process_one_file(
    src_csv: pathlib.Path,
    out_csv: pathlib.Path,
    recompute_script: pathlib.Path,
    solver_bin: str,
    n_truck: int,
    n_drone: int,
    jobs: int,
    failures_csv: pathlib.Path,
    recompute_retries: int,
) -> Dict[str, object]:
    rows, _ = read_csv(src_csv)
    if not rows:
        return {"file": src_csv.name, "output": str(out_csv), "status": "skip_empty"}

    selected, fields = group_select(rows)
    write_csv(out_csv, selected, fields)

    # recompute phase 1 with retries clearing invalid best_multi
    rc, log = 1, ""
    for _ in range(recompute_retries):
        rc, log = run_recompute(out_csv, recompute_script, solver_bin, n_truck, n_drone, jobs)
        if rc == 0:
            break
        if clear_invalid_best_multi(out_csv, failures_csv) == 0:
            break
    if rc != 0:
        return {
            "file": src_csv.name,
            "output": str(out_csv),
            "status": "recompute1_failed",
            "log_tail": "\n".join(log.splitlines()[-20:]),
        }

    rows2, _ = read_csv(out_csv)
    mono_rows, mono_fields, adj = apply_monotonic(rows2)
    write_csv(out_csv, mono_rows, mono_fields)

    # recompute phase 2 with retries clearing invalid best_multi
    rc2, log2 = 1, ""
    for _ in range(recompute_retries):
        rc2, log2 = run_recompute(out_csv, recompute_script, solver_bin, n_truck, n_drone, jobs)
        if rc2 == 0:
            break
        if clear_invalid_best_multi(out_csv, failures_csv) == 0:
            break
    if rc2 != 0:
        return {
            "file": src_csv.name,
            "output": str(out_csv),
            "status": "recompute2_failed",
            "adjust_ops": adj,
            "log_tail": "\n".join(log2.splitlines()[-20:]),
        }

    final_rows, _ = read_csv(out_csv)
    return {
        "file": src_csv.name,
        "output": str(out_csv),
        "status": "ok",
        "rows_in": len(rows),
        "rows_out": len(final_rows),
        "adjust_ops": adj,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build best_solution CSVs from raw solve summary CSVs.")
    parser.add_argument("--input-dir", default="/Users/huetran/Desktop/raw_csv")
    parser.add_argument("--output-dir", default="/Users/huetran/Desktop/best_solution")
    parser.add_argument("--recompute-script", default="/Users/huetran/Cuong123/recompute_metrics_from_solutions_csv.py")
    parser.add_argument("--solver-bin", default="/Users/huetran/Cuong123/C_Version/read_data_f6")
    parser.add_argument("--n-truck", type=int, default=2)
    parser.add_argument("--n-drone", type=int, default=2)
    parser.add_argument("--parallel-jobs", type=int, default=8)
    parser.add_argument("--recompute-retries", type=int, default=4)
    parser.add_argument("--failures-csv", default="/Users/huetran/Cuong123/batch_init_ats_improved_with_multivisit_recompute_failures.csv")
    args = parser.parse_args()

    in_dir = pathlib.Path(args.input_dir)
    out_dir = pathlib.Path(args.output_dir)
    recompute_script = pathlib.Path(args.recompute_script)
    failures_csv = pathlib.Path(args.failures_csv)

    if not in_dir.is_dir():
        print(f"Input dir not found: {in_dir}")
        return 2
    if not recompute_script.exists():
        print(f"Recompute script not found: {recompute_script}")
        return 2

    out_dir.mkdir(parents=True, exist_ok=True)

    src_files = sorted([p for p in in_dir.glob("*.csv") if p.is_file()])
    report: List[Dict[str, object]] = []

    for src in src_files:
        out_csv = out_dir / pick_output_name(src.name)
        item = process_one_file(
            src_csv=src,
            out_csv=out_csv,
            recompute_script=recompute_script,
            solver_bin=args.solver_bin,
            n_truck=args.n_truck,
            n_drone=args.n_drone,
            jobs=args.parallel_jobs,
            failures_csv=failures_csv,
            recompute_retries=args.recompute_retries,
        )
        report.append(item)
        print(f"{item['status']}: {src.name} -> {out_csv.name}")

    report_path = out_dir / "batch_build_report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    final_summary = []
    for p in sorted(out_dir.glob("best_*.csv")):
        rows, _ = read_csv(p)
        final_summary.append(
            {
                "file": p.name,
                "rows": len(rows),
                "adjusted_rows": sum(1 for r in rows if (r.get("rule_adjusted") or "") == "1"),
            }
        )
    final_report_path = out_dir / "batch_build_report_final.json"
    final_report_path.write_text(json.dumps(final_summary, ensure_ascii=False, indent=2), encoding="utf-8")

    ok = sum(1 for x in report if x.get("status") == "ok")
    fail = len(report) - ok
    print(f"Done. inputs={len(src_files)} ok={ok} fail={fail}")
    print(f"report={report_path}")
    print(f"final_report={final_report_path}")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
