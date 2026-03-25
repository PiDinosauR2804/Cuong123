import csv
import os
import pathlib
import re
import subprocess
import sys
import tempfile


VALIDATE_RE = re.compile(
    r"\[VALIDATE\]\s+OK\s+makespan\s+(?P<makespan>[0-9]+(?:\.[0-9]+)?)\s+"
    r"multi\s+(?P<multi>[0-9]+)\s+"
    r"max_drone_wait\s+(?P<max_wait>[0-9]+(?:\.[0-9]+)?)\s+"
    r"total_drone_wait\s+(?P<total_wait>[0-9]+(?:\.[0-9]+)?)"
)


def run_validate(instance: str, a: str, l: str, solution_text: str) -> tuple[bool, str]:
    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
        f.write("solution = " + solution_text.strip() + "\n")
        seed_path = f.name
    try:
        env = dict(os.environ)
        env["VALIDATE_ONLY"] = "1"
        env.setdefault("SOLVER_TIME_LIMIT_SEC", "5")
        cmd = ["C_Version/read_data_function2", instance, str(a), str(l), seed_path]
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        out = (p.stdout or "").strip()
        line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
        ok = (p.returncode == 0) and ("[VALIDATE] OK" in out)
        return ok, line
    finally:
        try:
            os.unlink(seed_path)
        except OSError:
            pass


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python3 recompute_best_solution_from_csv.py <batch_init_ats_improved.csv>")
        return 2

    csv_path = pathlib.Path(sys.argv[1])
    if not csv_path.exists():
        print(f"CSV not found: {csv_path}")
        return 2

    out_path = pathlib.Path("batch_init_ats_improved_best_solution_recompute.csv")
    rows_out: list[dict] = []

    with csv_path.open(newline="") as f:
        r = csv.DictReader(f)
        required = {"job_id", "instance", "A", "L", "best_fitness", "best_solution"}
        missing = required - set(r.fieldnames or [])
        if missing:
            print(f"Missing columns: {sorted(missing)}")
            print(f"Have columns: {r.fieldnames}")
            return 2

        for row in r:
            job_id = (row.get("job_id") or "").strip()
            instance = (row.get("instance") or "").strip()
            a = (row.get("A") or "").strip()
            l = (row.get("L") or "").strip()
            best_fitness_str = (row.get("best_fitness") or "").strip()
            best_solution = (row.get("best_solution") or "").strip()

            out_row = {
                "job_id": job_id,
                "instance": instance,
                "A": a,
                "L": l,
                "best_fitness_csv": best_fitness_str,
                "valid": "0",
                "recomputed_makespan": "",
                "recomputed_multi_visit_trip_count": "",
                "recomputed_max_drone_wait": "",
                "recomputed_total_drone_wait": "",
                "diff_recomputed_minus_csv": "",
                "reason": "",
            }

            if not best_solution:
                out_row["reason"] = "empty best_solution"
                rows_out.append(out_row)
                continue

            ok, line = run_validate(instance, a, l, best_solution)
            if not ok:
                out_row["reason"] = line or "validate failed"
                rows_out.append(out_row)
                continue

            m = VALIDATE_RE.search(line)
            if not m:
                out_row["reason"] = f"cannot parse validate output: {line}"
                rows_out.append(out_row)
                continue

            out_row["valid"] = "1"
            makespan = float(m.group("makespan"))
            out_row["recomputed_makespan"] = f"{makespan:.6f}".rstrip("0").rstrip(".")
            out_row["recomputed_multi_visit_trip_count"] = m.group("multi")
            out_row["recomputed_max_drone_wait"] = m.group("max_wait")
            out_row["recomputed_total_drone_wait"] = m.group("total_wait")

            try:
                csv_fit = float(best_fitness_str)
                diff = makespan - csv_fit
                out_row["diff_recomputed_minus_csv"] = f"{diff:.6f}".rstrip("0").rstrip(".")
            except ValueError:
                out_row["diff_recomputed_minus_csv"] = ""

            rows_out.append(out_row)

    with out_path.open("w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "job_id",
                "instance",
                "A",
                "L",
                "best_fitness_csv",
                "valid",
                "recomputed_makespan",
                "recomputed_multi_visit_trip_count",
                "recomputed_max_drone_wait",
                "recomputed_total_drone_wait",
                "diff_recomputed_minus_csv",
                "reason",
            ],
        )
        w.writeheader()
        w.writerows(rows_out)

    print(f"Wrote: {out_path} (rows={len(rows_out)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

