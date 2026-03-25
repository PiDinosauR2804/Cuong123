import csv
import os
import pathlib
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Optional, Tuple


BEST_MULTI_FILE_RE = re.compile(r"^\[SOL\]\s+best multi-visit solution file:\s+(?P<path>.+?)\s*$")
BEST_MULTI_MAKESPAN_RE = re.compile(r"^\[SOL\]\s+best multi-visit makespan:\s+(?P<makespan>[0-9]+(?:\.[0-9]+)?)\s*$")
VALIDATE_RE = re.compile(
    r"\[VALIDATE\]\s+OK\s+makespan\s+(?P<makespan>[0-9]+(?:\.[0-9]+)?)\s+"
    r"multi\s+(?P<multi>[0-9]+)\s+"
    r"max_drone_wait\s+(?P<max_wait>[0-9]+(?:\.[0-9]+)?)\s+"
    r"total_drone_wait\s+(?P<total_wait>[0-9]+(?:\.[0-9]+)?)"
)


def normalize_solution_text(file_text: str) -> str:
    # File format is `solution = [ ... ]`. CSV expects the raw list.
    s = file_text
    eq = s.find("=")
    if eq != -1:
        s = s[eq + 1 :]
    # Remove all whitespace to keep it single-cell and compact.
    s = "".join(ch for ch in s if not ch.isspace())
    return s


def validate_solution_file(instance: str, a: str, l: str, seed_file: pathlib.Path) -> Tuple[bool, str, Optional[float]]:
    env = dict(os.environ)
    env["VALIDATE_ONLY"] = "1"
    env.setdefault("SOLVER_TIME_LIMIT_SEC", "5")
    cmd = ["C_Version/read_data_function2", instance, str(a), str(l), str(seed_file)]
    p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = (p.stdout or "").strip()
    line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
    if p.returncode != 0 or "[VALIDATE] OK" not in out:
        return False, line or f"exit={p.returncode}", None
    m = VALIDATE_RE.search(line)
    if not m:
        return False, f"cannot parse validate output: {line}", None
    return True, line, float(m.group("makespan"))


def run_one(row: dict, time_limit_sec: int) -> dict:
    job_id = (row.get("job_id") or "").strip()
    instance = (row.get("instance") or "").strip()
    a = (row.get("A") or "").strip()
    l = (row.get("L") or "").strip()
    best_solution = (row.get("best_solution") or "").strip()

    out = {
        "job_id": job_id,
        "instance": instance,
        "A": a,
        "L": l,
        "status": "SKIP",
        "new_best_multi_fitness": "",
        "new_best_multi_solution": "",
        "reason": "",
    }
    if not best_solution:
        out["status"] = "FAIL"
        out["reason"] = "empty best_solution"
        return out

    # Seed file
    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
        f.write("solution = " + best_solution + "\n")
        seed_path = f.name

    # Output dir per job to avoid collisions
    out_dir = pathlib.Path("batch_multivisit_outputs") / f"job_{job_id}"
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        env = dict(os.environ)
        env["SKIP_ATS"] = "1"
        env["SKIP_TRUCK_2OPT"] = "1"
        env["LS_MULTI_ONLY"] = "1"
        env["SOLVER_TIME_LIMIT_SEC"] = str(time_limit_sec)
        env["SOL_OUT_DIR"] = str(out_dir)

        cmd = ["C_Version/read_data_function2", instance, str(a), str(l), seed_path]
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        stdout = p.stdout or ""

        best_multi_path = None
        best_multi_makespan = None
        for ln in stdout.splitlines():
            m = BEST_MULTI_MAKESPAN_RE.match(ln.strip())
            if m:
                best_multi_makespan = float(m.group("makespan"))
            m = BEST_MULTI_FILE_RE.match(ln.strip())
            if m:
                best_multi_path = m.group("path").strip()

        if not best_multi_path or best_multi_path.endswith("(not found)") or best_multi_path.endswith("(write failed)"):
            out["status"] = "OK"
            out["reason"] = "no multi-visit produced"
            return out

        multi_file = pathlib.Path(best_multi_path)
        if not multi_file.exists():
            out["status"] = "FAIL"
            out["reason"] = f"multi file missing: {best_multi_path}"
            return out

        # Validate the multi solution before accepting it.
        ok, msg, mk = validate_solution_file(instance, a, l, multi_file)
        if not ok or mk is None:
            out["status"] = "FAIL"
            out["reason"] = f"multi invalid: {msg}"
            return out

        out["status"] = "OK"
        out["new_best_multi_fitness"] = f"{mk:.12f}".rstrip("0").rstrip(".")
        out["new_best_multi_solution"] = normalize_solution_text(multi_file.read_text())
        return out
    finally:
        try:
            os.unlink(seed_path)
        except OSError:
            pass


def parse_float_or_inf(s: str) -> float:
    s = (s or "").strip()
    if not s:
        return float("inf")
    try:
        return float(s)
    except ValueError:
        return float("inf")


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python3 run_multivisit_ls_batch.py <batch_init_ats_improved.csv>")
        return 2

    csv_in = pathlib.Path(sys.argv[1])
    if not csv_in.exists():
        print(f"CSV not found: {csv_in}")
        return 2

    jobs = int(os.environ.get("PARALLEL_JOBS", "10"))
    time_limit_sec = int(os.environ.get("TIME_LIMIT_SEC", "900"))

    out_csv = pathlib.Path("batch_init_ats_improved_with_multivisit.csv")

    with csv_in.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    # Ensure columns exist
    if "best_multi_solution" not in fieldnames:
        fieldnames.append("best_multi_solution")
    if "best_multi_fitness" not in fieldnames:
        fieldnames.append("best_multi_fitness")

    # Run in parallel (subprocess work is IO-bound from python's perspective)
    results = {}
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = []
        for row in rows:
            futs.append(ex.submit(run_one, row, time_limit_sec))
        for fut in as_completed(futs):
            r = fut.result()
            results[r["job_id"]] = r

    updated = 0
    improved = 0
    for row in rows:
        job_id = (row.get("job_id") or "").strip()
        r = results.get(job_id)
        if not r or r["status"] != "OK":
            continue
        new_fit = parse_float_or_inf(r["new_best_multi_fitness"])
        if new_fit == float("inf"):
            continue
        old_fit = parse_float_or_inf(row.get("best_multi_fitness", ""))
        if new_fit + 1e-9 < old_fit:
            row["best_multi_fitness"] = r["new_best_multi_fitness"]
            row["best_multi_solution"] = r["new_best_multi_solution"]
            improved += 1
        updated += 1

    with out_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    print(f"Rows: {len(rows)}")
    print(f"Ran LS multi-only for: {updated}")
    print(f"Improved best_multi_fitness: {improved}")
    print(f"Wrote: {out_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
