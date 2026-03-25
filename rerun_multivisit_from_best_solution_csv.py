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
    return "".join(ch for ch in s if not ch.isspace())


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


def parse_float_or_blank(v: str) -> str:
    v = (v or "").strip()
    if not v:
        return ""
    try:
        f = float(v)
        s = f"{f:.12f}".rstrip("0").rstrip(".")
        return s
    except ValueError:
        return ""

def ensure_solver_binary() -> None:
    root = pathlib.Path(__file__).resolve().parent
    src = root / "C_Version" / "function2.cpp"
    binp = root / "C_Version" / "read_data_function2"
    if not src.exists():
        raise RuntimeError(f"missing solver source: {src}")
    if binp.exists():
        try:
            if binp.stat().st_mtime >= src.stat().st_mtime:
                return
        except OSError:
            pass
    cmd = [
        "/usr/bin/clang++",
        "-std=c++17",
        "-O2",
        str(src),
        "-o",
        str(binp),
    ]
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if p.returncode != 0:
        raise RuntimeError("failed to build solver binary:\n" + (p.stdout or ""))


def run_one(row: dict, time_limit_sec: int, out_root: pathlib.Path) -> dict:
    job_id = (row.get("job_id") or "").strip()
    instance = (row.get("instance") or "").strip()
    a = (row.get("A") or "").strip()
    l = (row.get("L") or "").strip()
    best_solution = (row.get("best_solution") or "").strip()

    out = {
        "job_id": job_id,
        "status": "SKIP",
        "best_multi_fitness": "",
        "best_multi_solution": "",
        "reason": "",
    }
    if not best_solution:
        out["status"] = "FAIL"
        out["reason"] = "empty best_solution"
        return out

    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
        f.write("solution = " + best_solution + "\n")
        seed_path = f.name

    job_out_dir = out_root / f"job_{job_id}"
    job_out_dir.mkdir(parents=True, exist_ok=True)

    try:
        env = dict(os.environ)
        # Defaults: only run drone-local-search to create multi-visit solutions from the seed.
        # Allow callers to override these via their own environment variables.
        env.setdefault("SKIP_ATS", "1")
        env.setdefault("SKIP_TRUCK_2OPT", "1")
        env.setdefault("LS_MULTI_ONLY", "1")
        env["SOLVER_TIME_LIMIT_SEC"] = str(time_limit_sec)
        env["SOL_OUT_DIR"] = str(job_out_dir)

        cmd = ["C_Version/read_data_function2", instance, str(a), str(l), seed_path]
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        stdout = p.stdout or ""
        if p.returncode != 0:
            tail = "\n".join(stdout.splitlines()[-5:]).strip()
            out["status"] = "FAIL"
            out["reason"] = f"solver exit={p.returncode}" + (f": {tail}" if tail else "")
            return out

        best_multi_path = None
        for ln in stdout.splitlines():
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

        ok, msg, mk = validate_solution_file(instance, a, l, multi_file)
        if not ok or mk is None:
            out["status"] = "FAIL"
            out["reason"] = f"multi invalid: {msg}"
            return out

        out["status"] = "OK"
        out["best_multi_fitness"] = parse_float_or_blank(str(mk))
        out["best_multi_solution"] = normalize_solution_text(multi_file.read_text())
        return out
    finally:
        try:
            os.unlink(seed_path)
        except OSError:
            pass


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python3 rerun_multivisit_from_best_solution_csv.py <batch_init_ats_improved_with_multivisit.csv>")
        return 2

    try:
        ensure_solver_binary()
    except Exception as e:
        print(f"[BUILD] FAIL: {e}")
        return 2

    csv_in = pathlib.Path(sys.argv[1])
    if not csv_in.exists():
        print(f"CSV not found: {csv_in}")
        return 2

    jobs = int(os.environ.get("PARALLEL_JOBS", "10"))
    time_limit_sec = int(os.environ.get("TIME_LIMIT_SEC", "600"))

    out_root = pathlib.Path(os.environ.get("MULTIVISIT_OUT_DIR", "batch_multivisit_outputs_rerun"))
    out_root.mkdir(parents=True, exist_ok=True)

    with csv_in.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    if "best_multi_solution" not in fieldnames:
        fieldnames.append("best_multi_solution")
    if "best_multi_fitness" not in fieldnames:
        fieldnames.append("best_multi_fitness")

    # 1) Clear all existing multi-visit results as requested.
    for row in rows:
        row["best_multi_solution"] = ""
        row["best_multi_fitness"] = ""

    # 2) Re-run LS multi-visit from best_solution.
    results = {}
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = [ex.submit(run_one, row, time_limit_sec, out_root) for row in rows]
        for fut in as_completed(futs):
            r = fut.result()
            results[r["job_id"]] = r

    ok = 0
    fail = 0
    for row in rows:
        job_id = (row.get("job_id") or "").strip()
        r = results.get(job_id)
        if not r:
            continue
        if r["status"] == "OK":
            ok += 1
            if r["best_multi_solution"]:
                row["best_multi_solution"] = r["best_multi_solution"]
                row["best_multi_fitness"] = r["best_multi_fitness"]
        else:
            fail += 1

    # Write back in-place with a backup.
    backup = csv_in.with_suffix(csv_in.suffix + ".bak")
    tmp = csv_in.with_suffix(csv_in.suffix + ".tmp")
    if not backup.exists():
        backup.write_text(csv_in.read_text())

    with tmp.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    tmp.replace(csv_in)

    print(f"Rows: {len(rows)}")
    print(f"OK jobs: {ok}")
    print(f"FAIL/SKIP jobs: {fail}")
    print(f"Wrote: {csv_in}")
    print(f"Backup: {backup}")
    print(f"Outputs: {out_root}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
