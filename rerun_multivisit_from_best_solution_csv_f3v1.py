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
VALIDATE_STATS_RE = re.compile(
    r"\[VALIDATE_STATS\]\s+OK\s+avg_drone_used\s+(?P<avg_used>[0-9]+(?:\.[0-9]+)?)\s+"
    r"max_drone_used\s+(?P<max_used>[0-9]+(?:\.[0-9]+)?)\s+"
    r"trips\s+(?P<trips>[0-9]+)\s+"
    r"sigma\s+(?P<sigma>[0-9]+(?:\.[0-9]+)?)"
)


def normalize_solution_text(file_text: str) -> str:
    # File format is `solution = [ ... ]`. CSV expects the raw list.
    s = file_text
    eq = s.find("=")
    if eq != -1:
        s = s[eq + 1 :]
    # Remove all whitespace to keep it single-cell and compact.
    return "".join(ch for ch in s if not ch.isspace())


def parse_float_or_inf(s: str) -> float:
    s = (s or "").strip()
    if not s:
        return float("inf")
    try:
        return float(s)
    except ValueError:
        return float("inf")


def parse_float_or_blank(v: str) -> str:
    v = (v or "").strip()
    if not v:
        return ""
    try:
        f = float(v)
        return f"{f:.12f}".rstrip("0").rstrip(".")
    except ValueError:
        return ""


def ensure_solver_binary() -> None:
    root = pathlib.Path(__file__).resolve().parent
    src = root / "C_Version" / "Function3.cpp"
    binp = root / "C_Version" / "read_data_function3"
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


def validate_solution_file(
    solver_bin: str, instance: str, a: str, l: str, seed_file: pathlib.Path
) -> Tuple[bool, str, Optional[float], Optional[int], Optional[int]]:
    env = dict(os.environ)
    env["VALIDATE_ONLY"] = "1"
    env.setdefault("SOLVER_TIME_LIMIT_SEC", "5")
    cmd = [solver_bin, instance, str(a), str(l), str(seed_file)]
    p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = (p.stdout or "").strip()
    line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
    if p.returncode != 0 or "[VALIDATE] OK" not in out:
        return False, line or f"exit={p.returncode}", None, None, None
    m = VALIDATE_RE.search(line)
    if not m:
        return False, f"cannot parse validate output: {line}", None, None, None
    mk = float(m.group("makespan"))
    mv = int(m.group("multi"))
    trips = None
    for ln in out.splitlines():
        if "[VALIDATE_STATS]" not in ln:
            continue
        m2 = VALIDATE_STATS_RE.search(ln.strip())
        if m2:
            trips = int(m2.group("trips"))
            break
    return True, line, mk, mv, trips


def _better_multi(
    new_mk: float,
    new_mv: int,
    new_trips: Optional[int],
    old_mk: float,
    old_mv: int,
    old_trips: Optional[int],
) -> bool:
    eps = 1e-9
    if new_mk + eps < old_mk:
        return True
    if abs(new_mk - old_mk) > eps:
        return False
    if new_mv != old_mv:
        return new_mv > old_mv
    if new_trips is None or old_trips is None:
        return False
    return new_trips < old_trips


def run_one(row: dict, time_limit_sec: int, out_root: pathlib.Path, solver_bin: str) -> dict:
    job_id = (row.get("job_id") or "").strip()
    instance = (row.get("instance") or "").strip()
    a = (row.get("A") or "").strip()
    l = (row.get("L") or "").strip()

    # Prefer improving from an existing multi solution if present; else from best_solution.
    seed_solution = (row.get("best_multi_solution") or "").strip() or (row.get("best_solution") or "").strip()

    out = {
        "job_id": job_id,
        "status": "SKIP",
        "new_best_multi_fitness": "",
        "new_best_multi_solution": "",
        "new_best_multi_solution_file": "",
        "new_best_multi_mv": "",
        "new_best_multi_trips": "",
        "reason": "",
    }
    if not seed_solution:
        out["status"] = "FAIL"
        out["reason"] = "empty seed (best_solution and best_multi_solution are empty)"
        return out

    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
        f.write("solution = " + seed_solution + "\n")
        seed_path = f.name

    job_out_dir = out_root / f"job_{job_id}"
    job_out_dir.mkdir(parents=True, exist_ok=True)

    try:
        env = dict(os.environ)
        # Defaults: only run drone-local-search to create/strengthen multi-visit solutions from the seed.
        env.setdefault("SKIP_ATS", "1")
        env.setdefault("SKIP_TRUCK_2OPT", "1")
        env.setdefault("LS_MULTI_ONLY", "1")
        env["SOLVER_TIME_LIMIT_SEC"] = str(time_limit_sec)
        env["SOL_OUT_DIR"] = str(job_out_dir)

        cmd = [solver_bin, instance, str(a), str(l), seed_path]
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        stdout = p.stdout or ""
        if p.returncode != 0:
            tail = "\n".join(stdout.splitlines()[-8:]).strip()
            out["status"] = "FAIL"
            out["reason"] = f"solver exit={p.returncode}" + (f": {tail}" if tail else "")
            return out

        best_multi_path = None
        best_multi_mk = None
        for ln in stdout.splitlines():
            m = BEST_MULTI_MAKESPAN_RE.match(ln.strip())
            if m:
                try:
                    best_multi_mk = float(m.group("makespan"))
                except Exception:
                    best_multi_mk = None
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

        ok, msg, mk, mv, trips = validate_solution_file(solver_bin, instance, a, l, multi_file)
        if not ok or mk is None or mv is None:
            out["status"] = "FAIL"
            out["reason"] = f"multi invalid: {msg}"
            return out

        out["status"] = "OK"
        # trust validated mk, but keep the one printed by solver as debug in reason if differs
        out["new_best_multi_fitness"] = parse_float_or_blank(str(mk))
        out["new_best_multi_solution"] = normalize_solution_text(multi_file.read_text())
        out["new_best_multi_solution_file"] = str(multi_file)
        out["new_best_multi_mv"] = str(int(mv))
        out["new_best_multi_trips"] = str(int(trips)) if trips is not None else ""
        if best_multi_mk is not None and abs(best_multi_mk - mk) > 1e-6:
            out["reason"] = f"solver_mk={best_multi_mk} validated_mk={mk}"
        return out
    finally:
        try:
            os.unlink(seed_path)
        except OSError:
            pass


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python3 rerun_multivisit_from_best_solution_csv_f3v1.py <batch_init_ats_improved_with_multivisit.csv>")
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
    time_limit_sec = int(os.environ.get("TIME_LIMIT_SEC", "60"))
    out_root = pathlib.Path(os.environ.get("F3V1_OUT_DIR", "batch_multivisit_outputs_f3v1"))
    out_root.mkdir(parents=True, exist_ok=True)
    solver_bin = os.environ.get("F3V1_SOLVER_BIN", "C_Version/read_data_function3")

    with csv_in.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    # Ensure expected columns exist.
    for c in [
        "best_multi_solution",
        "best_multi_fitness",
        "best_multi_solution_file",
    ]:
        if c not in fieldnames:
            fieldnames.append(c)
            for row in rows:
                row[c] = ""

    # Add debug columns for this run.
    for c in ["f3v1_status", "f3v1_reason"]:
        if c not in fieldnames:
            fieldnames.append(c)
            for row in rows:
                row[c] = ""

    results = {}
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = [ex.submit(run_one, row, time_limit_sec, out_root, solver_bin) for row in rows]
        for fut in as_completed(futs):
            r = fut.result()
            results[r["job_id"]] = r

    improved = 0
    ok = 0
    fail = 0
    for row in rows:
        job_id = (row.get("job_id") or "").strip()
        r = results.get(job_id)
        if not r:
            continue
        row["f3v1_status"] = r["status"]
        row["f3v1_reason"] = r.get("reason", "")
        if r["status"] == "FAIL":
            fail += 1
            continue
        ok += 1
        new_fit = parse_float_or_inf(r.get("new_best_multi_fitness", ""))
        new_mv = int((r.get("new_best_multi_mv") or "0") or "0")
        new_trips = None
        try:
            if (r.get("new_best_multi_trips") or "").strip():
                new_trips = int(r.get("new_best_multi_trips") or "")
        except ValueError:
            new_trips = None
        if new_fit == float("inf"):
            continue
        old_fit = parse_float_or_inf(row.get("best_multi_fitness", ""))
        old_mv = 0
        old_trips = None
        try:
            old_mv = int((row.get("best_multi_multi_visit_trip_count") or "0") or "0")
        except ValueError:
            old_mv = 0
        try:
            if (row.get("best_multi_drone_trip_count") or "").strip():
                old_trips = int(row.get("best_multi_drone_trip_count") or "")
        except ValueError:
            old_trips = None

        if old_fit == float("inf"):
            # fill previously-empty multi fields
            row["best_multi_fitness"] = r["new_best_multi_fitness"]
            row["best_multi_solution"] = r["new_best_multi_solution"]
            row["best_multi_solution_file"] = r.get("new_best_multi_solution_file", "")
            improved += 1
        elif _better_multi(new_fit, new_mv, new_trips, old_fit, old_mv, old_trips):
            row["best_multi_fitness"] = r["new_best_multi_fitness"]
            row["best_multi_solution"] = r["new_best_multi_solution"]
            row["best_multi_solution_file"] = r.get("new_best_multi_solution_file", "")
            improved += 1

    in_name = csv_in.name
    out_csv = pathlib.Path(os.environ.get("F3V1_OUT_CSV", "F3V2_" + in_name))
    with out_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    print(f"Rows: {len(rows)}")
    print(f"OK: {ok}, FAIL: {fail}")
    print(f"Improved best_multi_fitness: {improved}")
    print(f"Wrote: {out_csv}")
    print(f"Outputs: {out_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
