import ast
import csv
import json
import math
import os
import pathlib
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Dict, List, Optional, Tuple

_INSTANCE_PATH_CACHE: Dict[str, str] = {}

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

VALIDATE_WAITS_RE = re.compile(
    r"\[VALIDATE_WAITS\]\s+OK\s+avg_sortie_time\s+(?P<avg_sortie>[0-9]+(?:\.[0-9]+)?)\s+"
    r"(?:total_sortie_time\s+(?P<total_sortie>[0-9]+(?:\.[0-9]+)?)\s+)?"
    r"avg_truck_wait\s+(?P<avg_truck_wait>[0-9]+(?:\.[0-9]+)?)\s+"
    r"avg_drone_wait\s+(?P<avg_drone_wait>[0-9]+(?:\.[0-9]+)?)\s+"
    r"trips\s+(?P<trips>[0-9]+)\s+legs\s+(?P<legs>[0-9]+)"
)
def tokenize(line: str) -> List[str]:
    return line.strip().split()


def _is_numeric(s: str) -> bool:
    try:
        float(s)
        return True
    except Exception:
        return False


def read_instance(path: str) -> Dict[str, Any]:
    # Kept consistent with gha_write_run_meta.py
    p: Dict[str, Any] = {
        "truck_speed": 0.5,
        "drone_speed": 1.0,
        "sigma": 5.0,
        "customers": [],
        "truck_time": [],
        "drone_time": [],
    }
    lines = pathlib.Path(path).read_text(encoding="utf-8", errors="ignore").splitlines()
    if not lines:
        return p
    first = tokenize(lines[0])
    idx = 0
    parameterized = bool(first) and first[0] == "number_truck"
    header_only = (not parameterized) and bool(first) and (not _is_numeric(first[0]))
    if parameterized:
        p["truck_speed"] = float(tokenize(lines[2])[-1])
        p["drone_speed"] = float(tokenize(lines[3])[-1])
        p["sigma"] = float(tokenize(lines[6])[-1])
        idx = 8
    elif header_only:
        idx = 1
    else:
        idx = 0

    for ln in lines[idx:]:
        tok = tokenize(ln)
        if len(tok) < 4:
            continue
        x, y = float(tok[0]), float(tok[1])
        demand, release = int(tok[2]), int(tok[3])
        p["customers"].append({"x": x, "y": y, "demand": demand, "release": release})

    n = len(p["customers"])
    p["truck_time"] = [[0.0] * n for _ in range(n)]
    p["drone_time"] = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            a = p["customers"][i]
            b = p["customers"][j]
            manhattan = abs(a["x"] - b["x"]) + abs(a["y"] - b["y"])
            euclid = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
            p["truck_time"][i][j] = manhattan / float(p["truck_speed"])
            p["drone_time"][i][j] = euclid / float(p["drone_speed"])
    return p


def parse_solution_text(solution_text: str) -> Optional[Any]:
    txt = (solution_text or "").strip()
    if txt.lower().startswith("solution"):
        pos = txt.find("=")
        if pos >= 0:
            txt = txt[pos + 1 :].strip()
    if not txt:
        return None
    try:
        data = ast.literal_eval(txt)
    except Exception:
        return None
    if not isinstance(data, list) or len(data) != 2:
        return None
    return data


def normalize_solution_text(solution_text: str) -> str:
    txt = (solution_text or "").strip()
    if txt.lower().startswith("solution"):
        pos = txt.find("=")
        if pos >= 0:
            txt = txt[pos + 1 :].strip()
    if not txt:
        return ""
    try:
        data = ast.literal_eval(txt)
        return json.dumps(data, separators=(",", ":"))
    except Exception:
        return " ".join(txt.split())


def compute_solution_stats(solution: Any, instance: Dict[str, Any]) -> Dict[str, Optional[float]]:
    # Ported from gha_write_run_meta.py (expects "legacy" list format)
    if solution is None:
        return {
            "drone_avg_trip_time": None,
            "multi_visit_trip_count": None,
            "drone_trip_count": None,
            "avg_customers_per_trip": None,
        }

    trucks_raw, drone_raw = solution

    truck_routes: List[List[Tuple[int, List[int]]]] = []
    for tr in trucks_raw:
        route: List[Tuple[int, List[int]]] = []
        for stop in tr:
            city = int(stop[0])
            pkgs = [int(x) for x in stop[1]]
            route.append((city, pkgs))
        truck_routes.append(route)

    drone_trips: List[List[Tuple[int, List[int]]]] = []
    for trip in drone_raw:
        events: List[Tuple[int, List[int]]] = []
        for ev in trip:
            city = int(ev[0])
            pkgs = [int(x) for x in ev[1]]
            events.append((city, pkgs))
        drone_trips.append(events)

    # Build simple truck timelines matching C++ compute_truck_timeline behavior.
    truck_arrivals: List[Dict[Tuple[int, int], float]] = []
    for route in truck_routes:
        resupplied = set()
        for city, pkgs in route:
            if city != 0:
                for x in pkgs:
                    resupplied.add(x)
        max_rel = 0
        for city, _ in route:
            if city == 0:
                continue
            if city not in resupplied:
                rel = instance["customers"][city]["release"] if city < len(instance["customers"]) else 0
                max_rel = max(max_rel, rel)
        arr: Dict[Tuple[int, int], float] = {}
        t = float(max_rel)
        if route:
            arr[(route[0][0], 0)] = 0.0
        for i in range(len(route) - 1):
            frm = route[i][0]
            to = route[i + 1][0]
            if frm < len(instance["truck_time"]) and to < len(instance["truck_time"]):
                t += float(instance["truck_time"][frm][to])
            arr[(to, i + 1)] = t
        truck_arrivals.append(arr)

    # Infer (truck_id, stop_index) by matching rendezvous city + package subset of that stop's pkgs.
    stop_candidates: Dict[int, List[Tuple[int, int, List[int]]]] = {}
    for tid, route in enumerate(truck_routes):
        for idx, (city, pkgs) in enumerate(route):
            if city == 0:
                continue
            stop_candidates.setdefault(city, []).append((tid, idx, pkgs))

    def infer_truck_for_event(city: int, pkgs: List[int]) -> Tuple[Optional[int], Optional[int]]:
        cands = stop_candidates.get(city, [])
        sp = set(pkgs)
        for tid, idx, spkgs in cands:
            if sp.issubset(set(spkgs)):
                return tid, idx
        if cands:
            return cands[0][0], cands[0][1]
        return None, None

    durations: List[float] = []
    customers_per_trip: List[int] = []
    multi_visit_count = 0
    for trip in drone_trips:
        trip_trucks = set()
        for city, pkgs in trip:
            tid, _ = infer_truck_for_event(city, pkgs)
            if tid is not None:
                trip_trucks.add(tid)
        if len(trip_trucks) >= 2:
            multi_visit_count += 1

        customers_per_trip.append(sum(len(pkgs) for _, pkgs in trip))

        last = 0
        t = 0.0
        dur = 0.0
        for city, pkgs in trip:
            if last < len(instance["drone_time"]) and city < len(instance["drone_time"]):
                leg = float(instance["drone_time"][last][city])
            else:
                leg = 0.0
            t += leg
            dur += leg
            tid, sidx = infer_truck_for_event(city, pkgs)
            if tid is not None and sidx is not None:
                ta = truck_arrivals[tid].get((city, sidx), t)
                if ta > t:
                    dur += (ta - t)
                    t = ta
            last = city

        back = float(instance["drone_time"][last][0]) if last < len(instance["drone_time"]) else 0.0
        dur += back
        durations.append(dur)

    trip_count = len(drone_trips)
    avg_duration = (sum(durations) / float(trip_count)) if trip_count > 0 else 0.0
    avg_customers = (sum(customers_per_trip) / float(trip_count)) if trip_count > 0 else 0.0
    return {
        "drone_avg_trip_time": avg_duration,
        "multi_visit_trip_count": float(multi_visit_count),
        "drone_trip_count": float(trip_count),
        "avg_customers_per_trip": avg_customers,
    }


def validate_with_stats(
    instance: str, a: str, l: str, solution_text: str
) -> Tuple[
    bool,
    str,
    Optional[float],  # makespan
    Optional[float],  # avg_used
    Optional[float],  # max_used
    Optional[int],    # trips
    Optional[float],  # avg_sortie_time
    Optional[float],  # total_sortie_time
    Optional[float],  # avg_truck_wait
    Optional[float],  # avg_drone_wait
    Optional[int],    # legs
    ]:
    payload = (solution_text or "").strip()
    if payload.lower().startswith("solution"):
        pos = payload.find("=")
        if pos >= 0:
            payload = payload[pos + 1 :].strip()

    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
        f.write("solution = " + payload + "\n")
        seed_path = f.name
    try:
        env = dict(os.environ)
        env["VALIDATE_ONLY"] = "1"
        env.setdefault("SOLVER_TIME_LIMIT_SEC", "5")
        solver_bin = env.get("SOLVER_BIN", "C_Version/read_data_function2")
        cmd = [solver_bin, instance, str(a), str(l), seed_path]
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        out = (p.stdout or "").strip()
        line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
        if p.returncode != 0 or "[VALIDATE] OK" not in out:
            return False, line or f"exit={p.returncode}", None, None, None, None, None, None, None, None, None
        m = VALIDATE_RE.search(line)
        if not m:
            return False, "cannot parse validate output: " + (line or ""), None, None, None, None, None, None, None, None, None

        makespan = float(m.group("makespan"))
        avg_used = None
        max_used = None
        trips = None
        avg_sortie = None
        total_sortie = None
        avg_truck_wait = None
        avg_drone_wait = None
        legs = None
        # Newer binaries print [VALIDATE_STATS] on a separate line.
        for ln in out.splitlines():
            if "[VALIDATE_STATS]" not in ln:
                continue
            m2 = VALIDATE_STATS_RE.search(ln.strip())
            if m2:
                avg_used = float(m2.group("avg_used"))
                max_used = float(m2.group("max_used"))
                trips = int(m2.group("trips"))
                break
        for ln in out.splitlines():
            if "[VALIDATE_WAITS]" not in ln:
                continue
            m3 = VALIDATE_WAITS_RE.search(ln.strip())
            if m3:
                avg_sortie = float(m3.group("avg_sortie"))
                total_sortie = float(m3.group("total_sortie")) if m3.group("total_sortie") is not None else None
                avg_truck_wait = float(m3.group("avg_truck_wait"))
                avg_drone_wait = float(m3.group("avg_drone_wait"))
                legs = int(m3.group("legs"))
                if total_sortie is None:
                    trips_wait = int(m3.group("trips"))
                    total_sortie = avg_sortie * float(trips_wait)
                break
        return True, line, makespan, avg_used, max_used, trips, avg_sortie, total_sortie, avg_truck_wait, avg_drone_wait, legs
    finally:
        try:
            os.unlink(seed_path)
        except OSError:
            pass


def _fmt_float(v: Optional[float]) -> str:
    if v is None:
        return ""
    s = f"{float(v):.12f}".rstrip("0").rstrip(".")
    return s


def recompute_for_row(row: Dict[str, str]) -> Dict[str, Any]:
    job_id = (row.get("job_id") or "").strip()
    inst = (row.get("instance") or "").strip()
    dat_path = (row.get("dat_path") or "").strip()
    inst_file = dat_path or inst
    if inst and inst in _INSTANCE_PATH_CACHE:
        inst_file = _INSTANCE_PATH_CACHE[inst]
    else:
        p = pathlib.Path(inst_file) if inst_file else None
        if p is None or not p.exists():
            cand = pathlib.Path(f"{inst}.dat")
            if cand.exists():
                inst_file = str(cand)
            else:
                base_dir = os.environ.get("CSV_BASE_DIR", "result/generated_instances/c101_cluster5_strategies_augmented").strip()
                if base_dir:
                    cand2 = pathlib.Path(base_dir) / f"{inst}.dat"
                    if cand2.exists():
                        inst_file = str(cand2)
                    else:
                        found = next(pathlib.Path(".").rglob(f"{inst}.dat"), None)
                        if found is not None:
                            inst_file = str(found)
        if inst and inst_file:
            _INSTANCE_PATH_CACHE[inst] = inst_file
    a = (row.get("A") or "").strip()
    l = (row.get("L") or "").strip()
    best_sol = (row.get("best_solution") or "").strip()
    best_multi_sol = (row.get("best_multi_solution") or "").strip()

    out: Dict[str, Any] = {"ok": True, "reason": ""}
    if job_id:
        out["job_id"] = job_id

    # Load instance once per row. (Rows share instances, but caching isn't needed at this scale.)
    try:
        inst_data = read_instance(inst_file)
    except Exception as e:
        out["ok"] = False
        out["reason"] = f"cannot read instance ({inst_file}): {e}"
        return out

    if best_sol:
        ok, msg, mk, avg_used, _max_used, _trips, avg_sortie, total_sortie, avg_truck_wait, avg_drone_wait, legs = validate_with_stats(inst_file, a, l, best_sol)
        if not ok or mk is None:
            out["best_fitness"] = ""
            out["best_drone_avg_trip_time"] = ""
            out["best_avg_sortie_time"] = ""
            out["best_total_sortie_time"] = ""
            out["best_avg_truck_wait_for_drone"] = ""
            out["best_avg_drone_wait_for_truck"] = ""
            out["best_multi_visit_trip_count"] = ""
            out["best_drone_trip_count"] = ""
            out["best_avg_customers_per_trip"] = ""
            out["ok"] = False
            out["reason"] = "best_solution invalid: " + msg
            return out
        out["best_fitness"] = _fmt_float(mk)
        stats = compute_solution_stats(parse_solution_text(best_sol), inst_data)
        # Redefine to match endurance measure: (return-depart) - (#rendezvous*sigma)
        out["best_drone_avg_trip_time"] = _fmt_float(avg_used)
        out["best_avg_sortie_time"] = _fmt_float(avg_sortie)
        out["best_total_sortie_time"] = _fmt_float(total_sortie)
        out["best_avg_truck_wait_for_drone"] = _fmt_float(avg_truck_wait)
        out["best_avg_drone_wait_for_truck"] = _fmt_float(avg_drone_wait)
        out["best_multi_visit_trip_count"] = (
            str(int(stats["multi_visit_trip_count"])) if stats["multi_visit_trip_count"] is not None else ""
        )
        out["best_drone_trip_count"] = (
            str(int(stats["drone_trip_count"])) if stats["drone_trip_count"] is not None else ""
        )
        out["best_avg_customers_per_trip"] = _fmt_float(stats["avg_customers_per_trip"])
    else:
        out["best_fitness"] = ""
        out["best_drone_avg_trip_time"] = ""
        out["best_avg_sortie_time"] = ""
        out["best_total_sortie_time"] = ""
        out["best_avg_truck_wait_for_drone"] = ""
        out["best_avg_drone_wait_for_truck"] = ""
        out["best_multi_visit_trip_count"] = ""
        out["best_drone_trip_count"] = ""
        out["best_avg_customers_per_trip"] = ""

    if best_multi_sol:
        ok, msg, mk, avg_used, _max_used, _trips, avg_sortie, total_sortie, avg_truck_wait, avg_drone_wait, legs = validate_with_stats(inst_file, a, l, best_multi_sol)
        if not ok or mk is None:
            out["best_multi_fitness"] = ""
            out["best_multi_drone_avg_trip_time"] = ""
            out["best_multi_avg_sortie_time"] = ""
            out["best_multi_total_sortie_time"] = ""
            out["best_multi_avg_truck_wait_for_drone"] = ""
            out["best_multi_avg_drone_wait_for_truck"] = ""
            out["best_multi_multi_visit_trip_count"] = ""
            out["best_multi_drone_trip_count"] = ""
            out["best_multi_avg_customers_per_trip"] = ""
            out["ok"] = False
            out["reason"] = "best_multi_solution invalid: " + msg
            return out
        out["best_multi_fitness"] = _fmt_float(mk)
        stats = compute_solution_stats(parse_solution_text(best_multi_sol), inst_data)
        out["best_multi_drone_avg_trip_time"] = _fmt_float(avg_used)
        out["best_multi_avg_sortie_time"] = _fmt_float(avg_sortie)
        out["best_multi_total_sortie_time"] = _fmt_float(total_sortie)
        out["best_multi_avg_truck_wait_for_drone"] = _fmt_float(avg_truck_wait)
        out["best_multi_avg_drone_wait_for_truck"] = _fmt_float(avg_drone_wait)
        out["best_multi_multi_visit_trip_count"] = (
            str(int(stats["multi_visit_trip_count"])) if stats["multi_visit_trip_count"] is not None else ""
        )
        out["best_multi_drone_trip_count"] = (
            str(int(stats["drone_trip_count"])) if stats["drone_trip_count"] is not None else ""
        )
        out["best_multi_avg_customers_per_trip"] = _fmt_float(stats["avg_customers_per_trip"])
    else:
        # Keep multi-related columns empty if no multi solution
        out["best_multi_fitness"] = ""
        out["best_multi_drone_avg_trip_time"] = ""
        out["best_multi_avg_sortie_time"] = ""
        out["best_multi_total_sortie_time"] = ""
        out["best_multi_avg_truck_wait_for_drone"] = ""
        out["best_multi_avg_drone_wait_for_truck"] = ""
        out["best_multi_multi_visit_trip_count"] = ""
        out["best_multi_drone_trip_count"] = ""
        out["best_multi_avg_customers_per_trip"] = ""

    return out


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python3 recompute_metrics_from_solutions_csv.py <batch_init_ats_improved_with_multivisit.csv>")
        return 2

    csv_path = pathlib.Path(sys.argv[1])
    if not csv_path.exists():
        print("CSV not found:", csv_path)
        return 2

    workers = int(os.environ.get("PARALLEL_JOBS", "10"))

    with csv_path.open(newline="") as f:
        r = csv.DictReader(f)
        rows = list(r)
        fieldnames = r.fieldnames or []

    # Keep solution payloads in one-line compact form.
    for row in rows:
        if "best_solution" in row:
            row["best_solution"] = normalize_solution_text(row.get("best_solution", ""))
        if "best_multi_solution" in row:
            row["best_multi_solution"] = normalize_solution_text(row.get("best_multi_solution", ""))

    required = {"instance", "A", "L", "best_solution", "best_multi_solution"}
    missing = required - set(fieldnames)
    if missing:
        print("Missing columns:", sorted(missing))
        print("Have columns:", fieldnames)
        return 2

    # Ensure new columns exist
    new_cols = [
        "best_drone_avg_trip_time",
        "best_multi_visit_trip_count",
        "best_drone_trip_count",
        "best_avg_customers_per_trip",
        "best_multi_drone_avg_trip_time",
        "best_multi_multi_visit_trip_count",
        "best_multi_drone_trip_count",
        "best_multi_avg_customers_per_trip",
        "best_solution_file",
        "best_multi_solution_file",
        "log",
        "best_avg_sortie_time",
        "best_total_sortie_time",
        "best_avg_truck_wait_for_drone",
        "best_avg_drone_wait_for_truck",
        "best_multi_avg_sortie_time",
        "best_multi_total_sortie_time",
        "best_multi_avg_truck_wait_for_drone",
        "best_multi_avg_drone_wait_for_truck",
    ]
    for c in new_cols:
        if c not in fieldnames:
            fieldnames.append(c)
            for row in rows:
                row[c] = ""

    # Recompute in parallel
    results: Dict[int, Dict[str, Any]] = {}
    failures: List[Dict[str, str]] = []
    with ThreadPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(recompute_for_row, row): idx for idx, row in enumerate(rows)}
        for fut in as_completed(futs):
            out = fut.result()
            idx = futs[fut]
            results[idx] = out

    # Apply results to rows
    ok_cnt = 0
    fail_cnt = 0
    for idx, row in enumerate(rows):
        out = results.get(idx)
        if not out:
            fail_cnt += 1
            failures.append(
                {
                    "row_index": str(idx),
                    "job_id": (row.get("job_id") or "").strip(),
                    "instance": (row.get("instance") or "").strip(),
                    "A": (row.get("A") or "").strip(),
                    "L": (row.get("L") or "").strip(),
                    "reason": (out or {}).get("reason", "unknown failure"),
                }
            )
            continue
        for k, v in out.items():
            if k in ("row_index", "job_id", "ok", "reason"):
                continue
            if k in fieldnames:
                row[k] = str(v)
        if not out.get("ok"):
            fail_cnt += 1
            failures.append(
                {
                    "row_index": str(idx),
                    "job_id": (row.get("job_id") or "").strip(),
                    "instance": (row.get("instance") or "").strip(),
                    "A": (row.get("A") or "").strip(),
                    "L": (row.get("L") or "").strip(),
                    "reason": (out or {}).get("reason", "unknown failure"),
                }
            )
            continue
        ok_cnt += 1

    # Write back in-place with backup
    backup = csv_path.with_suffix(csv_path.suffix + ".bak2")
    tmp = csv_path.with_suffix(csv_path.suffix + ".tmp")
    if not backup.exists():
        backup.write_text(csv_path.read_text())

    with tmp.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    tmp.replace(csv_path)

    print("Rows:", len(rows))
    print("Recomputed OK:", ok_cnt)
    print("Recomputed FAIL:", fail_cnt)
    print("Wrote:", csv_path)
    print("Backup:", backup)
    if failures:
        rep = pathlib.Path("batch_init_ats_improved_with_multivisit_recompute_failures.csv")
        with rep.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["row_index", "job_id", "instance", "A", "L", "reason"])
            w.writeheader()
            w.writerows(failures)
        print("Wrote failures report:", rep)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
