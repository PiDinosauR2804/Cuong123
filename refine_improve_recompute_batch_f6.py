#!/usr/bin/env python3
import argparse
import ast
import csv
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Dict, List, Optional, Tuple


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

BEST_FILE_RE = re.compile(r"^\[SOL\]\s+best solution file:\s+(?P<path>.+?)\s*$")
BEST_MULTI_FILE_RE = re.compile(r"^\[SOL\]\s+best multi-visit solution file:\s+(?P<path>.+?)\s*$")


def _fmt(v: Optional[float]) -> str:
    if v is None:
        return ""
    s = f"{float(v):.12f}".rstrip("0").rstrip(".")
    return s


def _to_int(x: Any) -> Optional[int]:
    try:
        return int(x)
    except Exception:
        try:
            return int(float(x))
        except Exception:
            return None


def _is_numeric(s: str) -> bool:
    try:
        float(s)
        return True
    except Exception:
        return False


def read_instance_releases(path: str) -> List[int]:
    lines = pathlib.Path(path).read_text(encoding="utf-8", errors="ignore").splitlines()
    if not lines:
        return []
    first = lines[0].split()
    parameterized = bool(first) and first[0] == "number_truck"
    header_only = (not parameterized) and bool(first) and (not _is_numeric(first[0]))
    start = 8 if parameterized else (1 if header_only else 0)
    releases: List[int] = []
    for ln in lines[start:]:
        t = ln.split()
        if len(t) >= 4:
            releases.append(int(float(t[3])))
    return releases


def parse_solution(solution_text: str) -> Optional[Any]:
    txt = (solution_text or "").strip()
    if not txt:
        return None
    try:
        data = ast.literal_eval(txt)
    except Exception:
        return None
    if not isinstance(data, list) or len(data) != 2:
        return None
    return data


def compact_solution_text(solution_text: str) -> str:
    txt = (solution_text or "").strip()
    if not txt:
        return ""
    try:
        data = ast.literal_eval(txt)
        return json.dumps(data, separators=(",", ":"))
    except Exception:
        return " ".join(txt.split())


def normalize_solution_text(solution_text: str, releases: List[int]) -> str:
    txt = (solution_text or "").strip()
    if not txt:
        return ""
    data = parse_solution(txt)
    if data is None:
        return compact_solution_text(txt)
    trucks_raw, drone_raw = data
    if not isinstance(trucks_raw, list) or not isinstance(drone_raw, list):
        return compact_solution_text(txt)

    n = len(releases)
    owner: Dict[int, int] = {}
    pos_on_truck: Dict[int, int] = {}

    trucks = []
    for tid, route in enumerate(trucks_raw):
        route_norm = []
        if isinstance(route, list):
            for pos, st in enumerate(route):
                if not isinstance(st, (list, tuple)) or len(st) < 2:
                    continue
                c = _to_int(st[0])
                if c is None:
                    continue
                pk = st[1] if isinstance(st[1], list) else []
                pk_norm = [int(x) for x in pk if _to_int(x) is not None]
                route_norm.append([c, pk_norm])
                if 0 < c < n:
                    owner[c] = tid
                    pos_on_truck[c] = pos
        trucks.append(route_norm)

    drone = []
    for trip in drone_raw:
        if not isinstance(trip, list):
            continue
        trip_norm = []
        for ev in trip:
            if not isinstance(ev, (list, tuple)) or len(ev) < 2:
                continue
            rv = _to_int(ev[0])
            if rv is None:
                continue
            pk = ev[1] if isinstance(ev[1], list) else []
            pk_norm = [int(x) for x in pk if _to_int(x) is not None]
            trip_norm.append([rv, pk_norm])
        drone.append(trip_norm)

    def eligible(pkg: int) -> bool:
        return 0 < pkg < n and releases[pkg] > 0

    while True:
        changed = False
        cleaned = []
        for trip in drone:
            evs = []
            for rv, pkgs in trip:
                if rv <= 0 or rv >= n:
                    changed = True
                    continue
                tid = owner.get(rv, -1)
                rv_pos = pos_on_truck.get(rv, -1)
                if tid < 0 or rv_pos < 0:
                    changed = True
                    continue
                keep = []
                for x in pkgs:
                    if not eligible(x):
                        changed = True
                        continue
                    if owner.get(x, -1) != tid:
                        changed = True
                        continue
                    if pos_on_truck.get(x, -1) < rv_pos:
                        changed = True
                        continue
                    keep.append(x)
                if not keep:
                    changed = True
                    continue
                evs.append([rv, keep])
            if evs:
                cleaned.append(evs)
            elif trip:
                changed = True
        drone = cleaned

        is_resup = [0] * n
        for trip in drone:
            for _rv, pkgs in trip:
                for x in pkgs:
                    if 0 < x < n:
                        is_resup[x] = 1

        truck_count = max(owner.values(), default=-1) + 1
        rmax = [0] * max(1, truck_count)
        for c in range(1, n):
            t = owner.get(c, -1)
            if t >= 0 and not is_resup[c]:
                rmax[t] = max(rmax[t], releases[c])

        removed = False
        pruned = []
        for trip in drone:
            evs = []
            for rv, pkgs in trip:
                t = owner.get(rv, -1)
                if t < 0:
                    removed = True
                    continue
                keep = []
                for x in pkgs:
                    if x <= rmax[t]:
                        removed = True
                        continue
                    keep.append(x)
                if keep:
                    evs.append([rv, keep])
                else:
                    removed = True
            if evs:
                pruned.append(evs)
        drone = pruned

        if not changed and not removed:
            break

    return json.dumps([trucks, drone], separators=(",", ":"))


def multi_visit_count(solution_text: str) -> int:
    data = parse_solution(solution_text)
    if data is None:
        return 0
    trucks_raw, drone_raw = data
    owner: Dict[int, int] = {}
    for tid, route in enumerate(trucks_raw if isinstance(trucks_raw, list) else []):
        if not isinstance(route, list):
            continue
        for st in route:
            if not isinstance(st, (list, tuple)) or not st:
                continue
            city = _to_int(st[0])
            if city is not None and city > 0:
                owner[city] = tid

    cnt = 0
    for trip in drone_raw if isinstance(drone_raw, list) else []:
        if not isinstance(trip, list) or len(trip) <= 1:
            continue
        trucks = set()
        for ev in trip:
            if not isinstance(ev, (list, tuple)) or not ev:
                continue
            rv = _to_int(ev[0])
            if rv is None:
                continue
            t = owner.get(rv, -1)
            if t >= 0:
                trucks.add(t)
        if len(trucks) >= 2:
            cnt += 1
    return cnt


def avg_customers_per_trip(solution_text: str) -> Optional[float]:
    data = parse_solution(solution_text)
    if data is None:
        return None
    _trucks, drone_raw = data
    trips = [trip for trip in (drone_raw if isinstance(drone_raw, list) else []) if isinstance(trip, list)]
    if not trips:
        return 0.0
    total = 0
    for trip in trips:
        for ev in trip:
            if isinstance(ev, (list, tuple)) and len(ev) >= 2 and isinstance(ev[1], list):
                total += len(ev[1])
    return total / float(len(trips))


def validate_with_stats(
    solver_bin: str,
    n_truck: int,
    n_drone: int,
    instance: str,
    a: str,
    l: str,
    solution_text: str,
) -> Tuple[bool, str, Dict[str, Optional[float]]]:
    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
        f.write("solution = " + solution_text.strip() + "\n")
        seed_path = f.name
    try:
        env = dict(os.environ)
        env["VALIDATE_ONLY"] = "1"
        env["SOLVER_TIME_LIMIT_SEC"] = "10"
        env["N_TRUCK"] = str(n_truck)
        env["N_DRONE"] = str(n_drone)
        cmd = [solver_bin, instance, str(a), str(l), seed_path]
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        out = (p.stdout or "").strip()
        line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
        if p.returncode != 0 or "[VALIDATE] OK" not in out:
            return False, line or f"exit={p.returncode}", {}

        m = VALIDATE_RE.search(line)
        if not m:
            return False, "cannot parse validate output", {}
        metrics: Dict[str, Optional[float]] = {
            "makespan": float(m.group("makespan")),
            "multi_visit_trip_count": float(m.group("multi")),
            "max_drone_wait": float(m.group("max_wait")),
            "total_drone_wait": float(m.group("total_wait")),
            "avg_drone_used": None,
            "max_drone_used": None,
            "drone_trip_count": None,
            "avg_sortie_time": None,
            "total_sortie_time": None,
            "avg_truck_wait": None,
            "avg_drone_wait": None,
            "legs": None,
        }
        for ln in out.splitlines():
            if "[VALIDATE_STATS]" in ln:
                m2 = VALIDATE_STATS_RE.search(ln.strip())
                if m2:
                    metrics["avg_drone_used"] = float(m2.group("avg_used"))
                    metrics["max_drone_used"] = float(m2.group("max_used"))
                    metrics["drone_trip_count"] = float(m2.group("trips"))
            if "[VALIDATE_WAITS]" in ln:
                m3 = VALIDATE_WAITS_RE.search(ln.strip())
                if m3:
                    avg_sortie = float(m3.group("avg_sortie"))
                    metrics["avg_sortie_time"] = avg_sortie
                    total_sortie = float(m3.group("total_sortie")) if m3.group("total_sortie") else None
                    if total_sortie is None:
                        trips = int(m3.group("trips"))
                        total_sortie = avg_sortie * float(trips)
                    metrics["total_sortie_time"] = total_sortie
                    metrics["avg_truck_wait"] = float(m3.group("avg_truck_wait"))
                    metrics["avg_drone_wait"] = float(m3.group("avg_drone_wait"))
                    metrics["legs"] = float(m3.group("legs"))
        return True, line, metrics
    finally:
        try:
            os.unlink(seed_path)
        except OSError:
            pass


def run_local_search(
    solver_bin: str,
    n_truck: int,
    n_drone: int,
    time_limit_sec: int,
    out_dir: pathlib.Path,
    instance: str,
    a: str,
    l: str,
    seed_solution: str,
) -> Tuple[bool, str, str, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    seed_path = ""
    use_seed = bool(seed_solution.strip())
    if use_seed:
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
            f.write("solution = " + seed_solution.strip() + "\n")
            seed_path = f.name
    try:
        env = dict(os.environ)
        env["N_TRUCK"] = str(n_truck)
        env["N_DRONE"] = str(n_drone)
        env["SOL_OUT_DIR"] = str(out_dir)
        env["SOLVER_TIME_LIMIT_SEC"] = str(time_limit_sec)
        # Fallback mode from init solution: no seed, strong LS, one ATS segment, no diversification.
        if not use_seed:
            env["ATS_SEG"] = "1"
            env["ATS_DIV"] = "0"
        cmd = [solver_bin, instance, str(a), str(l)]
        if use_seed:
            cmd.append(seed_path)
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        out = p.stdout or ""
        if p.returncode != 0:
            return False, f"solver exit={p.returncode}", "", ""

        best_path = ""
        best_multi_path = ""
        for ln in out.splitlines():
            m = BEST_FILE_RE.match(ln.strip())
            if m:
                best_path = m.group("path").strip()
            m2 = BEST_MULTI_FILE_RE.match(ln.strip())
            if m2:
                best_multi_path = m2.group("path").strip()

        best_txt = ""
        best_multi_txt = ""
        if best_path and "(write failed)" not in best_path and "(not found)" not in best_path:
            pth = pathlib.Path(best_path)
            if pth.exists():
                best_txt = compact_solution_text(pth.read_text(encoding="utf-8", errors="ignore"))
        if best_multi_path and "(write failed)" not in best_multi_path and "(not found)" not in best_multi_path:
            pth = pathlib.Path(best_multi_path)
            if pth.exists():
                best_multi_txt = compact_solution_text(pth.read_text(encoding="utf-8", errors="ignore"))
        return True, "", best_txt, best_multi_txt
    finally:
        if use_seed and seed_path:
            try:
                os.unlink(seed_path)
            except OSError:
                pass


def choose_best_multi(
    cur: Optional[Dict[str, Any]],
    cand: Dict[str, Any],
) -> Dict[str, Any]:
    if cur is None:
        return cand
    eps = 1e-9
    cur_mk = float(cur["metrics"]["makespan"])
    cand_mk = float(cand["metrics"]["makespan"])
    if cand_mk + eps < cur_mk:
        return cand
    if abs(cand_mk - cur_mk) > eps:
        return cur
    cur_mv = int(round(float(cur["metrics"]["multi_visit_trip_count"] or 0.0)))
    cand_mv = int(round(float(cand["metrics"]["multi_visit_trip_count"] or 0.0)))
    if cand_mv > cur_mv:
        return cand
    if cand_mv < cur_mv:
        return cur
    cur_trips = cur["metrics"].get("drone_trip_count")
    cand_trips = cand["metrics"].get("drone_trip_count")
    if cur_trips is not None and cand_trips is not None and float(cand_trips) < float(cur_trips):
        return cand
    return cur


def process_row(
    idx: int,
    row: Dict[str, str],
    solver_bin: str,
    n_truck: int,
    n_drone: int,
    ls_time_limit_sec: int,
    ls_out_root: pathlib.Path,
    release_cache: Dict[str, List[int]],
) -> Tuple[int, Dict[str, str], str]:
    instance = (row.get("instance") or "").strip()
    a = (row.get("A") or "").strip()
    l = (row.get("L") or "").strip()
    if not instance:
        return idx, row, "missing instance"

    releases = release_cache.get(instance)
    if releases is None:
        releases = read_instance_releases(instance)
        release_cache[instance] = releases

    best_raw = (row.get("best_solution") or "").strip()
    multi_raw = (row.get("best_multi_solution") or "").strip()
    # Refine seeds before LS:
    # - move release<=0 packages out of drone resupply (truck takes from depot)
    # - clean invalid/empty drone events/trips
    best_seed = normalize_solution_text(compact_solution_text(best_raw), releases)
    multi_seed = normalize_solution_text(compact_solution_text(multi_raw), releases)
    # Keep only true multi-visit seed in best_multi_solution before local search.
    if multi_seed and multi_visit_count(multi_seed) <= 0:
        multi_seed = ""

    row["best_solution"] = best_seed
    row["best_multi_solution"] = multi_seed
    candidates_best: List[str] = [best_seed] if best_seed else []
    candidates_multi: List[str] = []
    if multi_seed:
        candidates_multi.append(multi_seed)

    base_best_metrics = None
    if best_seed:
        ok, _msg, metrics = validate_with_stats(solver_bin, n_truck, n_drone, instance, a, l, best_seed)
        if ok:
            base_best_metrics = metrics
    base_multi_metrics = None
    if multi_seed:
        okm, _msgm, metricsm = validate_with_stats(solver_bin, n_truck, n_drone, instance, a, l, multi_seed)
        if okm:
            base_multi_metrics = metricsm

    job_tag = (row.get("job_id") or str(idx)).strip()
    # Only run LS from the better seed (after Function6-based validation/normalization).
    eps = 1e-9
    chosen_seed = ""
    chosen_tag = "seed"
    if base_best_metrics is not None and base_multi_metrics is not None:
        mk_best = float(base_best_metrics.get("makespan") or float("inf"))
        mk_multi = float(base_multi_metrics.get("makespan") or float("inf"))
        if mk_multi + eps < mk_best:
            chosen_seed = multi_seed
            chosen_tag = "multi"
        else:
            chosen_seed = best_seed
            chosen_tag = "best"
    elif base_best_metrics is not None:
        chosen_seed = best_seed
        chosen_tag = "best"
    elif base_multi_metrics is not None:
        chosen_seed = multi_seed
        chosen_tag = "multi"
    elif best_seed:
        chosen_seed = best_seed
        chosen_tag = "best_unvalidated"
    elif multi_seed:
        chosen_seed = multi_seed
        chosen_tag = "multi_unvalidated"

    ok_ls1 = True
    msg_ls1 = ""
    if chosen_seed:
        ok_ls1, msg_ls1, ls1_best, ls1_multi = run_local_search(
            solver_bin,
            n_truck,
            n_drone,
            ls_time_limit_sec,
            ls_out_root / f"job_{job_tag}_{chosen_tag}",
            instance,
            a,
            l,
            chosen_seed,
        )
        if ok_ls1 and ls1_best:
            candidates_best.append(ls1_best)
        if ok_ls1 and ls1_multi:
            candidates_multi.append(ls1_multi)

    # Deduplicate candidate texts.
    candidates_best = list(dict.fromkeys([s for s in candidates_best if s]))
    candidates_multi = list(dict.fromkeys([s for s in candidates_multi if s]))

    # Choose best_solution (minimum makespan).
    best_sol_final = ""
    best_sol_metrics = None
    best_mk = float("inf")
    for cand in candidates_best:
        ok, _msg, metrics = validate_with_stats(solver_bin, n_truck, n_drone, instance, a, l, cand)
        if not ok or metrics.get("makespan") is None:
            continue
        mk = float(metrics["makespan"])
        if mk + 1e-9 < best_mk:
            best_mk = mk
            best_sol_final = cand
            best_sol_metrics = metrics

    if not best_sol_final and best_seed and base_best_metrics is not None:
        best_sol_final = best_seed
        best_sol_metrics = base_best_metrics

    # Choose best_multi_solution (must contain multi visit).
    best_multi_choice: Optional[Dict[str, Any]] = None
    for cand in candidates_multi:
        if multi_visit_count(cand) <= 0:
            continue
        ok, _msg, metrics = validate_with_stats(solver_bin, n_truck, n_drone, instance, a, l, cand)
        if not ok or metrics.get("makespan") is None:
            continue
        c = {"solution": cand, "metrics": metrics}
        best_multi_choice = choose_best_multi(best_multi_choice, c)

    best_multi_final = ""
    best_multi_metrics = None
    if best_multi_choice is not None:
        best_multi_final = best_multi_choice["solution"]
        best_multi_metrics = best_multi_choice["metrics"]

    # Fallback requested:
    # if both seeded best_solution and best_multi_solution yield no valid solution,
    # restart from init solution and run strong LS (plus one ATS segment, no diversification).
    if best_sol_metrics is None and best_multi_metrics is None:
        ok_fb, msg_fb, fb_best, fb_multi = run_local_search(
            solver_bin=solver_bin,
            n_truck=n_truck,
            n_drone=n_drone,
            time_limit_sec=ls_time_limit_sec,
            out_dir=ls_out_root / f"job_{job_tag}_fallback_init",
            instance=instance,
            a=a,
            l=l,
            seed_solution="",
        )
        if ok_fb:
            if fb_best:
                ok_b, _m_b, mt_b = validate_with_stats(solver_bin, n_truck, n_drone, instance, a, l, fb_best)
                if ok_b and mt_b.get("makespan") is not None:
                    best_sol_final = fb_best
                    best_sol_metrics = mt_b
            if fb_multi and multi_visit_count(fb_multi) > 0:
                ok_m, _m_m, mt_m = validate_with_stats(solver_bin, n_truck, n_drone, instance, a, l, fb_multi)
                if ok_m and mt_m.get("makespan") is not None:
                    best_multi_final = fb_multi
                    best_multi_metrics = mt_m

    row["best_solution"] = best_sol_final
    row["best_multi_solution"] = best_multi_final

    if best_sol_metrics is None:
        for c in [
            "best_fitness", "best_drone_avg_trip_time", "best_total_sortie_time", "best_multi_visit_trip_count",
            "best_drone_trip_count", "best_avg_customers_per_trip", "best_avg_sortie_time",
            "best_avg_truck_wait_for_drone", "best_avg_drone_wait_for_truck",
        ]:
            row[c] = ""
    else:
        row["best_fitness"] = _fmt(best_sol_metrics.get("makespan"))
        row["best_drone_avg_trip_time"] = _fmt(best_sol_metrics.get("avg_drone_used"))
        row["best_total_sortie_time"] = _fmt(best_sol_metrics.get("total_sortie_time"))
        row["best_multi_visit_trip_count"] = str(int(round(float(best_sol_metrics.get("multi_visit_trip_count") or 0.0))))
        row["best_drone_trip_count"] = str(int(round(float(best_sol_metrics.get("drone_trip_count") or 0.0))))
        row["best_avg_customers_per_trip"] = _fmt(avg_customers_per_trip(best_sol_final))
        row["best_avg_sortie_time"] = _fmt(best_sol_metrics.get("avg_sortie_time"))
        row["best_avg_truck_wait_for_drone"] = _fmt(best_sol_metrics.get("avg_truck_wait"))
        row["best_avg_drone_wait_for_truck"] = _fmt(best_sol_metrics.get("avg_drone_wait"))

    if best_multi_metrics is None:
        for c in [
            "best_multi_fitness", "best_multi_drone_avg_trip_time", "best_multi_total_sortie_time",
            "best_multi_multi_visit_trip_count", "best_multi_drone_trip_count",
            "best_multi_avg_customers_per_trip", "best_multi_avg_sortie_time",
            "best_multi_avg_truck_wait_for_drone", "best_multi_avg_drone_wait_for_truck",
        ]:
            row[c] = ""
    else:
        row["best_multi_fitness"] = _fmt(best_multi_metrics.get("makespan"))
        row["best_multi_drone_avg_trip_time"] = _fmt(best_multi_metrics.get("avg_drone_used"))
        row["best_multi_total_sortie_time"] = _fmt(best_multi_metrics.get("total_sortie_time"))
        row["best_multi_multi_visit_trip_count"] = str(
            int(round(float(best_multi_metrics.get("multi_visit_trip_count") or 0.0)))
        )
        row["best_multi_drone_trip_count"] = str(int(round(float(best_multi_metrics.get("drone_trip_count") or 0.0))))
        row["best_multi_avg_customers_per_trip"] = _fmt(avg_customers_per_trip(best_multi_final))
        row["best_multi_avg_sortie_time"] = _fmt(best_multi_metrics.get("avg_sortie_time"))
        row["best_multi_avg_truck_wait_for_drone"] = _fmt(best_multi_metrics.get("avg_truck_wait"))
        row["best_multi_avg_drone_wait_for_truck"] = _fmt(best_multi_metrics.get("avg_drone_wait"))

    reason = ""
    if not ok_ls1:
        reason = f"ls_best_seed_fail: {msg_ls1}"
    if best_sol_metrics is None and best_multi_metrics is None:
        if reason:
            reason += " | "
        reason += "fallback_init_used"
    return idx, row, reason


def ensure_columns(fieldnames: List[str], rows: List[Dict[str, str]]) -> List[str]:
    cols = [
        "best_fitness",
        "best_solution",
        "best_multi_fitness",
        "best_multi_solution",
        "best_drone_avg_trip_time",
        "best_total_sortie_time",
        "best_multi_visit_trip_count",
        "best_drone_trip_count",
        "best_avg_customers_per_trip",
        "best_multi_drone_avg_trip_time",
        "best_multi_total_sortie_time",
        "best_multi_multi_visit_trip_count",
        "best_multi_drone_trip_count",
        "best_multi_avg_customers_per_trip",
        "best_avg_sortie_time",
        "best_avg_truck_wait_for_drone",
        "best_avg_drone_wait_for_truck",
        "best_multi_avg_sortie_time",
        "best_multi_avg_truck_wait_for_drone",
        "best_multi_avg_drone_wait_for_truck",
        "f6_refine_status",
        "f6_refine_reason",
    ]
    out = list(fieldnames)
    for c in cols:
        if c not in out:
            out.append(c)
            for row in rows:
                row[c] = ""
    return out


def copy_inputs_for_git(inputs: List[pathlib.Path], tracked_dir: pathlib.Path) -> List[pathlib.Path]:
    tracked_dir.mkdir(parents=True, exist_ok=True)
    copied: List[pathlib.Path] = []
    for p in inputs:
        dst = tracked_dir / p.name
        try:
            same = dst.exists() and p.resolve() == dst.resolve()
        except OSError:
            same = False
        if not same:
            shutil.copy2(p, dst)
        copied.append(dst)
    return copied


def normalized_output_stem(stem: str) -> str:
    s = (stem or "").strip()
    if s.endswith("_input"):
        s = s[: -len("_input")]
    return s or stem


def process_file(
    in_csv: pathlib.Path,
    out_csv: pathlib.Path,
    solver_bin: str,
    n_truck: int,
    n_drone: int,
    workers: int,
    ls_time_limit_sec: int,
    ls_out_root: pathlib.Path,
) -> int:
    with in_csv.open(newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        rows = list(r)
        fieldnames = r.fieldnames or []

    fieldnames = ensure_columns(fieldnames, rows)
    release_cache: Dict[str, List[int]] = {}
    for row in rows:
        inst = (row.get("instance") or "").strip()
        if inst and inst not in release_cache:
            release_cache[inst] = read_instance_releases(inst)
    results: Dict[int, Tuple[Dict[str, str], str]] = {}

    with ThreadPoolExecutor(max_workers=workers) as ex:
        futs = [
            ex.submit(
                process_row,
                idx,
                row.copy(),
                solver_bin,
                n_truck,
                n_drone,
                ls_time_limit_sec,
                ls_out_root,
                release_cache,
            )
            for idx, row in enumerate(rows)
        ]
        for fut in as_completed(futs):
            idx, row_new, reason = fut.result()
            results[idx] = (row_new, reason)

    fail = 0
    for i in range(len(rows)):
        row_new, reason = results[i]
        row_new["f6_refine_status"] = "OK" if not reason else "WARN"
        row_new["f6_refine_reason"] = reason
        if reason:
            fail += 1
        rows[i] = row_new

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    print(f"[OUT] {out_csv}")
    print(f"[ROWS] {len(rows)}")
    print(f"[WARN] {fail}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--inputs", nargs="+", required=True, help="Input CSV files")
    ap.add_argument("--tracked-input-dir", default="result/Question4,14/tracked_inputs_2truck_1drone")
    ap.add_argument("--out-dir", default="result/Question4,14/refined_csv_2truck_1drone")
    ap.add_argument("--solver-bin", default="C_Version/read_data_f6")
    ap.add_argument("--n-truck", type=int, default=2)
    ap.add_argument("--n-drone", type=int, default=1)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--ls-time-limit-sec", type=int, default=120)
    ap.add_argument("--ls-out-dir", default="result/Question4,14/ls_outputs_f6_row_jobs")
    args = ap.parse_args()

    solver_bin = pathlib.Path(args.solver_bin)
    if not solver_bin.exists():
        print(f"solver not found: {solver_bin}")
        return 2

    input_paths = [pathlib.Path(x).expanduser() for x in args.inputs]
    for p in input_paths:
        if not p.exists():
            print(f"input not found: {p}")
            return 2

    tracked_inputs = copy_inputs_for_git(input_paths, pathlib.Path(args.tracked_input_dir))
    print("[TRACKED_INPUTS]")
    for p in tracked_inputs:
        print(" -", p)

    rc = 0
    out_dir = pathlib.Path(args.out_dir)
    ls_out_root = pathlib.Path(args.ls_out_dir)
    for in_csv in tracked_inputs:
        out_stem = normalized_output_stem(in_csv.stem)
        out_csv = out_dir / f"{out_stem}_refine.csv"
        one_rc = process_file(
            in_csv=in_csv,
            out_csv=out_csv,
            solver_bin=str(solver_bin),
            n_truck=args.n_truck,
            n_drone=args.n_drone,
            workers=args.workers,
            ls_time_limit_sec=args.ls_time_limit_sec,
            ls_out_root=ls_out_root / out_stem,
        )
        rc = max(rc, one_rc)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
