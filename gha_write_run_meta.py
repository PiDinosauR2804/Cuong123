#!/usr/bin/env python3
import argparse
import ast
import json
import math
import pathlib
import re
import re as _re

FIT_RE = re.compile(r'(?:Fitness_full \(makespan\)|Objective \(full\) makespan|Makespan:)\s*[: ]\s*([0-9]+(?:\.[0-9]+)?)')
MV_RE = re.compile(r'best multi-visit makespan:\s*([0-9]+(?:\.[0-9]+)?)', flags=re.IGNORECASE)
BEST_SOL_RE = re.compile(r'\[SOL\]\s+best solution file:\s*(.+)')
BEST_MULTI_SOL_RE = re.compile(r'\[SOL\]\s+best multi-visit solution file:\s*(.+)')
BEST_TOTAL_TRIP_TIME_RE = re.compile(r'\[SOL\]\s+best total_drone_trip_time:\s*([0-9]+(?:\.[0-9]+)?)', flags=re.IGNORECASE)
BEST_MULTI_TOTAL_TRIP_TIME_RE = re.compile(r'\[SOL\]\s+best multi-visit total_drone_trip_time:\s*([0-9]+(?:\.[0-9]+)?)', flags=re.IGNORECASE)


def extract_metrics(log_path: pathlib.Path):
    if not log_path.exists():
        return None, None, None, None, None, None
    txt = log_path.read_text(encoding='utf-8', errors='ignore')
    vals = [float(x) for x in FIT_RE.findall(txt)]
    mv_vals = [float(x) for x in MV_RE.findall(txt)]
    total_vals = [float(x) for x in BEST_TOTAL_TRIP_TIME_RE.findall(txt)]
    multi_total_vals = [float(x) for x in BEST_MULTI_TOTAL_TRIP_TIME_RE.findall(txt)]
    best_sol_match = BEST_SOL_RE.findall(txt)
    best_multi_sol_match = BEST_MULTI_SOL_RE.findall(txt)
    best_sol = best_sol_match[-1].strip() if best_sol_match else None
    best_multi_sol = best_multi_sol_match[-1].strip() if best_multi_sol_match else None
    return (
        min(vals) if vals else None,
        min(mv_vals) if mv_vals else None,
        best_sol,
        best_multi_sol,
        total_vals[-1] if total_vals else None,
        multi_total_vals[-1] if multi_total_vals else None,
    )


def tokenize(line: str):
    return line.strip().split()


def read_instance(path: str):
    p = {
        'truck_speed': 0.5,
        'drone_speed': 1.0,
        'sigma': 5.0,
        'customers': [],
        'truck_time': [],
        'drone_time': [],
    }
    lines = pathlib.Path(path).read_text(encoding='utf-8', errors='ignore').splitlines()
    if not lines:
        return p
    first = tokenize(lines[0])
    idx = 0
    parameterized = bool(first) and first[0] == 'number_truck'
    header_only = (not parameterized) and bool(first) and (not _is_numeric(first[0]))
    if parameterized:
        p['truck_speed'] = float(tokenize(lines[2])[-1])
        p['drone_speed'] = float(tokenize(lines[3])[-1])
        p['sigma'] = float(tokenize(lines[6])[-1])
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
        p['customers'].append({'x': x, 'y': y, 'demand': demand, 'release': release})
    n = len(p['customers'])
    p['truck_time'] = [[0.0] * n for _ in range(n)]
    p['drone_time'] = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            a = p['customers'][i]
            b = p['customers'][j]
            manhattan = abs(a['x'] - b['x']) + abs(a['y'] - b['y'])
            euclid = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            p['truck_time'][i][j] = manhattan / p['truck_speed']
            p['drone_time'][i][j] = euclid / p['drone_speed']
    return p


def _is_numeric(s: str):
    try:
        float(s)
        return True
    except Exception:
        return False


def parse_solution_file(sol_path: str):
    if not sol_path or '(not found)' in sol_path or '(write failed)' in sol_path:
        return None
    p = pathlib.Path(sol_path)
    if not p.exists():
        return None
    txt = p.read_text(encoding='utf-8', errors='ignore')
    pos = txt.find('=')
    if pos < 0:
        return None
    payload = txt[pos + 1:].strip()
    try:
        data = ast.literal_eval(payload)
    except Exception:
        return None
    if not isinstance(data, list) or len(data) != 2:
        return None
    return data


def read_solution_payload(sol_path: str):
    if not sol_path or '(not found)' in sol_path or '(write failed)' in sol_path:
        return None
    p = pathlib.Path(sol_path)
    if not p.exists():
        return None
    txt = p.read_text(encoding='utf-8', errors='ignore')
    pos = txt.find('=')
    if pos < 0:
        return None
    payload = txt[pos + 1:].strip()
    # Normalize to a single-line compact payload so CSV cells are one-line.
    try:
        data = ast.literal_eval(payload)
        return json.dumps(data, separators=(',', ':'))
    except Exception:
        return _re.sub(r'\s+', ' ', payload).strip()


def compute_solution_stats(solution, instance):
    if solution is None:
        return {
            'drone_avg_trip_time': None,
            'multi_visit_trip_count': None,
            'drone_trip_count': None,
            'avg_customers_per_trip': None,
            'total_drone_trip_time': None,
        }

    trucks_raw, drone_raw = solution
    truck_routes = []
    for tr in trucks_raw:
        route = []
        for stop in tr:
            city = int(stop[0])
            pkgs = [int(x) for x in stop[1]]
            route.append((city, pkgs))
        truck_routes.append(route)

    drone_trips = []
    for trip in drone_raw:
        events = []
        for ev in trip:
            city = int(ev[0])
            pkgs = [int(x) for x in ev[1]]
            events.append((city, pkgs))
        drone_trips.append(events)

    # Build simple truck timelines matching C++ compute_truck_timeline behavior.
    truck_arrivals = []
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
                rel = instance['customers'][city]['release'] if city < len(instance['customers']) else 0
                max_rel = max(max_rel, rel)
        arr = {}
        t = float(max_rel)
        if route:
            arr[(route[0][0], 0)] = 0.0
        for i in range(len(route) - 1):
            frm = route[i][0]
            to = route[i + 1][0]
            if frm < len(instance['truck_time']) and to < len(instance['truck_time']):
                t += instance['truck_time'][frm][to]
            arr[(to, i + 1)] = t
        truck_arrivals.append(arr)

    # Infer (truck_id, stop_index) by matching rendezvous city + package set.
    stop_candidates = {}
    for tid, route in enumerate(truck_routes):
        for idx, (city, pkgs) in enumerate(route):
            if city == 0:
                continue
            stop_candidates.setdefault(city, []).append((tid, idx, pkgs))

    def infer_truck_for_event(city, pkgs):
        cands = stop_candidates.get(city, [])
        for tid, idx, spkgs in cands:
            if set(pkgs).issubset(set(spkgs)):
                return tid, idx
        if cands:
            return cands[0][0], cands[0][1]
        return None, None

    durations = []
    customers_per_trip = []
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
            if last < len(instance['drone_time']) and city < len(instance['drone_time']):
                leg = instance['drone_time'][last][city]
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
        if last < len(instance['drone_time']):
            back = instance['drone_time'][last][0]
        else:
            back = 0.0
        dur += back
        durations.append(dur)

    trip_count = len(drone_trips)
    total_duration = sum(durations) if trip_count > 0 else 0.0
    avg_duration = (sum(durations) / trip_count) if trip_count > 0 else 0.0
    avg_customers = (sum(customers_per_trip) / trip_count) if trip_count > 0 else 0.0
    return {
        'drone_avg_trip_time': avg_duration,
        'multi_visit_trip_count': multi_visit_count,
        'drone_trip_count': trip_count,
        'avg_customers_per_trip': avg_customers,
        'total_drone_trip_time': total_duration,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results-dir', required=True)
    ap.add_argument('--base', required=True)
    ap.add_argument('--a', required=True, type=float)
    ap.add_argument('--l', required=True, type=float)
    ap.add_argument('--reps', required=True, type=int)
    ap.add_argument('--job-id', required=True, type=int)
    ap.add_argument('--instance', required=True)
    args = ap.parse_args()

    results_dir = pathlib.Path(args.results_dir)
    instance = read_instance(args.instance)
    for idx in range(1, args.reps + 1):
        log_name = f"{args.base}_A{int(args.a) if args.a.is_integer() else args.a}_L{int(args.l) if args.l.is_integer() else args.l}_r{idx}.log"
        log_path = results_dir / log_name
        best_fit, best_mv, best_sol_path, best_multi_sol_path, best_total_trip_time_log, best_multi_total_trip_time_log = extract_metrics(log_path)
        best_sol_payload = read_solution_payload(best_sol_path)
        best_multi_payload = read_solution_payload(best_multi_sol_path)
        best_stats = compute_solution_stats(parse_solution_file(best_sol_path), instance)
        best_multi_stats = compute_solution_stats(parse_solution_file(best_multi_sol_path), instance)
        meta = {
            'job_id': args.job_id,
            'run_index': idx,
            'instance': args.instance,
            'A': args.a,
            'L': args.l,
            'best_fitness': best_fit,
            'best_solution': best_sol_payload,
            'best_multi_fitness': best_mv,
            'best_multi_solution': best_multi_payload,
            'best_solution_file': best_sol_path,
            'best_multi_solution_file': best_multi_sol_path,
            'best_drone_avg_trip_time': best_stats['drone_avg_trip_time'],
            'best_total_sortie_time': best_total_trip_time_log if best_total_trip_time_log is not None else best_stats['total_drone_trip_time'],
            'best_multi_visit_trip_count': best_stats['multi_visit_trip_count'],
            'best_drone_trip_count': best_stats['drone_trip_count'],
            'best_avg_customers_per_trip': best_stats['avg_customers_per_trip'],
            'best_multi_drone_avg_trip_time': best_multi_stats['drone_avg_trip_time'],
            'best_multi_total_sortie_time': best_multi_total_trip_time_log if best_multi_total_trip_time_log is not None else best_multi_stats['total_drone_trip_time'],
            'best_multi_multi_visit_trip_count': best_multi_stats['multi_visit_trip_count'],
            'best_multi_drone_trip_count': best_multi_stats['drone_trip_count'],
            'best_multi_avg_customers_per_trip': best_multi_stats['avg_customers_per_trip'],
            'log': log_name,
        }
        out_path = results_dir / f'meta_run{idx}.json'
        out_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
