#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


@dataclass
class Instance:
    n_truck: int
    n_drone: int
    truck_speed: float
    drone_speed: float
    m_d: float
    l_d: float
    sigma: float
    coords: List[Tuple[float, float]]
    demand: List[int]
    release: List[int]


def parse_dat(path: Path) -> Instance:
    lines = [ln.strip() for ln in path.read_text().splitlines() if ln.strip()]
    header = {}
    start = None
    for i, ln in enumerate(lines):
        tok = ln.split()
        if tok[0] == "XCOORD":
            start = i + 1
            break
        header[tok[0]] = tok[-1]
    if start is None:
        raise ValueError(f"Invalid .dat format: {path}")

    coords: List[Tuple[float, float]] = []
    demand: List[int] = []
    release: List[int] = []
    for ln in lines[start:]:
        x, y, d, r = ln.split()[:4]
        coords.append((float(x), float(y)))
        demand.append(int(float(d)))
        release.append(int(float(r)))

    return Instance(
        n_truck=int(header.get("number_truck", 2)),
        n_drone=int(header.get("number_drone", 1)),
        truck_speed=float(header["truck_speed"]),
        drone_speed=float(header["drone_speed"]),
        m_d=float(header["M_d"]),
        l_d=float(header["L_d"]),
        sigma=float(header["Sigma"]),
        coords=coords,
        demand=demand,
        release=release,
    )


def manhattan_time(inst: Instance, i: int, j: int) -> float:
    (x1, y1), (x2, y2) = inst.coords[i], inst.coords[j]
    return (abs(x1 - x2) + abs(y1 - y2)) / inst.truck_speed


def euclid_time(inst: Instance, i: int, j: int) -> float:
    (x1, y1), (x2, y2) = inst.coords[i], inst.coords[j]
    return math.hypot(x1 - x2, y1 - y2) / inst.drone_speed


def split_two_clusters(inst: Instance) -> Tuple[List[int], List[int]]:
    customers = list(range(1, len(inst.coords)))
    customers.sort(key=lambda c: (inst.coords[c][0], inst.coords[c][1]))
    mid = len(customers) // 2
    c1 = customers[:mid]
    c2 = customers[mid:]
    return c1, c2


def nearest_neighbor_route(inst: Instance, cluster: List[int]) -> List[int]:
    remain = set(cluster)
    route = [0]
    cur = 0
    while remain:
        nxt = min(remain, key=lambda c: manhattan_time(inst, cur, c))
        route.append(nxt)
        remain.remove(nxt)
        cur = nxt
    return route


def pick_trip_packages(route: List[int]) -> Tuple[List[int], List[int], List[int], List[int], List[int], List[int]]:
    # route format: [0, c1, c2, ...]
    seq = route[1:]
    # keep stable and deterministic picks
    t1 = seq[1:4] if len(seq) >= 4 else seq[:3]
    t2 = seq[2:6] if len(seq) >= 6 else seq[:4]
    m1 = seq[6:10] if len(seq) >= 10 else seq[:4]
    m2 = seq[10:12] if len(seq) >= 12 else seq[:2]
    x1 = seq[12:15] if len(seq) >= 15 else seq[:3]
    x2 = seq[15:18] if len(seq) >= 18 else seq[:3]
    return t1, t2, m1, m2, x1, x2


def compute_arrivals(inst: Instance, route: List[int]) -> dict[int, float]:
    arr = {route[0]: 0.0}
    t = 0.0
    for i in range(len(route) - 1):
        t += manhattan_time(inst, route[i], route[i + 1])
        arr[route[i + 1]] = t
    return arr


def build_solution(inst: Instance) -> Tuple[list, list, List[int]]:
    c1, c2 = split_two_clusters(inst)
    r1 = nearest_neighbor_route(inst, c1)
    r2 = nearest_neighbor_route(inst, c2)

    t1_pk, m1_pk, m1_mix_pk, x1_pk, _, _ = pick_trip_packages(r1)
    t2_pk, m2_pk, _, _, x2_pk, _ = pick_trip_packages(r2)

    # Planned 5 trips:
    # trip1: 3 pk for truck1
    # trip2: 4 pk for truck2
    # trip3: 6 pk mix (4 truck1 + 2 truck2)
    # trip4: 3 pk truck1
    # trip5: 3 pk truck2
    trip1_pk = t1_pk[:3]
    trip2_pk = t2_pk[:4]
    trip3_pk1 = m1_mix_pk[:4]
    trip3_pk2 = m2_pk[:2]
    trip4_pk = x1_pk[:3]
    trip5_pk = x2_pk[:3]

    # choose rendezvous nodes by first package in each bucket
    rv1 = trip1_pk[0] if trip1_pk else r1[min(1, len(r1) - 1)]
    rv2 = trip2_pk[0] if trip2_pk else r2[min(1, len(r2) - 1)]
    rv3a = trip3_pk1[0] if trip3_pk1 else rv1
    rv3b = trip3_pk2[0] if trip3_pk2 else rv2
    rv4 = trip4_pk[0] if trip4_pk else rv1
    rv5 = trip5_pk[0] if trip5_pk else rv2

    drone_queue = []
    if trip1_pk:
        drone_queue.append([[rv1, trip1_pk]])
    if trip2_pk:
        drone_queue.append([[rv2, trip2_pk]])
    if trip3_pk1 or trip3_pk2:
        events = []
        if trip3_pk1:
            events.append([rv3a, trip3_pk1])
        if trip3_pk2:
            events.append([rv3b, trip3_pk2])
        drone_queue.append(events)
    if trip4_pk:
        drone_queue.append([[rv4, trip4_pk]])
    if trip5_pk:
        drone_queue.append([[rv5, trip5_pk]])

    # package -> whether resupplied
    resup = set()
    for trip in drone_queue:
        for _, pk in trip:
            resup.update(pk)

    # truck stop encoding: [customer, [packages delivered/resupplied at this stop]]
    stops1 = [[0, []]] + [[c, []] for c in r1[1:]]
    stops2 = [[0, []]] + [[c, []] for c in r2[1:]]
    idx1 = {c: i for i, (c, _) in enumerate(stops1)}
    idx2 = {c: i for i, (c, _) in enumerate(stops2)}

    for trip in drone_queue:
        for rv, pk in trip:
            if rv in idx1:
                stops1[idx1[rv]][1].extend(pk)
            elif rv in idx2:
                stops2[idx2[rv]][1].extend(pk)

    sol = [[stops1, stops2], drone_queue]
    return sol, [r1, r2], sorted(resup)


def assign_release_from_timeline(inst: Instance, solution: list, routes: List[List[int]]) -> List[int]:
    trucks, drone_q = solution

    # arrival map (truck starts at 0)
    arr1 = compute_arrivals(inst, routes[0])
    arr2 = compute_arrivals(inst, routes[1])
    owner = {}
    for c in routes[0]:
        owner[c] = 0
    for c in routes[1]:
        owner[c] = 1

    # release init: non-resupply=0, resupply=1
    n = len(inst.coords)
    rel = [0] * n
    for trip in drone_q:
        for _, pk in trip:
            for p in pk:
                rel[p] = 1

    avail = 0.0
    for trip in drone_q:
        if not trip:
            continue
        max_rel = max((rel[p] for _, pk in trip for p in pk), default=0)
        first = trip[0][0]
        arr_first = arr1[first] if owner[first] == 0 else arr2[first]
        launch_earliest = arr_first - euclid_time(inst, 0, first)
        depart = max(avail, float(max_rel), launch_earliest)

        # cumulative release by trip depart
        r_trip = int(math.ceil(depart))
        for _, pk in trip:
            for p in pk:
                rel[p] = r_trip

        # simulate trip duration to update drone availability
        t = depart
        last = 0
        for rv, _pk in trip:
            t += euclid_time(inst, last, rv)
            arr_rv = arr1[rv] if owner[rv] == 0 else arr2[rv]
            if t < arr_rv:
                t = arr_rv
            t += inst.sigma
            last = rv
        t += euclid_time(inst, last, 0)
        avail = t

    rel[0] = 0
    return rel


def write_dat(inst: Instance, out_path: Path, release: List[int], a_val: float, l_val: float) -> None:
    out = []
    out.append("number_truck\t2")
    out.append("number_drone\t1")
    out.append(f"truck_speed\t{inst.truck_speed:g}")
    out.append(f"drone_speed\t{inst.drone_speed:g}")
    out.append(f"M_d\t{a_val:g}")
    out.append(f"L_d\t{l_val:g}")
    out.append(f"Sigma\t{inst.sigma:g}")
    out.append("XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE")
    for i, (x, y) in enumerate(inst.coords):
        xi = int(x) if float(x).is_integer() else x
        yi = int(y) if float(y).is_integer() else y
        out.append(f"{xi}\t{yi}\t{inst.demand[i]}\t{release[i]}")
    out_path.write_text("\n".join(out) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate purposeful C101 batch instances + seeded solution")
    ap.add_argument("--input-dir", default="test_data/data_demand_random_50_batch_all1_equal_cluster")
    ap.add_argument("--pattern", default="C101_*.dat")
    ap.add_argument("--output-dir", default="result/generated_instances/batch_c101_purposeful")
    ap.add_argument("--A", type=float, default=4.0)
    ap.add_argument("--L", type=float, default=120.0)
    args = ap.parse_args()

    in_dir = Path(args.input_dir)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    files = sorted(in_dir.glob(args.pattern))
    if not files:
        raise SystemExit(f"No files matched: {in_dir}/{args.pattern}")

    summary = []
    for path in files:
        inst = parse_dat(path)
        solution, routes, resup = build_solution(inst)
        rel = assign_release_from_timeline(inst, solution, routes)

        base = path.stem
        out_dat = out_dir / f"{base}_A{int(args.A)}_L{int(args.L)}_purposeful.dat"
        out_sol = out_dir / f"{base}_A{int(args.A)}_L{int(args.L)}_purposeful_solution.json"
        write_dat(inst, out_dat, rel, args.A, args.L)
        out_sol.write_text(json.dumps(solution, ensure_ascii=True))

        summary.append(
            {
                "instance": path.name,
                "out_dat": str(out_dat),
                "out_solution": str(out_sol),
                "n_customers": len(inst.coords) - 1,
                "n_resupply": len(resup),
                "n_trips": len(solution[1]),
            }
        )

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=True))
    print(f"Generated {len(summary)} instances in {out_dir}")


if __name__ == "__main__":
    main()
