#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass
class Instance:
    meta: Dict[str, str]
    coords: List[Tuple[float, float]]
    demand: List[int]
    release: List[int]


@dataclass
class Plan:
    trips: int
    multi: int


def parse_dat(path: Path) -> Instance:
    lines = [ln.strip() for ln in path.read_text().splitlines() if ln.strip()]
    meta: Dict[str, str] = {}
    start = None
    for i, ln in enumerate(lines):
        tok = ln.split()
        if tok[0] == "XCOORD":
            start = i + 1
            break
        meta[tok[0]] = tok[-1]
    if start is None:
        raise ValueError(f"Invalid .dat file: {path}")

    coords: List[Tuple[float, float]] = []
    demand: List[int] = []
    release: List[int] = []
    for ln in lines[start:]:
        x, y, d, r = ln.split()[:4]
        coords.append((float(x), float(y)))
        demand.append(int(float(d)))
        release.append(int(float(r)))
    return Instance(meta=meta, coords=coords, demand=demand, release=release)


def parse_beta(path: Path) -> float:
    # C101_0.5.dat -> 0.5
    stem = path.stem
    return float(stem.split("_")[-1])


def beta_plan(beta: float) -> Plan:
    # Target "đều đều" values inside the requested ranges
    mapping = {
        0.5: Plan(trips=2, multi=1),
        1.0: Plan(trips=3, multi=2),   # ~1/2
        1.5: Plan(trips=5, multi=3),
        2.0: Plan(trips=6, multi=3),
        2.5: Plan(trips=8, multi=4),
        3.0: Plan(trips=10, multi=4),
    }
    if beta not in mapping:
        raise ValueError(f"Unsupported beta={beta}, expected one of {sorted(mapping.keys())}")
    return mapping[beta]


def manhattan(i: int, j: int, coords: List[Tuple[float, float]]) -> float:
    (x1, y1), (x2, y2) = coords[i], coords[j]
    return abs(x1 - x2) + abs(y1 - y2)


def split_clusters(coords: List[Tuple[float, float]]) -> Tuple[List[int], List[int]]:
    customers = list(range(1, len(coords)))
    customers.sort(key=lambda c: (coords[c][0], coords[c][1]))
    mid = len(customers) // 2
    return customers[:mid], customers[mid:]


def nn_route(cluster: List[int], coords: List[Tuple[float, float]]) -> List[int]:
    remain = set(cluster)
    cur = 0
    route = [0]
    while remain:
        nxt = min(remain, key=lambda c: manhattan(cur, c, coords))
        route.append(nxt)
        remain.remove(nxt)
        cur = nxt
    return route


def take_from_pool(pool: List[int], used: set[int], k: int) -> List[int]:
    out: List[int] = []
    for c in pool:
        if c in used:
            continue
        out.append(c)
        used.add(c)
        if len(out) == k:
            break
    return out


def build_solution(coords: List[Tuple[float, float]], plan: Plan) -> Tuple[list, Dict[str, int]]:
    c1, c2 = split_clusters(coords)
    r1 = nn_route(c1, coords)
    r2 = nn_route(c2, coords)

    used: set[int] = set()
    p1 = r1[1:]
    p2 = r2[1:]

    # deterministic package sizes in [3,7]
    sizes = [3 + (i % 5) for i in range(plan.trips)]

    # mark first `multi` trips as multi-visit to guarantee count
    is_multi = [True] * plan.multi + [False] * (plan.trips - plan.multi)

    drone_q: List[List[List[int]]] = []
    for i in range(plan.trips):
        sz = sizes[i]
        if is_multi[i]:
            k1 = max(1, int(round(sz * 0.6)))
            k2 = max(1, sz - k1)
            pk1 = take_from_pool(p1, used, k1)
            pk2 = take_from_pool(p2, used, k2)
            # fallback if one side short
            if len(pk1) < k1:
                pk1 += take_from_pool(p2, used, k1 - len(pk1))
            if len(pk2) < k2:
                pk2 += take_from_pool(p1, used, k2 - len(pk2))
            if not pk1 and not pk2:
                continue
            events: List[List[int]] = []
            if pk1:
                events.append([pk1[0], pk1])
            if pk2:
                events.append([pk2[0], pk2])
            drone_q.append(events)
        else:
            choose_t1 = (i % 2 == 0)
            pool = p1 if choose_t1 else p2
            other = p2 if choose_t1 else p1
            pk = take_from_pool(pool, used, sz)
            if len(pk) < sz:
                pk += take_from_pool(other, used, sz - len(pk))
            if not pk:
                continue
            drone_q.append([[pk[0], pk]])

    # truck stops with resupply lists
    stops1 = [[0, []]] + [[c, []] for c in r1[1:]]
    stops2 = [[0, []]] + [[c, []] for c in r2[1:]]
    idx1 = {c: i for i, (c, _) in enumerate(stops1)}
    idx2 = {c: i for i, (c, _) in enumerate(stops2)}
    for trip in drone_q:
        for rv, pk in trip:
            if rv in idx1:
                stops1[idx1[rv]][1].extend(pk)
            elif rv in idx2:
                stops2[idx2[rv]][1].extend(pk)

    sol = [[stops1, stops2], drone_q]
    info = {
        "trip_count": len(drone_q),
        "multi_count": sum(1 for t in drone_q if len(t) >= 2),
        "resupply_count": len(used),
    }
    return sol, info


def assign_release(sol: list, n: int, max_release: int) -> List[int]:
    _, drone_q = sol
    rel = [0] * n
    if not drone_q:
        return rel
    m = len(drone_q)
    for i, trip in enumerate(drone_q, start=1):
        # monotonic wave, final trip gets exact max_release
        if i == m:
            r = max_release
        else:
            r = max(1, round(max_release * (i / m)))
        for _, pk in trip:
            for p in pk:
                rel[p] = int(r)
    rel[0] = 0
    return rel


def write_dat(inst: Instance, release: List[int], out_path: Path) -> None:
    out: List[str] = []
    out.append(f"number_truck\t{inst.meta.get('number_truck', '2')}")
    out.append("number_drone\t1")
    out.append(f"truck_speed\t{inst.meta['truck_speed']}")
    out.append(f"drone_speed\t{inst.meta['drone_speed']}")
    out.append(f"M_d\t{inst.meta['M_d']}")
    out.append(f"L_d\t{inst.meta['L_d']}")
    out.append(f"Sigma\t{inst.meta['Sigma']}")
    out.append("XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE")
    for i, (x, y) in enumerate(inst.coords):
        xi = int(x) if float(x).is_integer() else x
        yi = int(y) if float(y).is_integer() else y
        out.append(f"{xi}\t{yi}\t{inst.demand[i]}\t{release[i]}")
    out_path.write_text("\n".join(out) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-dir", default="test_data/data_demand_random_50_batch_all1_equal_cluster")
    ap.add_argument("--glob", default="C101_*.dat")
    ap.add_argument("--output-dir", default="result/generated_instances/c101_beta_release_batches")
    args = ap.parse_args()

    in_dir = Path(args.input_dir)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    files = sorted(in_dir.glob(args.glob))
    if not files:
        raise SystemExit(f"No inputs matched: {in_dir}/{args.glob}")

    summary = []
    for path in files:
        beta = parse_beta(path)
        inst = parse_dat(path)
        plan = beta_plan(beta)
        sol, info = build_solution(inst.coords, plan)
        max_rel = max(inst.release)
        rel = assign_release(sol, len(inst.coords), max_rel)
        if max(rel) != max_rel:
            raise RuntimeError(f"max release mismatch for {path.name}: got {max(rel)} expected {max_rel}")

        stem = path.stem
        out_dat = out_dir / f"{stem}_beta_pattern.dat"
        out_sol = out_dir / f"{stem}_beta_pattern_solution.json"
        write_dat(inst, rel, out_dat)
        out_sol.write_text(json.dumps(sol, ensure_ascii=True))

        trips = info["trip_count"]
        multi = info["multi_count"]
        pk_counts = []
        for trip in sol[1]:
            c = 0
            for _, pk in trip:
                c += len(pk)
            pk_counts.append(c)

        summary.append(
            {
                "instance": path.name,
                "beta": beta,
                "target_trips": plan.trips,
                "target_multi": plan.multi,
                "actual_trips": trips,
                "actual_multi": multi,
                "trip_pkg_min": min(pk_counts) if pk_counts else 0,
                "trip_pkg_max": max(pk_counts) if pk_counts else 0,
                "max_release_input": max_rel,
                "max_release_output": max(rel),
                "out_dat": str(out_dat),
                "out_solution": str(out_sol),
            }
        )

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=True))
    print(f"Generated {len(summary)} files in {out_dir}")


if __name__ == "__main__":
    main()
