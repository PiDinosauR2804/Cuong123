#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass
class Instance:
    meta: Dict[str, str]
    coords: List[Tuple[float, float]]
    demand: List[int]
    release: List[int]


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
        raise ValueError(f"Invalid .dat: {path}")

    coords: List[Tuple[float, float]] = []
    demand: List[int] = []
    release: List[int] = []
    for ln in lines[start:]:
        x, y, d, r = ln.split()[:4]
        coords.append((float(x), float(y)))
        demand.append(int(float(d)))
        release.append(int(float(r)))
    return Instance(meta=meta, coords=coords, demand=demand, release=release)


def beta_from_name(path: Path) -> float:
    return float(path.stem.split("_")[-1])


def beta_ranges(beta: float) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    # trips_range, multi_range
    table = {
        0.5: ((4, 5), (2, 2)),
        1.0: ((4, 6), (1, 3)),  # beta=1 uses half-trips rule below
        1.5: ((5, 7), (2, 4)),
        2.0: ((6, 9), (2, 4)),
        2.5: ((7, 12), (3, 5)),
        3.0: ((7, 14), (4, 5)),
    }
    if beta not in table:
        raise ValueError(f"Unsupported beta={beta}; expected one of {sorted(table.keys())}")
    return table[beta]


def manhattan(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def kmeans5(coords: List[Tuple[float, float]], rng: random.Random, max_iter: int = 40) -> List[int]:
    pts = coords[1:]  # customers only
    n = len(pts)
    if n < 5:
        return [0] * n

    seeds = rng.sample(range(n), 5)
    centers = [pts[i] for i in seeds]
    labels = [0] * n

    for _ in range(max_iter):
        changed = False
        for i, p in enumerate(pts):
            best = min(range(5), key=lambda c: (p[0] - centers[c][0]) ** 2 + (p[1] - centers[c][1]) ** 2)
            if labels[i] != best:
                labels[i] = best
                changed = True

        new_centers: List[Tuple[float, float]] = []
        for c in range(5):
            group = [pts[i] for i in range(n) if labels[i] == c]
            if not group:
                # re-seed empty cluster
                new_centers.append(pts[rng.randrange(n)])
            else:
                new_centers.append((sum(x for x, _ in group) / len(group), sum(y for _, y in group) / len(group)))
        centers = new_centers
        if not changed:
            break
    return labels


def route_eta(route: List[int], coords: List[Tuple[float, float]], truck_speed: float) -> Dict[int, float]:
    eta: Dict[int, float] = {route[0]: 0.0}
    t = 0.0
    for i in range(len(route) - 1):
        t += manhattan(coords[route[i]], coords[route[i + 1]]) / truck_speed
        eta[route[i + 1]] = t
    return eta


def nearest_neighbor_order(customers: List[int], coords: List[Tuple[float, float]], start_node: int = 0) -> List[int]:
    remain = set(customers)
    cur = start_node
    out: List[int] = []
    while remain:
        nxt = min(remain, key=lambda c: manhattan(coords[cur], coords[c]))
        out.append(nxt)
        remain.remove(nxt)
        cur = nxt
    return out


def build_two_truck_routes(inst: Instance, labels: List[int], rng: random.Random) -> Tuple[List[int], List[int]]:
    # cluster id for customer c is labels[c-1]
    cluster_customers: Dict[int, List[int]] = {k: [] for k in range(5)}
    for c in range(1, len(inst.coords)):
        cluster_customers[labels[c - 1]].append(c)

    # choose starting clusters by farthest centroids to spread 2 trucks
    centroids: Dict[int, Tuple[float, float]] = {}
    for cid in range(5):
        group = cluster_customers[cid]
        if not group:
            centroids[cid] = inst.coords[0]
        else:
            centroids[cid] = (
                sum(inst.coords[c][0] for c in group) / len(group),
                sum(inst.coords[c][1] for c in group) / len(group),
            )
    pairs = [(i, j) for i in range(5) for j in range(i + 1, 5)]
    s1, s2 = max(pairs, key=lambda p: manhattan(centroids[p[0]], centroids[p[1]]))
    t1_clusters = [s1]
    t2_clusters = [s2]

    rest = [c for c in range(5) if c not in {s1, s2}]
    rng.shuffle(rest)
    # assign by balancing route length proxy + size
    for cid in rest:
        n1 = sum(len(cluster_customers[x]) for x in t1_clusters)
        n2 = sum(len(cluster_customers[x]) for x in t2_clusters)
        d1 = min(manhattan(centroids[cid], centroids[x]) for x in t1_clusters)
        d2 = min(manhattan(centroids[cid], centroids[x]) for x in t2_clusters)
        score1 = n1 + 0.2 * d1
        score2 = n2 + 0.2 * d2
        if score1 <= score2:
            t1_clusters.append(cid)
        else:
            t2_clusters.append(cid)

    # per truck: visit clusters in assigned order, each cluster NN internally
    def route_for(clusters: List[int]) -> List[int]:
        route = [0]
        cur = 0
        # cluster visit order by nearest centroid from current
        remain = set(clusters)
        ordered: List[int] = []
        cur_xy = inst.coords[0]
        while remain:
            nxt = min(remain, key=lambda c: manhattan(cur_xy, centroids[c]))
            ordered.append(nxt)
            cur_xy = centroids[nxt]
            remain.remove(nxt)
        for cid in ordered:
            cust = cluster_customers[cid]
            if not cust:
                continue
            seq = nearest_neighbor_order(cust, inst.coords, cur)
            route.extend(seq)
            cur = seq[-1]
        return route

    return route_for(t1_clusters), route_for(t2_clusters)


def choose_trip_sizes(rng: random.Random, n_trips: int, total_cap: int) -> List[int]:
    # each trip in [3,8], with sum <= total_cap
    sizes = [rng.randint(3, 8) for _ in range(n_trips)]
    s = sum(sizes)
    while s > total_cap:
        idx = max(range(n_trips), key=lambda i: sizes[i])
        if sizes[idx] > 3:
            sizes[idx] -= 1
            s -= 1
        else:
            # all at min=3 -> must cut trip count
            break
    # if still overflow (when 3*n_trips > total_cap), shrink by dropping tail trips to 3
    while sum(sizes) > total_cap and sizes:
        sizes.pop()
    return sizes


def build_solution_with_trips(
    inst: Instance,
    route1: List[int],
    route2: List[int],
    beta: float,
    rng: random.Random,
) -> Tuple[list, Dict[str, int], List[int]]:
    trips_range, multi_range = beta_ranges(beta)
    n_trips = rng.randint(trips_range[0], trips_range[1])

    # exactly 5 zeros required => at most 45 customers can be resupplied
    total_customers = len(inst.coords) - 1
    max_resup_customers = max(0, total_customers - 5)

    # ensure feasible trip count with min 3 pkg/trip
    n_trips = min(n_trips, max_resup_customers // 3 if max_resup_customers >= 3 else 0)
    if n_trips <= 0:
        n_trips = 1

    if beta == 1.0:
        # User rule: about half of trips are multi-visit
        n_multi = n_trips // 2
    else:
        n_multi = rng.randint(multi_range[0], multi_range[1])
        n_multi = min(n_multi, n_trips)

    sizes = choose_trip_sizes(rng, n_trips, max_resup_customers)
    n_trips = len(sizes)
    n_multi = min(n_multi, n_trips)

    p1 = route1[1:]
    p2 = route2[1:]
    eta1 = route_eta(route1, inst.coords, float(inst.meta["truck_speed"]))
    eta2 = route_eta(route2, inst.coords, float(inst.meta["truck_speed"]))

    used: set[int] = set()
    pos1 = {c: i for i, c in enumerate(p1)}
    pos2 = {c: i for i, c in enumerate(p2)}

    def take(pool: List[int], k: int) -> List[int]:
        out: List[int] = []
        for c in pool:
            if c in used:
                continue
            used.add(c)
            out.append(c)
            if len(out) == k:
                break
        return out

    def available(pool: List[int]) -> int:
        return sum(1 for c in pool if c not in used)

    def take_chunk(route_nodes: List[int], pos: Dict[int, int], center: int, k: int) -> List[int]:
        if center not in pos:
            return take(route_nodes, k)
        out: List[int] = []
        idx = pos[center]
        radius = 0
        n = len(route_nodes)
        while len(out) < k and radius < n:
            for j in (idx - radius, idx + radius):
                if 0 <= j < n:
                    c = route_nodes[j]
                    if c not in used and c not in out:
                        out.append(c)
                        used.add(c)
                        if len(out) == k:
                            break
            radius += 1
        return out

    # candidate synchronized pairs for multi-trips
    pairs = []
    for a in p1:
        for b in p2:
            pairs.append((abs(eta1[a] - eta2[b]), a, b))
    pairs.sort(key=lambda x: x[0])

    drone_q: List[List[List[int]]] = []
    pair_ptr = 0
    single_cursor1 = 0
    single_cursor2 = 0

    def next_single(truck: int) -> int:
        nonlocal single_cursor1, single_cursor2
        if truck == 1:
            while single_cursor1 < len(p1) and p1[single_cursor1] in used:
                single_cursor1 += 1
            return p1[single_cursor1] if single_cursor1 < len(p1) else p1[0]
        while single_cursor2 < len(p2) and p2[single_cursor2] in used:
            single_cursor2 += 1
        return p2[single_cursor2] if single_cursor2 < len(p2) else p2[0]

    # interleave multi/single to avoid very rigid "first n_multi are multi"
    multi_slots = set(range(0, n_trips, max(1, n_trips // max(1, n_multi)))) if n_multi > 0 else set()
    while len(multi_slots) > n_multi:
        multi_slots.pop()

    for i in range(n_trips):
        sz = sizes[i]
        multi = i in multi_slots
        if multi:
            # Enforce strict rule: a multi-visit trip must touch two different trucks.
            # So we only pick packages from truck-1 pool for event-1 and truck-2 pool for event-2.
            a1 = available(p1)
            a2 = available(p2)
            if a1 <= 0 or a2 <= 0:
                # cannot build a valid cross-truck multi-trip anymore
                continue

            k1 = max(1, int(round(sz * 0.55)))
            k2 = max(1, sz - k1)
            k1 = min(k1, a1)
            k2 = min(k2, a2)
            # keep at least 1 package per side in multi-trip
            if k1 <= 0 or k2 <= 0:
                continue
            while k1 + k2 > sz:
                if k1 >= k2 and k1 > 1:
                    k1 -= 1
                elif k2 > 1:
                    k2 -= 1
                else:
                    break
            while k1 + k2 < max(3, sz):
                if a1 - k1 >= a2 - k2 and k1 < a1:
                    k1 += 1
                elif k2 < a2:
                    k2 += 1
                else:
                    break

            a = p1[0] if p1 else 0
            b = p2[0] if p2 else 0
            while pair_ptr < len(pairs):
                _, aa, bb = pairs[pair_ptr]
                pair_ptr += 1
                if aa not in used or bb not in used:
                    a, b = aa, bb
                    break
            pk1 = take_chunk(p1, pos1, a, k1)
            pk2 = take_chunk(p2, pos2, b, k2)
            # No cross-fill here; otherwise a trip can hit the same truck twice.
            if len(pk1) == 0 or len(pk2) == 0:
                continue
            trip: List[List[int]] = []
            if pk1:
                trip.append([a if a in pk1 else pk1[0], pk1])
            if pk2:
                trip.append([b if b in pk2 else pk2[0], pk2])
            drone_q.append(trip)
        else:
            pick_t1 = (i % 2 == 0)
            if pick_t1:
                rv = next_single(1)
                pk = take_chunk(p1, pos1, rv, sz)
            else:
                rv = next_single(2)
                pk = take_chunk(p2, pos2, rv, sz)
            # keep single-trip on one truck side only
            if len(pk) < 3:
                continue
            drone_q.append([[rv if rv in pk else pk[0], pk]])

    # construct truck stops
    stops1 = [[0, []]] + [[c, []] for c in route1[1:]]
    stops2 = [[0, []]] + [[c, []] for c in route2[1:]]
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
        "actual_trips": len(drone_q),
        "actual_multi": sum(1 for t in drone_q if len(t) >= 2),
        "trip_pkg_min": min(sum(len(pk) for _, pk in t) for t in drone_q) if drone_q else 0,
        "trip_pkg_max": max(sum(len(pk) for _, pk in t) for t in drone_q) if drone_q else 0,
        "resupply_count": len(used),
    }
    return sol, info, sorted(used)


def assign_release(inst: Instance, n: int, resupplied: List[int], sol: list, route1: List[int], route2: List[int], max_release: int, rng: random.Random) -> List[int]:
    rel = [0] * n
    # exactly 5 zeros total (excluding depot 0 is also zero)
    # choose 5 non-resupplied customers to stay zero
    all_customers = set(range(1, n))
    res = set(resupplied)
    non = sorted(all_customers - res)
    rng.shuffle(non)
    zero5 = set(non[:5]) if len(non) >= 5 else set(non)

    # ETA-based raw releases then monotone smoothing
    eta1 = route_eta(route1, inst.coords, float(inst.meta["truck_speed"]))
    eta2 = route_eta(route2, inst.coords, float(inst.meta["truck_speed"]))
    owner = {c: 1 for c in route1}
    owner.update({c: 2 for c in route2})
    trips = sol[1]
    raw: List[int] = []
    for trip in trips:
        if not trip:
            raw.append(1)
            continue
        first = trip[0][0]
        eta = eta1.get(first, 0.0) if owner.get(first, 1) == 1 else eta2.get(first, 0.0)
        # small jitter to avoid flat plateaus
        base = max(1, int(round(eta * 0.8 + rng.uniform(-2.0, 2.0))))
        raw.append(base)
    # isotonic non-decreasing
    smooth = raw[:]
    for i in range(1, len(smooth)):
        if smooth[i] < smooth[i - 1]:
            smooth[i] = smooth[i - 1]
    # rescale to max_release while preserving order
    mn = min(smooth) if smooth else 1
    mx = max(smooth) if smooth else 1
    scaled: List[int] = []
    for i, s in enumerate(smooth):
        if i == len(smooth) - 1:
            r = max_release
        elif mx == mn:
            r = max(1, int(round(max_release * (i + 1) / max(1, len(smooth)))))
        else:
            r = int(round((s - mn) * (max_release - 1) / (mx - mn) + 1))
            r = min(r, max_release - 1)
        scaled.append(max(1, r))

    for i, trip in enumerate(trips):
        r = scaled[i] if i < len(scaled) else 1
        for _, pk in trip:
            for p in pk:
                rel[p] = int(r)

    # enforce only ~5 zeros among customers; other non-resupply set to small positive
    for c in range(1, n):
        if c in zero5:
            rel[c] = 0
        elif rel[c] == 0:
            rel[c] = 1
    rel[0] = 0
    return rel


def write_dat(inst: Instance, rel: List[int], out_path: Path) -> None:
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
        out.append(f"{xi}\t{yi}\t{inst.demand[i]}\t{rel[i]}")
    out_path.write_text("\n".join(out) + "\n")


def validate_feasible(dat_path: Path, sol: list) -> bool:
    """
    Validate seed solution using Function6 validator path (same feasibility core as fitness_full flow).
    """
    bin_path = Path("C_Version/read_data_f6")
    if not bin_path.exists():
        return False
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as tf:
        tf.write(json.dumps(sol, ensure_ascii=True))
        seed_path = tf.name
    try:
        # read M_d and L_d directly from dat header
        lines = [ln.strip() for ln in dat_path.read_text().splitlines() if ln.strip()]
        md = None
        ld = None
        for ln in lines:
            tok = ln.split()
            if tok[0] == "M_d":
                md = tok[-1]
            elif tok[0] == "L_d":
                ld = tok[-1]
            elif tok[0] == "XCOORD":
                break
        if md is None or ld is None:
            return False
        env = os.environ.copy()
        env["VALIDATE_ONLY"] = "1"
        p = subprocess.run(
            [str(bin_path), str(dat_path), str(md), str(ld), seed_path],
            env=env,
            capture_output=True,
            text=True,
        )
        out = (p.stdout or "") + (p.stderr or "")
        return "[VALIDATE] OK" in out
    finally:
        try:
            Path(seed_path).unlink(missing_ok=True)
        except Exception:
            pass


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-dir", default="test_data/data_demand_random_50_batch_all1_equal_cluster")
    ap.add_argument("--glob", default="C101_*.dat")
    ap.add_argument("--output-dir", default="result/generated_instances/c101_cluster5_strategies")
    ap.add_argument("--strategies-per-instance", type=int, default=3)
    ap.add_argument("--seed", type=int, default=20260330)
    ap.add_argument("--max-attempts-per-strategy", type=int, default=120)
    args = ap.parse_args()

    in_dir = Path(args.input_dir)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    files = sorted(in_dir.glob(args.glob))
    # only requested betas
    files = [p for p in files if beta_from_name(p) in {0.5, 1.0, 1.5, 2.0, 2.5, 3.0}]
    if not files:
        raise SystemExit("No matching C101 beta files found.")

    rng = random.Random(args.seed)
    summary: List[dict] = []
    for path in files:
        beta = beta_from_name(path)
        inst = parse_dat(path)
        max_rel = max(inst.release)

        for sidx in range(1, args.strategies_per_instance + 1):
            stem = path.stem
            tag = f"strategy{sidx}"
            out_dat = out_dir / f"{stem}_{tag}.dat"
            out_sol = out_dir / f"{stem}_{tag}_solution.json"

            accepted = False
            info = {}
            zero_customers = 0
            attempts = 0
            for attempts in range(1, args.max_attempts_per_strategy + 1):
                local_rng = random.Random(rng.randint(0, 10**9))
                labels = kmeans5(inst.coords, local_rng)
                r1, r2 = build_two_truck_routes(inst, labels, local_rng)
                sol, info, resup = build_solution_with_trips(inst, r1, r2, beta, local_rng)
                rel = assign_release(inst, len(inst.coords), resup, sol, r1, r2, max_rel, local_rng)

                if max(rel) != max_rel:
                    continue
                zero_customers = sum(1 for x in rel[1:] if x == 0)
                if zero_customers != 5:
                    extra = [i for i in range(1, len(rel)) if rel[i] == 0]
                    for i in extra[5:]:
                        rel[i] = 1
                    zero_customers = sum(1 for x in rel[1:] if x == 0)
                if zero_customers != 5:
                    continue

                write_dat(inst, rel, out_dat)
                out_sol.write_text(json.dumps(sol, ensure_ascii=True))
                if validate_feasible(out_dat, sol):
                    accepted = True
                    break

            if not accepted:
                raise RuntimeError(
                    f"Could not generate feasible solution for {path.name} strategy {sidx} "
                    f"after {args.max_attempts_per_strategy} attempts"
                )

            summary.append(
                {
                    "instance": path.name,
                    "beta": beta,
                    "strategy": sidx,
                    "attempts": attempts,
                    "actual_trips": info["actual_trips"],
                    "actual_multi": info["actual_multi"],
                    "trip_pkg_min": info["trip_pkg_min"],
                    "trip_pkg_max": info["trip_pkg_max"],
                    "max_release_input": max_rel,
                    "max_release_output": max_rel,
                    "zero_release_customers": zero_customers,
                    "out_dat": str(out_dat),
                    "out_solution": str(out_sol),
                }
            )

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=True))
    print(f"Generated {len(summary)} strategy instances in {out_dir}")


if __name__ == "__main__":
    main()
