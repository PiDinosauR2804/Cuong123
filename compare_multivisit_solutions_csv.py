import ast
import csv
import math
import pathlib
import sys
from typing import Any, Dict, List, Optional, Tuple


def _f(x: str) -> float:
    s = (x or "").strip()
    if not s:
        return math.inf
    try:
        return float(s)
    except Exception:
        return math.inf


def _i(x: str) -> int:
    s = (x or "").strip()
    if not s:
        return 0
    try:
        return int(float(s))
    except Exception:
        return 0


def parse_solution_text(solution_text: str) -> Optional[Any]:
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


def infer_trip_trucks(trucks_raw: Any, trip_events: Any) -> Tuple[List[int], List[int]]:
    """
    Infer truck_id per event by matching rendezvous city and subset of packages
    against the truck routes (same logic style as recompute_metrics_from_solutions_csv.py).
    Returns:
      event_trucks: list of inferred truck_id (or -1)
      event_stop_idx: list of inferred stop index (or -1)
    """
    truck_routes: List[List[Tuple[int, List[int]]]] = []
    for tr in trucks_raw:
        route: List[Tuple[int, List[int]]] = []
        for stop in tr:
            city = int(stop[0])
            pkgs = [int(x) for x in stop[1]]
            route.append((city, pkgs))
        truck_routes.append(route)

    stop_candidates: Dict[int, List[Tuple[int, int, List[int]]]] = {}
    for tid, route in enumerate(truck_routes):
        for idx, (city, pkgs) in enumerate(route):
            if city == 0:
                continue
            stop_candidates.setdefault(city, []).append((tid, idx, pkgs))

    event_trucks: List[int] = []
    event_stop_idx: List[int] = []
    for ev in trip_events:
        city = int(ev[0])
        pkgs = [int(x) for x in ev[1]]
        cands = stop_candidates.get(city, [])
        sp = set(pkgs)
        chosen = None
        for tid, idx, spkgs in cands:
            if sp.issubset(set(spkgs)):
                chosen = (tid, idx)
                break
        if chosen is None and cands:
            chosen = (cands[0][0], cands[0][1])
        if chosen is None:
            event_trucks.append(-1)
            event_stop_idx.append(-1)
        else:
            event_trucks.append(int(chosen[0]))
            event_stop_idx.append(int(chosen[1]))
    return event_trucks, event_stop_idx


def summarize_multi(sol: Any) -> Dict[str, Any]:
    """
    Summarize solution structure without instance simulation:
    - trip_count, leg_count (events), mv_trip_count (>=2 distinct trucks per trip)
    - duplicates: packages resupplied > 1 time
    """
    if sol is None:
        return {
            "trip_count": 0,
            "leg_count": 0,
            "mv_trip_count": 0,
            "dup_pkg_cnt": 0,
            "dup_pkgs": "",
            "mv_trip_details": "",
        }
    trucks_raw, drone_raw = sol
    trip_count = len(drone_raw)
    leg_count = 0
    mv_trip_count = 0
    mv_details: List[str] = []

    pkg_counts: Dict[int, int] = {}
    for trip_idx, trip in enumerate(drone_raw):
        leg_count += len(trip)
        ev_trucks, _ = infer_trip_trucks(trucks_raw, trip)
        uniq = sorted({t for t in ev_trucks if t >= 0})
        if len(uniq) >= 2:
            mv_trip_count += 1
            mv_details.append(f"{trip_idx}:trucks={uniq},events={len(trip)}")
        for ev in trip:
            for pk in ev[1]:
                pki = int(pk)
                if pki <= 0:
                    continue
                pkg_counts[pki] = pkg_counts.get(pki, 0) + 1

    dup = sorted([k for k, v in pkg_counts.items() if v > 1])
    return {
        "trip_count": trip_count,
        "leg_count": leg_count,
        "mv_trip_count": mv_trip_count,
        "dup_pkg_cnt": len(dup),
        "dup_pkgs": ",".join(str(x) for x in dup[:40]),
        "mv_trip_details": " | ".join(mv_details[:20]),
    }


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage: python3 compare_multivisit_solutions_csv.py <old.csv> <new.csv>")
        return 2
    oldp = pathlib.Path(sys.argv[1])
    newp = pathlib.Path(sys.argv[2])
    if not oldp.exists() or not newp.exists():
        print("Missing file(s). old:", oldp, "new:", newp)
        return 2

    old_rows: Dict[str, Dict[str, str]] = {}
    with oldp.open(newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            old_rows[(row.get("job_id") or "").strip()] = row

    out_rows: List[Dict[str, str]] = []
    improved = 0
    tie_mv = 0
    tie_trip = 0
    new_multi = 0

    with newp.open(newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            jid = (row.get("job_id") or "").strip()
            old = old_rows.get(jid, {})
            old_fit = _f(old.get("best_multi_fitness", ""))
            new_fit = _f(row.get("best_multi_fitness", ""))

            # Parse and summarize structure for old/new
            old_sol = parse_solution_text(old.get("best_multi_solution", ""))
            new_sol = parse_solution_text(row.get("best_multi_solution", ""))
            old_sum = summarize_multi(old_sol)
            new_sum = summarize_multi(new_sol)

            if old_fit == math.inf and new_fit < math.inf:
                new_multi += 1
            if new_fit + 1e-9 < old_fit:
                improved += 1
            elif abs(new_fit - old_fit) <= 1e-9:
                if new_sum["mv_trip_count"] > old_sum["mv_trip_count"]:
                    tie_mv += 1
                elif new_sum["mv_trip_count"] == old_sum["mv_trip_count"] and new_sum["trip_count"] < old_sum["trip_count"]:
                    tie_trip += 1

            out_rows.append(
                {
                    "job_id": jid,
                    "instance": (row.get("instance") or "").strip(),
                    "A": (row.get("A") or "").strip(),
                    "L": (row.get("L") or "").strip(),
                    "old_best_multi_fitness": "" if old_fit == math.inf else f"{old_fit:.12f}".rstrip("0").rstrip("."),
                    "new_best_multi_fitness": "" if new_fit == math.inf else f"{new_fit:.12f}".rstrip("0").rstrip("."),
                    "delta_best_multi_fitness": ""
                    if old_fit == math.inf or new_fit == math.inf
                    else f"{(new_fit-old_fit):.12f}".rstrip("0").rstrip("."),
                    "old_trip_count": str(old_sum["trip_count"]),
                    "new_trip_count": str(new_sum["trip_count"]),
                    "old_leg_count": str(old_sum["leg_count"]),
                    "new_leg_count": str(new_sum["leg_count"]),
                    "old_mv_trip_count": str(old_sum["mv_trip_count"]),
                    "new_mv_trip_count": str(new_sum["mv_trip_count"]),
                    "old_dup_pkg_cnt": str(old_sum["dup_pkg_cnt"]),
                    "new_dup_pkg_cnt": str(new_sum["dup_pkg_cnt"]),
                    "old_dup_pkgs": str(old_sum["dup_pkgs"]),
                    "new_dup_pkgs": str(new_sum["dup_pkgs"]),
                    "old_mv_trip_details": str(old_sum["mv_trip_details"]),
                    "new_mv_trip_details": str(new_sum["mv_trip_details"]),
                }
            )

    out_csv = pathlib.Path("multivisit_compare_report.csv")
    with out_csv.open("w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=list(out_rows[0].keys()) if out_rows else [],
        )
        w.writeheader()
        w.writerows(out_rows)

    print("Wrote:", out_csv)
    print("Rows:", len(out_rows))
    print("best_multi_fitness improved:", improved)
    print("makespan tie but mv_trip_count improved:", tie_mv)
    print("makespan tie and mv tie but trip_count improved:", tie_trip)
    print("new multi solutions (was empty):", new_multi)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

