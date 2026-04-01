#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, List, Tuple


def read_dat(path: Path) -> Tuple[List[str], List[List[str]]]:
    lines = [ln.rstrip("\n") for ln in path.read_text().splitlines() if ln.strip()]
    h = lines.index("XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE")
    header = lines[: h + 1]
    rows = [ln.split("\t") for ln in lines[h + 1 :]]
    return header, rows


def write_dat(path: Path, header: List[str], rows: List[List[str]]) -> None:
    out = header + ["\t".join(r) for r in rows]
    path.write_text("\n".join(out) + "\n")


def euclid(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest_group(cands: List[int], coords: List[Tuple[float, float]], anchor: int, k: int) -> List[int]:
    if not cands or k <= 0:
        return []
    # seed near anchor, then greedily add nearest to current selected centroid
    seed = min(cands, key=lambda c: euclid(coords[c], coords[anchor]))
    selected = [seed]
    remain = [c for c in cands if c != seed]
    while remain and len(selected) < k:
        cx = sum(coords[c][0] for c in selected) / len(selected)
        cy = sum(coords[c][1] for c in selected) / len(selected)
        nxt = min(remain, key=lambda c: euclid(coords[c], (cx, cy)))
        selected.append(nxt)
        remain.remove(nxt)
    return selected


def process_pair(dat_path: Path, sol_path: Path, out_dat: Path, out_sol: Path) -> Dict[str, int]:
    header, rows = read_dat(dat_path)
    sol = json.loads(sol_path.read_text())
    trucks, drone_q = sol
    # read M_d/L_d from header to validate with same config as file
    md = ld = None
    for ln in header:
        tok = ln.split()
        if not tok:
            continue
        if tok[0] == "M_d":
            md = tok[-1]
        elif tok[0] == "L_d":
            ld = tok[-1]
    if md is None or ld is None:
        raise ValueError(f"Missing M_d/L_d in {dat_path}")

    def validate_current(_rows: List[List[str]], _drone_q: List[List[List[int]]]) -> bool:
        # Validate seed feasibility using read_data_f6 validator.
        with tempfile.NamedTemporaryFile("w", suffix=".dat", delete=False) as tf_dat:
            tmp_dat = tf_dat.name
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as tf_sol:
            tmp_sol = tf_sol.name
        try:
            write_dat(Path(tmp_dat), header, _rows)
            Path(tmp_sol).write_text(json.dumps([trucks, _drone_q], ensure_ascii=True))
            env = os.environ.copy()
            env["VALIDATE_ONLY"] = "1"
            p = subprocess.run(
                ["C_Version/read_data_f6", tmp_dat, str(md), str(ld), tmp_sol],
                env=env,
                capture_output=True,
                text=True,
            )
            out = (p.stdout or "") + (p.stderr or "")
            return "[VALIDATE] OK" in out
        finally:
            try:
                Path(tmp_dat).unlink(missing_ok=True)
            except Exception:
                pass
            try:
                Path(tmp_sol).unlink(missing_ok=True)
            except Exception:
                pass

    coords: List[Tuple[float, float]] = []
    rel: List[int] = []
    for r in rows:
        coords.append((float(r[0]), float(r[1])))
        rel.append(int(r[3]))

    # truck route position map
    route_pos: List[Dict[int, int]] = []
    for tr in trucks:
        seq = [node for node, _ in tr]
        route_pos.append({c: i for i, c in enumerate(seq)})

    changed_points = set()
    per_trip_aug = []

    for trip in drone_q:
        trip_pkg = []
        for _, pk in trip:
            trip_pkg.extend(pk)
        base_count = len(trip_pkg)
        headroom = max(0, 8 - base_count)
        if headroom == 0:
            per_trip_aug.append(0)
            continue

        # reference release of this trip from current resup packages
        trip_rel = max((rel[p] for p in trip_pkg), default=1)
        picked: List[int] = []
        picked_by_event: Dict[int, List[int]] = {}

        # collect downstream candidates from each event
        for ev_idx, (rv, _pk) in enumerate(trip):
            # find owning truck for rv
            owner = None
            for t, mp in enumerate(route_pos):
                if rv in mp:
                    owner = t
                    break
            if owner is None:
                continue

            tr_seq = [node for node, _ in trucks[owner]]
            p0 = route_pos[owner][rv]
            downstream = tr_seq[p0 + 1 :]

            # only points currently release=1 and not already in trip or selected
            cands = [
                c
                for c in downstream
                if c > 0 and rel[c] == 1 and c not in trip_pkg and c not in picked
            ]
            if not cands:
                continue
            need = headroom - len(picked)
            if need <= 0:
                break
            group = nearest_group(cands, coords, rv, need)
            picked.extend(group)
            if group:
                picked_by_event.setdefault(ev_idx, []).extend(group)
            if len(picked) >= headroom:
                break

        # apply release update rule
        if not picked:
            per_trip_aug.append(0)
            continue
        if len(picked) == 1:
            new_rel = trip_rel
        else:
            new_rel = max(1, trip_rel - 10)

        # apply tentative update
        old_rel = {c: rel[c] for c in picked}
        old_pkg_lists = {ev_idx: list(trip[ev_idx][1]) for ev_idx in picked_by_event}
        for c in picked:
            rel[c] = new_rel
        # update solution packages for those events (tentative)
        for ev_idx, pts in picked_by_event.items():
            # append only new points, keep deterministic order
            current = trip[ev_idx][1]
            seen = set(current)
            for c in pts:
                if c not in seen:
                    current.append(c)
                    seen.add(c)
        # validate; if invalid, rollback this trip changes
        trial_rows = [r[:] for r in rows]
        for i in range(len(trial_rows)):
            trial_rows[i][3] = str(rel[i])
        if validate_current(trial_rows, drone_q):
            for c in picked:
                changed_points.add(c)
            per_trip_aug.append(len(picked))
        else:
            for c, rr in old_rel.items():
                rel[c] = rr
            for ev_idx, lst in old_pkg_lists.items():
                trip[ev_idx][1] = lst
            per_trip_aug.append(0)

    # write back
    for i in range(len(rows)):
        rows[i][3] = str(rel[i])
    write_dat(out_dat, header, rows)
    out_sol.write_text(json.dumps([trucks, drone_q], ensure_ascii=True))

    return {
        "trips": len(drone_q),
        "augmented_points": len(changed_points),
        "trip_aug_sum": sum(per_trip_aug),
        "max_release": max(rel[1:]) if len(rel) > 1 else 0,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-dir", default="result/generated_instances/c101_cluster5_strategies")
    ap.add_argument("--output-dir", default="result/generated_instances/c101_cluster5_strategies_augmented")
    ap.add_argument("--glob", default="*_strategy*.dat")
    args = ap.parse_args()

    in_dir = Path(args.input_dir)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    dat_files = sorted(in_dir.glob(args.glob))
    summary = []
    for dat in dat_files:
        if dat.name.endswith("_solution.json"):
            continue
        sol = dat.with_name(dat.stem + "_solution.json")
        if not sol.exists():
            continue
        out_dat = out_dir / dat.name
        # copy solution as-is
        out_sol = out_dir / sol.name

        stats = process_pair(dat, sol, out_dat, out_sol)
        summary.append(
            {
                "file": dat.name,
                "solution": sol.name,
                **stats,
                "out_dat": str(out_dat),
            }
        )

    (out_dir / "summary_augmented.json").write_text(json.dumps(summary, indent=2, ensure_ascii=True))
    print(f"Processed {len(summary)} files -> {out_dir}")


if __name__ == "__main__":
    main()
