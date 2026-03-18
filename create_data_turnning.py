import re
import random
from typing import Dict, List, Tuple, Any

# ----------------------------
# 1) IO helpers for .dat
# ----------------------------
def read_dat(path: str):
    """
    Returns:
      header_lines: list[str]  (everything up to and including the column header line)
      rows: list[dict]         (index 0 is depot, then customers)
      params: dict             (parsed key-value lines, including M_d as A)
    """
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.rstrip("\n") for ln in f]

    params = {}
    header_lines = []
    data_start = None

    # Parse parameter lines until the column header
    for i, ln in enumerate(lines):
        if ln.strip().startswith("XCOORD"):
            header_lines = lines[: i + 1]
            data_start = i + 1
            break
        if not ln.strip():
            continue
        parts = re.split(r"\s+", ln.strip())
        if len(parts) >= 2:
            key, val = parts[0], parts[1]
            # keep as string then cast later if needed
            params[key] = val

    if data_start is None:
        raise ValueError("Cannot find data table header line starting with 'XCOORD'.")

    rows = []
    for ln in lines[data_start:]:
        if not ln.strip():
            continue
        parts = re.split(r"\s+", ln.strip())
        if len(parts) < 4:
            continue
        x, y, dem, rel = parts[:4]
        rows.append(
            {
                "x": float(x),
                "y": float(y),
                "demand": int(float(dem)),
                "release": int(float(rel)),
            }
        )

    return header_lines, rows, params


def write_dat(path: str, header_lines: List[str], rows: List[Dict[str, Any]]):
    with open(path, "w", encoding="utf-8") as f:
        for ln in header_lines:
            f.write(ln + "\n")
        for r in rows:
            f.write(f"{int(r['x'])}\t{int(r['y'])}\t{int(r['demand'])}\t{int(r['release'])}\n")


# ----------------------------
# 2) Parse your solution format
# ----------------------------
def parse_solution(solution: Any) -> Tuple[List[List[int]], List[Tuple[int, List[int]]]]:
    """
    Expects solution like:
      [
        [ truck0_route_repr, truck1_route_repr, ... ],
        [ drone_trips_repr ],
        ... (optional)
      ]

    In your example, solution[0] is list of trucks, solution[2] seems list of drone trips.

    We will robustly extract:
      truck_paths: list of list of visited node ids (excluding depot 0 repeats)
      drone_trips: list of (rendezvous_node, orders_list) for each trip
    """
    # Heuristic: trucks are the big nested list with many nodes, drones are list of trips like [[i,[...]]]
    # Your printed structure:
    # [
    #   [ [..truck0..], [..truck1..] ],
    #   [ ... maybe something ... ],
    #   [ [[5,[5,6]]], [[14,[14]]], [[11,[11]]] ]
    # ]
    trucks_repr = solution[0]
    # Find drone trips block (scan for element that looks like list of trips where trip is [[node,[orders]]...])
    drone_block = None
    for blk in solution:
        if isinstance(blk, list) and blk and all(isinstance(t, list) for t in blk):
            # candidate: list of trips
            ok = True
            for trip in blk:
                # trip should be list with 1+ rendezvous entries, each entry like [node, [orders...]]
                if not (isinstance(trip, list) and trip):
                    ok = False
                    break
                entry = trip[0]
                if not (isinstance(entry, list) and len(entry) == 2 and isinstance(entry[0], int) and isinstance(entry[1], list)):
                    ok = False
                    break
            if ok:
                drone_block = blk
    if drone_block is None:
        raise ValueError("Cannot locate drone trips block in the provided solution structure.")

    # Extract truck visited nodes
    truck_paths: List[List[int]] = []
    for tr in trucks_repr:
        visited = []
        for item in tr:
            if isinstance(item, list) and len(item) == 2 and isinstance(item[0], int):
                node = item[0]
                if node != 0:
                    visited.append(node)
        # remove duplicates while preserving order
        seen = set()
        path = []
        for v in visited:
            if v not in seen:
                seen.add(v)
                path.append(v)
        truck_paths.append(path)

    # Extract drone trips as (rendezvous_node, orders)
    drone_trips: List[Tuple[int, List[int]]] = []
    for trip in drone_block:
        # trip is list of rendezvous entries
        # we use the first entry as rendezvous node; and merge all orders in the trip
        rv_nodes = []
        orders = []
        for entry in trip:
            if isinstance(entry, list) and len(entry) == 2:
                rv_nodes.append(entry[0])
                orders.extend(entry[1])
        # Keep the rendezvous list if you want; but for tuning we mainly use first rv
        rendezvous_node = rv_nodes[0] if rv_nodes else None
        if rendezvous_node is None:
            continue
        # unique orders
        orders_u = []
        seen = set()
        for o in orders:
            if o not in seen:
                seen.add(o)
                orders_u.append(o)
        drone_trips.append((rendezvous_node, orders_u))

    return truck_paths, drone_trips


def build_customer_to_truck_map(truck_paths: List[List[int]]) -> Dict[int, int]:
    cust2truck = {}
    for k, path in enumerate(truck_paths):
        for node in path:
            cust2truck[node] = k
    return cust2truck


# ----------------------------
# 3) Auto-tune logic
# ----------------------------
def auto_tune_release_demand(
    dat_in: str,
    dat_out: str,
    solution: Any,
    seed: int = 42,
    early_release_window: Tuple[int, int] = (0, 5),
    mid_release_window: Tuple[int, int] = (20, 30),
    overlap_slack: int = 2,
    demand_choices: Tuple[int, int] = (1, 2),
    fill_to_capacity: bool = True,
):
    """
    - Picks two drone trips that belong to different trucks and forces their orders to have overlapping release times.
    - Adjusts demands of orders in those two trips so their total is near A (= M_d).
    """
    random.seed(seed)

    header, rows, params = read_dat(dat_in)
    # A is drone capacity, stored as M_d in your file
    A = int(float(params.get("M_d", "4")))

    truck_paths, drone_trips = parse_solution(solution)
    cust2truck = build_customer_to_truck_map(truck_paths)

    # Identify which drone trip serves which truck (based on rendezvous node)
    trip_infos = []
    for (rv, orders) in drone_trips:
        truck_id = cust2truck.get(rv, None)
        if truck_id is None:
            continue
        trip_infos.append({"rv": rv, "orders": orders, "truck": truck_id})

    if len(trip_infos) < 2:
        raise ValueError("Need at least 2 drone trips to auto-tune for multi-visit.")

    # Choose 2 trips from different trucks if possible
    pair = None
    for i in range(len(trip_infos)):
        for j in range(i + 1, len(trip_infos)):
            if trip_infos[i]["truck"] != trip_infos[j]["truck"]:
                pair = (trip_infos[i], trip_infos[j])
                break
        if pair:
            break

    # If all trips are on same truck, pick any 2 (still can force multi-rendezvous within same truck)
    if pair is None:
        pair = (trip_infos[0], trip_infos[1])

    t1, t2 = pair
    focus_orders = set(t1["orders"]) | set(t2["orders"])
    focus_rv = (t1["rv"], t2["rv"])
    focus_trucks = (t1["truck"], t2["truck"])

    # --- Release date tuning ---
    # Strategy:
    # 1) Set most non-focus customers early (truck departs early).
    # 2) Set focus orders (and optionally their rendezvous nodes) into the same mid window, very tight overlap.
    base = random.randint(mid_release_window[0], mid_release_window[1])
    # create two very-close timestamps
    base2 = base + random.randint(-overlap_slack, overlap_slack)

    n_rows = len(rows)  # includes depot at index 0
    # depot is rows[0]
    for idx in range(1, n_rows):
        cust_id = idx  # node id matches line order: 1..n
        if cust_id in focus_orders:
            # force overlap
            rows[idx]["release"] = base if random.random() < 0.5 else base2
        else:
            # push early so trucks can start
            rows[idx]["release"] = random.randint(*early_release_window)

    # Also: make sure at least one "early" customer exists on each truck (helps trucks depart early).
    for k, path in enumerate(truck_paths):
        if not path:
            continue
        anchor = path[0]
        rows[anchor]["release"] = random.randint(*early_release_window)

    # --- Demand tuning ---
    # Baseline: keep demands small
    for idx in range(1, n_rows):
        rows[idx]["demand"] = demand_choices[0] if random.random() < 0.7 else demand_choices[1]

    # Focus orders: adjust to (near) fill capacity A
    # Goal: sum_demands(focus_orders) ≈ A (or slightly below), to encourage combining.
    focus_list = sorted(list(focus_orders))
    if focus_list:
        if fill_to_capacity:
            # Start with all ones, then add +1 until near capacity
            for cid in focus_list:
                rows[cid]["demand"] = 1
            remaining = A - len(focus_list)
            # distribute extra demand (each can be at most 2 here)
            i = 0
            while remaining > 0 and i < len(focus_list):
                cid = focus_list[i]
                if rows[cid]["demand"] < 2:
                    rows[cid]["demand"] += 1
                    remaining -= 1
                i += 1
            # If still remaining > 0 but we limited to {1,2}, that's okay—still “as full as possible”.
        else:
            # Random {1,2} but keep sum <= A
            total = 0
            for cid in focus_list:
                val = 2 if random.random() < 0.5 else 1
                if total + val > A:
                    val = 1
                rows[cid]["demand"] = val
                total += val

    # Write output
    write_dat(dat_out, header, rows)

    # Print a concise report
    total_focus_demand = sum(rows[cid]["demand"] for cid in focus_list) if focus_list else 0
    releases_focus = {cid: rows[cid]["release"] for cid in focus_list}
    print("=== Auto-tune report ===")
    print(f"Input : {dat_in}")
    print(f"Output: {dat_out}")
    print(f"Drone capacity A (M_d) = {A}")
    print(f"Chosen trips: RV nodes {focus_rv} on trucks {focus_trucks}")
    print(f"Focus orders: {focus_list}")
    print(f"Focus total demand: {total_focus_demand} (target near {A})")
    print(f"Focus releases (tight overlap): {releases_focus}")


# ----------------------------
# 4) Example usage with YOUR solution
# ----------------------------
if __name__ == "__main__":
    # Paste your solution here EXACTLY as python literal (list of lists)
    solution = [
        [
            [[0, [2, 3]], [5, [5, 6]], [2, []], [3, []], [6, []], [11, [11]]],
            [[0, [4, 12, 15, 7, 1, 9, 13, 8, 10]], [4, []], [14, [14]], [12, []], [15, []], [7, []], [1, []], [9, []], [13, []], [8, []], [10, []]]
        ],
        # (the middle block in your printout may exist; keep it if you want)
        [
            [[5, [5, 6]]],
            [[14, [14]]],
            [[11, [11]]]
        ]
    ]

    auto_tune_release_demand(
        dat_in=r"test_data\data_demand_random\15\C101_0.5.dat",
        dat_out="test_data\data_new/C101_0.5_MR15_autotuned.dat",
        solution=solution,
        seed=42,
        early_release_window=(0, 5),
        mid_release_window=(20, 30),
        overlap_slack=2,
        demand_choices=(1, 2),
        fill_to_capacity=True,
    )
