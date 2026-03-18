import argparse
import glob
import math
import os
import random
from typing import Dict, List, Tuple


SIZE_LIST = [10, 15, 20, 30]
K_PATCH_MAP = {10: 3, 15: 4, 20: 5, 30: 6}
DELTA_MAP = {10: 20, 15: 20, 20: 18, 30: 16}
JITTER_MAP = {10: 8, 15: 8, 20: 8, 30: 8}


def parse_instance(file_path: str):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f]

    header_lines = lines[:8]
    table_header = lines[7]
    data_rows = []
    for line in lines[8:]:
        if not line.strip():
            continue
        parts = line.split()
        x = int(parts[0])
        y = int(parts[1])
        demand = int(parts[2])
        release = int(parts[3])
        data_rows.append([x, y, demand, release])

    return header_lines, table_header, data_rows


def set_header_value(header_lines: List[str], key: str, value: str) -> List[str]:
    out = []
    for line in header_lines:
        if line.startswith(key + "\t"):
            out.append(f"{key}\t{value}")
        else:
            out.append(line)
    return out


def dist2(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def choose_initial_centroids(points: List[Tuple[float, float]], k: int) -> List[Tuple[float, float]]:
    centroids = [points[0]]
    while len(centroids) < k:
        farthest = None
        farthest_d = -1.0
        for p in points:
            d = min(dist2(p, c) for c in centroids)
            if d > farthest_d:
                farthest_d = d
                farthest = p
        centroids.append(farthest)
    return centroids


def kmeans(points: List[Tuple[float, float]], k: int, max_iter: int = 20):
    k = max(1, min(k, len(points)))
    centroids = choose_initial_centroids(points, k)
    assignment = [0] * len(points)

    for _ in range(max_iter):
        changed = False

        for i, p in enumerate(points):
            best_idx = 0
            best_d = dist2(p, centroids[0])
            for c_idx in range(1, k):
                d = dist2(p, centroids[c_idx])
                if d < best_d:
                    best_d = d
                    best_idx = c_idx
            if assignment[i] != best_idx:
                assignment[i] = best_idx
                changed = True

        cluster_points = [[] for _ in range(k)]
        for i, c_idx in enumerate(assignment):
            cluster_points[c_idx].append(points[i])

        new_centroids = []
        for c_idx in range(k):
            if cluster_points[c_idx]:
                sx = sum(p[0] for p in cluster_points[c_idx])
                sy = sum(p[1] for p in cluster_points[c_idx])
                n = len(cluster_points[c_idx])
                new_centroids.append((sx / n, sy / n))
            else:
                new_centroids.append(centroids[c_idx])

        centroids = new_centroids
        if not changed:
            break

    return assignment, centroids


def generate_patch_release(
    data_rows: List[List[int]],
    size: int,
    rng: random.Random,
) -> List[List[int]]:
    # data_rows[0] is depot.
    depot = data_rows[0]
    customers = data_rows[1:]

    points = [(row[0], row[1]) for row in customers]
    k = K_PATCH_MAP.get(size, max(3, int(math.sqrt(size))))
    delta = DELTA_MAP.get(size, 20)
    jitter = JITTER_MAP.get(size, 8)

    assignment, centroids = kmeans(points, k)

    depot_xy = (depot[0], depot[1])
    cluster_ids = list(range(k))
    cluster_ids.sort(key=lambda c: dist2(centroids[c], depot_xy))
    cluster_order = {cid: idx for idx, cid in enumerate(cluster_ids)}

    clusters: Dict[int, List[int]] = {cid: [] for cid in range(k)}
    for idx, cid in enumerate(assignment):
        clusters[cid].append(idx)

    new_rows = [depot[:]]
    new_rows[0][3] = 0

    for cid in cluster_ids:
        base_t = cluster_order[cid] * delta
        member_indices = clusters[cid]
        rng.shuffle(member_indices)

        # A small bridge overlap between adjacent patches encourages chaining visits.
        bridge_count = max(1, len(member_indices) // 6)

        for local_idx, customer_idx in enumerate(member_indices):
            row = customers[customer_idx][:]
            release = base_t + rng.randint(0, jitter)
            if cluster_order[cid] > 0 and local_idx < bridge_count:
                release = max(0, release - delta // 2)
            row[3] = int(release)
            customers[customer_idx] = row

    new_rows.extend(customers)
    return new_rows


def write_instance(file_path: str, header_lines: List[str], rows: List[List[int]]):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        for line in header_lines[:7]:
            f.write(line + "\n")
        f.write("XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE\n")
        for row in rows:
            f.write(f"{row[0]}\t{row[1]}\t{row[2]}\t{row[3]}\n")


def process_all(
    input_root: str,
    output_root: str,
    pattern: str,
    seed: int,
    number_truck: int,
    number_drone: int,
    drone_capacity: int,
):
    rng = random.Random(seed)
    total = 0

    for size in SIZE_LIST:
        in_dir = os.path.join(input_root, str(size))
        out_dir = os.path.join(output_root, str(size))
        files = sorted(glob.glob(os.path.join(in_dir, pattern)))

        for src in files:
            header_lines, _, data_rows = parse_instance(src)

            header_lines = set_header_value(header_lines, "number_truck", str(number_truck))
            header_lines = set_header_value(header_lines, "number_drone", str(number_drone))
            header_lines = set_header_value(header_lines, "M_d", str(drone_capacity))

            new_rows = generate_patch_release(data_rows, size=size, rng=rng)

            dst = os.path.join(out_dir, os.path.basename(src))
            write_instance(dst, header_lines, new_rows)
            total += 1

    print(f"Generated {total} instances into: {output_root}")


def main():
    parser = argparse.ArgumentParser(description="Generate patch-style release-date datasets for multi-visit behavior.")
    parser.add_argument(
        "--input-root",
        default=os.path.join("test_data", "data_demand_random"),
        help="Input root directory containing size folders (10/15/20/30).",
    )
    parser.add_argument(
        "--output-root",
        default=os.path.join("test_data", "data_demand_random_patch"),
        help="Output root directory for generated datasets.",
    )
    parser.add_argument(
        "--pattern",
        default="C201*.dat",
        help="Filename glob pattern applied inside each size folder.",
    )
    parser.add_argument("--seed", type=int, default=20260310)
    parser.add_argument("--number-truck", type=int, default=2)
    parser.add_argument("--number-drone", type=int, default=1)
    parser.add_argument("--drone-capacity", type=int, default=8)
    args = parser.parse_args()

    process_all(
        input_root=args.input_root,
        output_root=args.output_root,
        pattern=args.pattern,
        seed=args.seed,
        number_truck=args.number_truck,
        number_drone=args.number_drone,
        drone_capacity=args.drone_capacity,
    )


if __name__ == "__main__":
    main()
