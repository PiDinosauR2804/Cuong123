import argparse
import glob
import os
import random
from typing import List, Tuple


def parse_instance(file_path: str):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f]

    header_lines = lines[:8]
    data_rows = []
    for line in lines[8:]:
        if not line.strip():
            continue
        x, y, demand, release = line.split()[:4]
        data_rows.append([int(x), int(y), int(demand), int(release)])
    return header_lines, data_rows


def write_instance(file_path: str, header_lines: List[str], rows: List[List[int]]):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        for line in header_lines[:7]:
            f.write(line + "\n")
        f.write("XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE\n")
        for row in rows:
            f.write(f"{row[0]}\t{row[1]}\t{row[2]}\t{row[3]}\n")


def build_batch_sizes(n_customers: int, min_batch: int, max_batch: int, rng: random.Random) -> List[int]:
    min_batches = (n_customers + max_batch - 1) // max_batch
    max_batches = n_customers // min_batch
    if min_batches > max_batches:
        raise ValueError("Cannot split customers into batches with given min/max size")

    # Pick a feasible batch count, then distribute remaining customers.
    batch_count = rng.randint(min_batches, max_batches)
    sizes = [min_batch] * batch_count
    remain = n_customers - min_batch * batch_count

    while remain > 0:
        i = rng.randrange(batch_count)
        if sizes[i] < max_batch:
            sizes[i] += 1
            remain -= 1

    rng.shuffle(sizes)
    return sizes


def _dist2(a: List[int], b: List[int]) -> int:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def build_clustered_batches(customers: List[List[int]], sizes: List[int], depot_xy: Tuple[int, int]) -> List[List[List[int]]]:
    remaining = [c[:] for c in customers]
    depot = [depot_xy[0], depot_xy[1], 0, 0]
    batches: List[List[List[int]]] = []

    for size in sizes:
        if not remaining:
            break

        # Start a batch from the customer closest to depot, then grow by nearest neighbors.
        seed = min(remaining, key=lambda c: _dist2(c, depot))
        batch = [seed]
        remaining.remove(seed)

        while len(batch) < size and remaining:
            nxt = min(remaining, key=lambda c: min(_dist2(c, b) for b in batch))
            batch.append(nxt)
            remaining.remove(nxt)

        batches.append(batch)

    if remaining:
        if not batches:
            batches.append([])
        batches[-1].extend(remaining)

    return batches


def build_random_batches(customers: List[List[int]], sizes: List[int], rng: random.Random) -> List[List[List[int]]]:
    remaining = [c[:] for c in customers]
    batches: List[List[List[int]]] = []

    for size in sizes:
        if not remaining:
            break

        take = min(size, len(remaining))
        selected = rng.sample(remaining, take)
        batch = [c[:] for c in selected]
        for c in selected:
            remaining.remove(c)
        batches.append(batch)

    if remaining:
        if not batches:
            batches.append([])
        batches[-1].extend(remaining)

    return batches


def assign_equal_interval_release_to_batches(
    batches: List[List[List[int]]],
    max_release: int,
    rng: random.Random,
    release_order: str,
):
    batch_count = len(batches)
    if batch_count == 0:
        return

    if batch_count == 1:
        release_values = [max_release]
    else:
        step = max_release / float(batch_count - 1)
        release_values = [int(round(i * step)) for i in range(batch_count)]

    if release_order == "random" and batch_count > 1:
        rng.shuffle(release_values)

    for batch_idx, batch in enumerate(batches):
        rel = release_values[batch_idx]
        for customer in batch:
            customer[3] = rel


def process_all(
    input_dir: str,
    output_dir_cluster: str,
    output_dir_random: str,
    pattern: str,
    min_batch: int,
    max_batch: int,
    seed: int,
    release_order: str,
):
    rng_cluster = random.Random(seed)
    rng_random = random.Random(seed + 1)
    files = sorted(glob.glob(os.path.join(input_dir, pattern)))
    if not files:
        raise FileNotFoundError(f"No files matched pattern '{pattern}' in {input_dir}")

    total_cluster = 0
    total_random = 0
    for src in files:
        header_lines, rows = parse_instance(src)
        if not rows:
            continue

        depot = rows[0][:]
        depot[2] = 0
        depot[3] = 0

        customers_base = [r[:] for r in rows[1:]]
        for row in customers_base:
            row[2] = 1

        current_max_release = max((r[3] for r in customers_base), default=0)

        sizes_cluster = build_batch_sizes(len(customers_base), min_batch, max_batch, rng_cluster)
        cluster_batches = build_clustered_batches(customers_base, sizes_cluster, (depot[0], depot[1]))
        assign_equal_interval_release_to_batches(cluster_batches, current_max_release, rng_cluster, release_order)
        customers_cluster = [c for batch in cluster_batches for c in batch]
        out_rows_cluster = [depot] + customers_cluster
        dst_cluster = os.path.join(output_dir_cluster, os.path.basename(src))
        write_instance(dst_cluster, header_lines, out_rows_cluster)
        total_cluster += 1

        sizes_random = build_batch_sizes(len(customers_base), min_batch, max_batch, rng_random)
        random_batches = build_random_batches(customers_base, sizes_random, rng_random)
        assign_equal_interval_release_to_batches(random_batches, current_max_release, rng_random, release_order)
        customers_random = [c for batch in random_batches for c in batch]
        out_rows_random = [depot] + customers_random
        dst_random = os.path.join(output_dir_random, os.path.basename(src))
        write_instance(dst_random, header_lines, out_rows_random)
        total_random += 1

    print(f"Generated {total_cluster} instances into: {output_dir_cluster}")
    print(f"Generated {total_random} instances into: {output_dir_random}")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a new dataset from test_data/data_demand_random/50 where all customer demands are 1, "
            "and release dates are assigned by equal-interval batches (4-8 customers per batch)."
        )
    )
    parser.add_argument(
        "--input-dir",
        default=os.path.join("test_data", "data_demand_random", "50"),
        help="Source folder containing base instances.",
    )
    parser.add_argument(
        "--output-dir-cluster",
        default=os.path.join("test_data", "data_demand_random_50_batch_all1_equal_cluster"),
        help="Destination folder for clustered-release instances.",
    )
    parser.add_argument(
        "--output-dir-random",
        default=os.path.join("test_data", "data_demand_random_50_batch_all1_equal_random"),
        help="Destination folder for random-position-release instances.",
    )
    parser.add_argument("--pattern", default="*.dat", help="Glob pattern for source files.")
    parser.add_argument("--min-batch-size", type=int, default=4, help="Minimum customers per batch.")
    parser.add_argument("--max-batch-size", type=int, default=8, help="Maximum customers per batch.")
    parser.add_argument("--seed", type=int, default=20260312)
    parser.add_argument(
        "--release-order",
        choices=["random", "sequential"],
        default="random",
        help=(
            "How equal-interval release values are mapped to batches. "
            "'random' (default) removes near-depot -> early-release bias; "
            "'sequential' keeps original behavior."
        ),
    )
    args = parser.parse_args()

    if args.min_batch_size < 1:
        raise ValueError("min-batch-size must be >= 1")
    if args.max_batch_size < args.min_batch_size:
        raise ValueError("max-batch-size must be >= min-batch-size")

    process_all(
        input_dir=args.input_dir,
        output_dir_cluster=args.output_dir_cluster,
        output_dir_random=args.output_dir_random,
        pattern=args.pattern,
        min_batch=args.min_batch_size,
        max_batch=args.max_batch_size,
        seed=args.seed,
        release_order=args.release_order,
    )


if __name__ == "__main__":
    main()
