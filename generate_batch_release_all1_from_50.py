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


def set_header_value(header_lines: List[str], key: str, value: str) -> List[str]:
    out = []
    for line in header_lines:
        if line.startswith(key + "\t"):
            out.append(f"{key}\t{value}")
        else:
            out.append(line)
    return out


def write_instance(file_path: str, header_lines: List[str], rows: List[List[int]]):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        for line in header_lines[:7]:
            f.write(line + "\n")
        f.write("XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE\n")
        for row in rows:
            f.write(f"{row[0]}\t{row[1]}\t{row[2]}\t{row[3]}\n")


def sort_customers_for_batch(customers: List[List[int]], depot_xy: Tuple[int, int]):
    # Sort by radial distance and angle to make local points share similar release windows.
    def key_fn(row):
        dx = row[0] - depot_xy[0]
        dy = row[1] - depot_xy[1]
        return (dx * dx + dy * dy, dy, dx)

    return sorted(customers, key=key_fn)


def assign_release_by_batch(
    customers: List[List[int]],
    min_batch_size: int,
    max_batch_size: int,
    delta_t: int,
    jitter: int,
    rng: random.Random,
):
    n = len(customers)
    start = 0
    batch_id = 0
    while start < n:
        batch_size = rng.randint(min_batch_size, max_batch_size)
        end = min(n, start + batch_size)
        base = batch_id * delta_t
        for idx in range(start, end):
            customers[idx][3] = base + rng.randint(0, jitter)
        start = end
        batch_id += 1


def process_all(
    input_dir: str,
    output_dir: str,
    pattern: str,
    min_batch_size: int,
    max_batch_size: int,
    delta_t: int,
    jitter: int,
    seed: int,
    number_truck: int,
    number_drone: int,
    drone_capacity: int,
):
    rng = random.Random(seed)
    files = sorted(glob.glob(os.path.join(input_dir, pattern)))
    if not files:
        raise FileNotFoundError(f"No files matched pattern '{pattern}' in {input_dir}")

    total = 0
    for src in files:
        header_lines, rows = parse_instance(src)
        header_lines = set_header_value(header_lines, "number_truck", str(number_truck))
        header_lines = set_header_value(header_lines, "number_drone", str(number_drone))
        header_lines = set_header_value(header_lines, "M_d", str(drone_capacity))
        if not rows:
            continue

        depot = rows[0]
        depot[2] = 0
        depot[3] = 0

        customers = [row[:] for row in rows[1:]]
        for row in customers:
            row[2] = 1

        customers = sort_customers_for_batch(customers, (depot[0], depot[1]))
        assign_release_by_batch(
            customers,
            min_batch_size=min_batch_size,
            max_batch_size=max_batch_size,
            delta_t=delta_t,
            jitter=jitter,
            rng=rng,
        )

        out_rows = [depot] + customers
        dst = os.path.join(output_dir, os.path.basename(src))
        write_instance(dst, header_lines, out_rows)
        total += 1

    print(f"Generated {total} instances into: {output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate new datasets from test_data/data_demand_random/50 with batch release dates and all customer demands = 1."
    )
    parser.add_argument(
        "--input-dir",
        default=os.path.join("test_data", "data_demand_random", "50"),
        help="Source folder containing base instances (default: test_data/data_demand_random/50).",
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join("test_data", "data_demand_random_50_batch_all1"),
        help="Destination folder for generated instances.",
    )
    parser.add_argument("--pattern", default="*.dat", help="Glob pattern for source files.")
    parser.add_argument("--min-batch-size", type=int, default=4, help="Minimum customers per release batch.")
    parser.add_argument("--max-batch-size", type=int, default=8, help="Maximum customers per release batch.")
    parser.add_argument("--delta-t", type=int, default=20, help="Time gap between consecutive batches.")
    parser.add_argument("--jitter", type=int, default=6, help="Random noise within each batch.")
    parser.add_argument("--seed", type=int, default=20260310)
    parser.add_argument("--number-truck", type=int, default=2)
    parser.add_argument("--number-drone", type=int, default=2)
    parser.add_argument("--drone-capacity", type=int, default=4)
    args = parser.parse_args()

    if args.min_batch_size < 1:
        raise ValueError("min-batch-size must be >= 1")
    if args.max_batch_size < args.min_batch_size:
        raise ValueError("max-batch-size must be >= min-batch-size")

    process_all(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        pattern=args.pattern,
        min_batch_size=args.min_batch_size,
        max_batch_size=args.max_batch_size,
        delta_t=args.delta_t,
        jitter=args.jitter,
        seed=args.seed,
        number_truck=args.number_truck,
        number_drone=args.number_drone,
        drone_capacity=args.drone_capacity,
    )


if __name__ == "__main__":
    main()
