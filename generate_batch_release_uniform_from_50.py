import argparse
import glob
import os
import random
from typing import List, Tuple


def parse_instance(file_path: str) -> Tuple[List[str], List[List[int]]]:
    with open(file_path, "r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f]

    header_lines = lines[:8]
    data_rows: List[List[int]] = []
    for line in lines[8:]:
        if not line.strip():
            continue
        x, y, demand, release = line.split()[:4]
        data_rows.append([int(x), int(y), int(demand), int(release)])
    return header_lines, data_rows


def write_instance(file_path: str, header_lines: List[str], rows: List[List[int]]) -> None:
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        for line in header_lines[:7]:
            f.write(line + "\n")
        f.write("XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE\n")
        for row in rows:
            f.write(f"{row[0]}\t{row[1]}\t{row[2]}\t{row[3]}\n")


def generate_uniform_release_dataset(
    input_dir: str,
    output_root: str,
    pattern: str,
    num_versions: int,
    start_version: int,
    base_seed: int,
) -> None:
    files = sorted(glob.glob(os.path.join(input_dir, pattern)))
    if not files:
        raise FileNotFoundError(f"No files matched pattern '{pattern}' in {input_dir}")

    total_written = 0
    for offset in range(num_versions):
        version = start_version + offset
        rng = random.Random(base_seed + offset)
        out_dir = os.path.join(output_root, f"v{version}")

        written_this_version = 0
        for src in files:
            header_lines, rows = parse_instance(src)
            if not rows:
                continue

            depot = rows[0][:]
            depot[2] = 0
            depot[3] = 0

            customers = [r[:] for r in rows[1:]]
            for row in customers:
                row[2] = 1

            max_release = max((r[3] for r in customers), default=0)
            for row in customers:
                if max_release > 0:
                    row[3] = rng.randint(0, max_release)
                else:
                    row[3] = 0

            out_rows = [depot] + customers
            dst = os.path.join(out_dir, os.path.basename(src))
            write_instance(dst, header_lines, out_rows)
            written_this_version += 1

        total_written += written_this_version
        print(
            f"[uniform] v{version}: generated {written_this_version} instances into {out_dir} "
            f"(seed={base_seed + offset})"
        )

    print(f"Done. Total generated instances: {total_written}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Generate multiple datasets from test_data/data_demand_random/50 with all customer demand = 1, "
            "and release dates sampled independently from a uniform distribution U(0, max_release_of_instance)."
        )
    )
    parser.add_argument(
        "--input-dir",
        default=os.path.join("test_data", "data_demand_random", "50"),
        help="Source folder containing base instances.",
    )
    parser.add_argument(
        "--output-root",
        default=os.path.join("test_data", "data_demand_random_50_batch_all1_equal_uniform"),
        help="Root output folder. Data will be written to subfolders v1..vN.",
    )
    parser.add_argument("--pattern", default="*.dat", help="Glob pattern for source files.")
    parser.add_argument("--num-versions", type=int, default=9, help="Number of dataset versions to generate.")
    parser.add_argument(
        "--start-version",
        type=int,
        default=1,
        help="Starting version index (e.g. 1 -> v1).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=20260410,
        help="Base seed. Version i uses seed + (i-start_version).",
    )
    args = parser.parse_args()

    if args.num_versions < 1:
        raise ValueError("num-versions must be >= 1")
    if args.start_version < 0:
        raise ValueError("start-version must be >= 0")

    generate_uniform_release_dataset(
        input_dir=args.input_dir,
        output_root=args.output_root,
        pattern=args.pattern,
        num_versions=args.num_versions,
        start_version=args.start_version,
        base_seed=args.seed,
    )


if __name__ == "__main__":
    main()
