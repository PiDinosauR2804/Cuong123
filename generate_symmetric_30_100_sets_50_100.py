import math
import random
from pathlib import Path

ROOT = Path("test_data/special_data/symmetric_patch_multivisit_30_100")
COUNTS = [50, 100]
FILES_PER_COUNT = 5

HEADER = [
    "number_truck\t1",
    "number_drone\t2",
    "truck_speed\t0.5",
    "drone_speed\t1",
    "M_d\t4",
    "L_d\t90",
    "Sigma\t5",
    "XCOORD\tYCOORD\tDEMAND\tRELEASE_DATE",
    "0\t0\t0\t0",
]


def build_release_by_pair(pair_count: int) -> list[int]:
    # Keep exactly 4 farthest pairs at release 0 so both drones are fully used early.
    zero_pairs = 4
    positive_pairs = pair_count - zero_pairs
    rel = []
    for i in range(pair_count):
        if i >= positive_pairs:
            rel.append(0)
        else:
            # Monotone non-increasing by distance (nearer has larger release).
            val = round(20 + (200 - 20) * (positive_pairs - 1 - i) / max(1, positive_pairs - 1))
            rel.append(int(val))
    return rel


def build_points(pair_count: int, seed: int) -> list[tuple[int, int, int]]:
    rng = random.Random(seed)
    points = []
    used = set()

    for i in range(pair_count):
        # Distance from 30..100, slightly perturbed for variety but still ordered by i.
        base_d = 30 + (70 * i / max(1, pair_count - 1))
        d = max(30, min(100, base_d + rng.uniform(-1.2, 1.2)))

        # Angle on positive-x side to preserve Ox symmetry with +/- y points.
        base_a = 6 + (32 * i / max(1, pair_count - 1))
        a = max(5.0, min(42.0, base_a + rng.uniform(-1.0, 1.0)))

        x = int(round(d * math.cos(math.radians(a))))
        y = int(round(d * math.sin(math.radians(a))))

        # Ensure valid coordinates and uniqueness.
        x = max(25, x)
        y = max(2, y)
        while (x, y) in used:
            x += 1
            y = max(2, y - 1)
        used.add((x, y))

        # True Euclidean distance from integer coordinates.
        d_int = math.sqrt(x * x + y * y)
        if d_int < 30:
            x += 1
        if d_int > 100:
            x -= 1

        points.append((x, y, 1))

    # Final sort by true distance (near -> far) to align release monotonicity.
    points.sort(key=lambda p: math.sqrt(p[0] * p[0] + p[1] * p[1]))
    return points


def write_dataset(path: Path, pair_count: int, seed: int) -> None:
    points = build_points(pair_count, seed)
    release = build_release_by_pair(pair_count)

    lines = list(HEADER)
    for i, (x, y, demand) in enumerate(points):
        r = release[i]
        lines.append(f"{x}\t{y}\t{demand}\t{r}")
        lines.append(f"{x}\t{-y}\t{demand}\t{r}")

    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def main() -> None:
    for n in COUNTS:
        pair_count = n // 2
        out_dir = ROOT / str(n)
        out_dir.mkdir(parents=True, exist_ok=True)
        for idx in range(1, FILES_PER_COUNT + 1):
            out = out_dir / f"SYM{n}_patch_mv_30_100_{idx}.dat"
            write_dataset(out, pair_count, seed=1000 * n + idx)
            print(f"written: {out}")


if __name__ == "__main__":
    main()
