from __future__ import annotations

import argparse
import statistics
import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from main import AtsParams, adaptive_tabu_search


def run_mode(instance: str, nrep: int, use_eval_cache: bool, nimp: int, seg: int, div: int, seed: int) -> None:
    times = []
    fits = []
    for _ in range(nrep):
        params = AtsParams(
            nimp=nimp,
            seg=seg,
            div=div,
            seed=seed,
            use_eval_cache=use_eval_cache,
        )
        t0 = time.perf_counter()
        _, best_fit = adaptive_tabu_search(instance, params)
        t1 = time.perf_counter()
        times.append(t1 - t0)
        fits.append(best_fit)

    mean_t = statistics.mean(times)
    median_t = statistics.median(times)
    print(f"mode={'cache' if use_eval_cache else 'no_cache'}")
    print(f"times_sec={times}")
    print(f"mean_sec={mean_t:.6f}")
    print(f"median_sec={median_t:.6f}")
    print(f"best_fits={fits}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark ATS with and without eval cache")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--nrep", type=int, default=3)
    parser.add_argument("--nimp", type=int, default=5)
    parser.add_argument("--seg", type=int, default=2)
    parser.add_argument("--div", type=int, default=1)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    run_mode(args.instance, args.nrep, use_eval_cache=False, nimp=args.nimp, seg=args.seg, div=args.div, seed=args.seed)
    run_mode(args.instance, args.nrep, use_eval_cache=True, nimp=args.nimp, seg=args.seg, div=args.div, seed=args.seed)


if __name__ == "__main__":
    main()
