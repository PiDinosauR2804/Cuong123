#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple


@dataclass
class Node:
    x: float
    y: float
    demand: int
    release: int


@dataclass
class DatFile:
    header_lines: List[str]
    nodes: List[Node]  # index 0 is depot


def parse_dat(path: Path) -> DatFile:
    raw = path.read_text(encoding="utf-8").splitlines()
    lines = [ln.strip() for ln in raw if ln.strip()]

    start = -1
    for i, ln in enumerate(lines):
        tok = ln.split()
        if tok and tok[0] == "XCOORD":
            start = i + 1
            break
    if start < 0:
        raise ValueError(f"Invalid .dat format (missing XCOORD header): {path}")

    header = lines[:start]
    nodes: List[Node] = []
    for ln in lines[start:]:
        t = ln.split()
        if len(t) < 4:
            continue
        x = float(t[0])
        y = float(t[1])
        d = int(float(t[2]))
        r = int(float(t[3]))
        nodes.append(Node(x=x, y=y, demand=d, release=r))
    if not nodes:
        raise ValueError(f"No node data in {path}")
    return DatFile(header_lines=header, nodes=nodes)


def format_num(v: float) -> str:
    if float(v).is_integer():
        return str(int(v))
    s = f"{v:.10f}".rstrip("0").rstrip(".")
    return s if s else "0"


def write_dat(path: Path, header_lines: Sequence[str], nodes: Sequence[Node]) -> None:
    out: List[str] = list(header_lines)
    for n in nodes:
        out.append(
            f"{format_num(n.x)}\t{format_num(n.y)}\t{int(n.demand)}\t{int(n.release)}"
        )
    path.write_text("\n".join(out) + "\n", encoding="utf-8")


def split_sizes(n_customers: int, b: int) -> List[int]:
    base = n_customers // b
    rem = n_customers % b
    return [base + 1 if i < rem else base for i in range(b)]


def dist2(a: Node, b: Node) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    return dx * dx + dy * dy


def cluster_batches(customer_ids: List[int], nodes: Sequence[Node], sizes: Sequence[int], rng: random.Random) -> List[List[int]]:
    remaining = set(customer_ids)
    groups: List[List[int]] = []
    for sz in sizes:
        if not remaining or sz <= 0:
            groups.append([])
            continue
        seed = rng.choice(list(remaining))
        remaining.remove(seed)

        group = [seed]
        if sz > 1 and remaining:
            ranked = sorted(remaining, key=lambda cid: dist2(nodes[seed], nodes[cid]))
            take = ranked[: max(0, sz - 1)]
            for cid in take:
                remaining.remove(cid)
                group.append(cid)
        groups.append(group)

    if remaining:
        groups[-1].extend(sorted(remaining))
    return groups


def random_batches(customer_ids: List[int], sizes: Sequence[int], rng: random.Random) -> List[List[int]]:
    ids = customer_ids[:]
    rng.shuffle(ids)
    groups: List[List[int]] = []
    idx = 0
    for sz in sizes:
        groups.append(ids[idx : idx + sz])
        idx += sz
    return groups


def assign_release(groups: Sequence[Sequence[int]], max_beta: int, b: int) -> List[Tuple[int, List[int]]]:
    if b <= 1:
        thresholds = [0]
    else:
        step = max_beta // (b - 1)
        thresholds = [i * step for i in range(b)]
        thresholds[-1] = max_beta
    return [(thresholds[i], list(groups[i])) for i in range(len(groups))]


def generate_one(
    src: DatFile,
    beta: float,
    b: int,
    version: int,
    mode: str,
    out_dir: Path,
    base_seed: int,
) -> Path:
    n = len(src.nodes)
    if n < 2:
        raise ValueError("Need at least depot + 1 customer")
    customer_ids = list(range(1, n))
    sizes = split_sizes(len(customer_ids), b)

    r_max1 = max(node.release for node in src.nodes[1:])
    max_beta = int(round(beta * r_max1))
    if max_beta < 0:
        max_beta = 0

    # Keep grouping identical across betas for the same (B, version, mode).
    # Only release thresholds change with beta.
    local_seed = (
        base_seed
        + b * 1_000
        + version * 17
        + (0 if mode == "cluster" else 1)
    )
    rng = random.Random(local_seed)

    if mode == "cluster":
        groups = cluster_batches(customer_ids, src.nodes, sizes, rng)
    elif mode == "random":
        groups = random_batches(customer_ids, sizes, rng)
    else:
        raise ValueError(f"Unsupported mode: {mode}")

    rel_assign = assign_release(groups, max_beta=max_beta, b=b)

    nodes = [Node(x=n0.x, y=n0.y, demand=n0.demand, release=n0.release) for n0 in src.nodes]
    nodes[0].release = 0
    for rel, ids in rel_assign:
        for cid in ids:
            nodes[cid].release = int(rel)

    beta_txt = str(beta).rstrip("0").rstrip(".")
    out_name = f"R101_{beta_txt}_B{b}_v{version}.dat"
    out_path = out_dir / out_name
    write_dat(out_path, src.header_lines, nodes)
    return out_path


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate R101 beta-batch datasets (cluster/random).")
    ap.add_argument(
        "--input",
        default="test_data/data_demand_random/50/R101_1.dat",
        help="Source .dat file used to compute r_max1 and node data.",
    )
    ap.add_argument("--betas", nargs="+", type=float, default=[0.5, 1.5, 3.0])
    ap.add_argument("--batches", nargs="+", type=int, default=[3, 8])
    ap.add_argument("--versions", type=int, default=5, help="Number of variants per (beta, B), starting from v0.")
    ap.add_argument("--seed", type=int, default=20260406)
    ap.add_argument(
        "--out-cluster",
        default="test_data/data_demand_random_50_batch_custom_cluster_all_flat",
    )
    ap.add_argument(
        "--out-random",
        default="test_data/data_demand_random_50_batch_custom_random_all_flat",
    )
    args = ap.parse_args()

    src_path = Path(args.input)
    src = parse_dat(src_path)
    out_cluster = Path(args.out_cluster)
    out_random = Path(args.out_random)
    out_cluster.mkdir(parents=True, exist_ok=True)
    out_random.mkdir(parents=True, exist_ok=True)

    r_max1 = max(node.release for node in src.nodes[1:])
    print(f"[INFO] source={src_path}")
    print(f"[INFO] r_max1={r_max1}")

    count_cluster = 0
    count_random = 0
    for beta in args.betas:
        max_beta = int(round(beta * r_max1))
        print(f"[INFO] beta={beta} -> max_beta={max_beta}")
        for b in args.batches:
            if b <= 1:
                raise ValueError("Batch count B must be >= 2")
            for v in range(args.versions):
                generate_one(src, beta, b, v, "cluster", out_cluster, args.seed)
                generate_one(src, beta, b, v, "random", out_random, args.seed)
                count_cluster += 1
                count_random += 1

    print(f"[DONE] cluster files: {count_cluster} -> {out_cluster}")
    print(f"[DONE] random  files: {count_random} -> {out_random}")


if __name__ == "__main__":
    main()
