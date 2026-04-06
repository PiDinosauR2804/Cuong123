#!/usr/bin/env python3
import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


INSTANCE_ORDER = ["C101", "C201", "R101", "RC101"]
DIST_ORDER = ["cluster", "random", "uniform"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot 3 heatmaps (cluster/random/uniform) of percent rows with "
            "best_multi_visit_trip_count > 0 by beta and instance set."
        )
    )
    parser.add_argument(
        "--input",
        default="/Users/huetran/Desktop/best_solution2/Analysis/best_all_batch.csv",
        help="Input CSV path",
    )
    parser.add_argument(
        "--output-pdf",
        default="/Users/huetran/Desktop/best_solution2/Analysis/heatmap_beta_instance_by_distribution_multi_visit_corrected.pdf",
        help="Output PDF path",
    )
    parser.add_argument(
        "--output-png",
        default="/Users/huetran/Desktop/best_solution2/Analysis/heatmap_beta_instance_by_distribution_multi_visit_corrected.png",
        help="Output PNG path",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="PNG DPI",
    )
    return parser.parse_args()


def extract_distribution(instance_path: str) -> str:
    s = str(instance_path).lower()
    if "cluster" in s:
        return "cluster"
    if "uniform" in s:
        return "uniform"
    if "random" in s:
        return "random"
    return "unknown"


def extract_instance_set(instance_path: str) -> str:
    name = Path(str(instance_path)).name.upper()
    m = re.match(r"^(RC101|C101|C201|R101)", name)
    if m:
        return m.group(1)
    return "OTHER"


def extract_beta(row: pd.Series) -> float:
    if pd.notna(row.get("beta")):
        try:
            return float(row["beta"])
        except Exception:
            pass
    name = Path(str(row.get("instance", ""))).name
    m = re.search(r"_([0-9]+(?:\.[0-9]+)?)\.dat$", name, flags=re.IGNORECASE)
    if m:
        return float(m.group(1))
    return float("nan")


def beta_label(v: float) -> str:
    if pd.isna(v):
        return "NA"
    if float(v).is_integer():
        return str(int(v))
    return f"{v:g}"


def main() -> None:
    args = parse_args()

    df = pd.read_csv(args.input)
    df = df.copy()

    df["distribution"] = df["instance"].map(extract_distribution)
    df["instance_set"] = df["instance"].map(extract_instance_set)
    df["beta_num"] = df.apply(extract_beta, axis=1)
    # Use visit-trip metric; missing values are treated as 0 so denominator is full group size.
    df["trip"] = pd.to_numeric(df.get("best_multi_visit_trip_count"), errors="coerce").fillna(0)
    df["is_multi_visit"] = (df["trip"] > 0).astype(int)

    work = df[
        df["distribution"].isin(DIST_ORDER)
        & df["instance_set"].isin(INSTANCE_ORDER)
        & df["beta_num"].notna()
    ].copy()

    summary = (
        work.groupby(["distribution", "instance_set", "beta_num"], as_index=False)
        .agg(total=("is_multi_visit", "size"), positive=("is_multi_visit", "sum"))
    )
    summary["percent"] = summary["positive"] / summary["total"] * 100.0

    all_betas = sorted(summary["beta_num"].dropna().unique().tolist())

    sns.set_theme(style="white")
    fig, axes = plt.subplots(1, 3, figsize=(22, 6), constrained_layout=True)
    cmap = sns.color_palette("YlGnBu", as_cmap=True)

    for ax, dist in zip(axes, DIST_ORDER):
        sub = summary[summary["distribution"] == dist]
        piv = sub.pivot(index="instance_set", columns="beta_num", values="percent")

        piv = piv.reindex(index=INSTANCE_ORDER)
        piv = piv.reindex(columns=all_betas)
        piv.columns = [beta_label(x) for x in piv.columns]

        sns.heatmap(
            piv,
            ax=ax,
            cmap=cmap,
            vmin=0,
            vmax=100,
            linewidths=0.6,
            linecolor="#F2F2F2",
            cbar=(dist == DIST_ORDER[-1]),
            annot=True,
            fmt=".1f",
            annot_kws={"size": 9},
            mask=piv.isna(),
        )

        ax.set_title(f"{dist.capitalize()}", fontsize=13, fontweight="bold")
        ax.set_xlabel("beta", fontsize=11)
        ax.set_ylabel("instance set" if dist == DIST_ORDER[0] else "", fontsize=11)

    fig.suptitle(
        "% Best Solutions with Multi-Visit (best_multi_visit_trip_count > 0)",
        fontsize=14,
        fontweight="bold",
    )

    out_pdf = Path(args.output_pdf)
    out_png = Path(args.output_png)
    out_pdf.parent.mkdir(parents=True, exist_ok=True)
    out_png.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(out_pdf, format="pdf", bbox_inches="tight")
    fig.savefig(out_png, format="png", dpi=args.dpi, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved PDF: {out_pdf}")
    print(f"Saved PNG: {out_png}")
    print(f"Rows used: {len(work)}")
    print(f"Betas: {[beta_label(b) for b in all_betas]}")


if __name__ == "__main__":
    main()
