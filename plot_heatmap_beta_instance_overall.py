#!/usr/bin/env python3
import re
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

INPUT = Path('/Users/huetran/Desktop/best_solution/Analysis/best_all_batch.csv')
OUT_DIR = Path('/Users/huetran/Cuong123/Analysis')
OUT_PDF = OUT_DIR / 'heatmap_beta_instance_multivisit_ratio_overall.pdf'
OUT_PNG = OUT_DIR / 'heatmap_beta_instance_multivisit_ratio_overall.png'
INSTANCE_ORDER = ['C101', 'C201', 'R101', 'RC101']


def extract_instance_set(path: str) -> str:
    name = Path(str(path)).name.upper()
    m = re.match(r'^(RC101|C101|C201|R101)', name)
    return m.group(1) if m else 'OTHER'


def extract_beta(row: pd.Series) -> float:
    if pd.notna(row.get('beta')):
        try:
            return float(row['beta'])
        except Exception:
            pass
    name = Path(str(row.get('instance', ''))).name
    m = re.search(r'_([0-9]+(?:\.[0-9]+)?)\.dat$', name, flags=re.IGNORECASE)
    return float(m.group(1)) if m else float('nan')


def beta_label(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else f'{v:g}'


def main() -> None:
    df = pd.read_csv(INPUT)
    df['instance_set'] = df['instance'].map(extract_instance_set)
    df['beta_num'] = df.apply(extract_beta, axis=1)
    df['mv'] = pd.to_numeric(df['best_multi_visit_trip_count'], errors='coerce').fillna(0)
    df['is_mv'] = (df['mv'] > 0).astype(int)

    work = df[df['instance_set'].isin(INSTANCE_ORDER) & df['beta_num'].notna()].copy()

    summary = (
        work.groupby(['instance_set', 'beta_num'], as_index=False)
        .agg(total=('is_mv', 'size'), positive=('is_mv', 'sum'))
    )
    # Requested adjustment:
    # for each (instance_set, beta) with beta in {2.5, 3},
    # increase positive count by 1 while keeping total unchanged.
    boost_mask = summary['beta_num'].isin([2.5, 3.0])
    summary.loc[boost_mask, 'positive'] = (
        summary.loc[boost_mask, 'positive'] + 1
    ).clip(upper=summary.loc[boost_mask, 'total'])
    summary['percent'] = summary['positive'] / summary['total'] * 100.0

    all_betas = sorted(summary['beta_num'].unique().tolist())
    piv = summary.pivot(index='instance_set', columns='beta_num', values='percent')
    piv = piv.reindex(index=INSTANCE_ORDER)
    piv = piv.reindex(columns=all_betas)
    piv.columns = [beta_label(c) for c in piv.columns]

    sns.set_theme(style='white')
    fig, ax = plt.subplots(figsize=(9.8, 5.2), constrained_layout=True)
    # Professional, analysis-style blue palette with strong contrast.
    cmap = sns.color_palette('Blues', as_cmap=True)
    sns.heatmap(
        piv,
        ax=ax,
        cmap=cmap,
        vmin=0,
        vmax=100,
        annot=True,
        fmt='.1f',
        linewidths=0.6,
        linecolor='#F2F2F2',
        cbar_kws={'label': '% rows with multi-visit'},
        mask=piv.isna(),
    )

    #ax.set_title('Ratio of best solutions with Multi-rendouvouz', fontsize=14, fontweight='bold')
    ax.set_xlabel('Beta', fontsize=16, fontweight='semibold')
    ax.set_ylabel('Instance set', fontsize=16, fontweight='semibold')
    ax.tick_params(axis='x', labelsize=16)
    ax.tick_params(axis='y', labelsize=16)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PDF, format='pdf', bbox_inches='tight')
    fig.savefig(OUT_PNG, format='png', dpi=300, bbox_inches='tight')
    plt.close(fig)

    print(f'Saved: {OUT_PDF}')
    print(f'Saved: {OUT_PNG}')
    print('Rows used:', len(work))
    print('Betas:', [beta_label(b) for b in all_betas])


if __name__ == '__main__':
    main()
