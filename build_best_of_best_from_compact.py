from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Dict, Optional, Tuple

INPUT = Path('summary_all_best_compact.csv')
OUTPUT = Path('best_of_best.csv')


def to_float(x: str) -> Optional[float]:
    t = str(x or '').strip()
    if not t:
        return None
    try:
        v = float(t)
        if math.isnan(v) or math.isinf(v):
            return None
        return v
    except Exception:
        return None


def fmt(v: Optional[float]) -> str:
    if v is None:
        return ''
    s = f"{v:.9f}".rstrip('0').rstrip('.')
    return s if s else '0'


def better_or_equal(a: Optional[float], b: Optional[float]) -> bool:
    """Return True if a <= b with missing treated as +inf."""
    if a is None and b is None:
        return True
    if a is None:
        return False
    if b is None:
        return True
    return a <= b + 1e-12


def should_replace(target_fit: Optional[float], source_fit: Optional[float]) -> bool:
    """
    Replace only when target is strictly worse than source.
    Equal fitness keeps original solution (to preserve diversity).
    Missing target is treated as worse than any present source.
    """
    if source_fit is None:
        return False
    if target_fit is None:
        return True
    return target_fit > source_fit + 1e-12


def maybe_copy_metric(
    rows_by_key: Dict[Tuple[str, str, str], Dict[str, str]],
    tgt_key: Tuple[str, str, str],
    src_key: Tuple[str, str, str],
    fit_col: str,
    sol_col: str,
) -> bool:
    tgt = rows_by_key.get(tgt_key)
    src = rows_by_key.get(src_key)
    if tgt is None or src is None:
        return False

    tgt_fit = to_float(tgt.get(fit_col, ''))
    src_fit = to_float(src.get(fit_col, ''))

    if should_replace(tgt_fit, src_fit):
        # copy both fitness and solution from source
        tgt[fit_col] = fmt(src_fit)
        tgt[sol_col] = src.get(sol_col, '')
        return True
    return False


def main() -> None:
    if not INPUT.exists():
        raise FileNotFoundError(INPUT)

    with INPUT.open('r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        fields = list(reader.fieldnames or [])
        rows = list(reader)

    # Keep output columns order as input.
    if not fields:
        fields = [
            'instance', 'A', 'L', 'best_solution', 'best_fitness',
            'best_multi_solution', 'best_multi_fitness'
        ]

    rows_by_key: Dict[Tuple[str, str, str], Dict[str, str]] = {}
    for r in rows:
        key = (str(r.get('instance', '')).strip(), str(r.get('A', '')).strip(), str(r.get('L', '')).strip())
        rows_by_key[key] = r

    upd_best = 0
    upd_multi = 0

    # Group by instance
    instances = sorted({k[0] for k in rows_by_key.keys()})
    for inst in instances:
        k460 = (inst, '4', '60')
        k490 = (inst, '4', '90')
        k4120 = (inst, '4', '120')
        k860 = (inst, '8', '60')
        k890 = (inst, '8', '90')
        k8120 = (inst, '8', '120')

        # A=4, L monotonic: 60 -> 90 -> 120
        if maybe_copy_metric(rows_by_key, k490, k460, 'best_fitness', 'best_solution'):
            upd_best += 1
        if maybe_copy_metric(rows_by_key, k490, k460, 'best_multi_fitness', 'best_multi_solution'):
            upd_multi += 1

        if maybe_copy_metric(rows_by_key, k4120, k490, 'best_fitness', 'best_solution'):
            upd_best += 1
        if maybe_copy_metric(rows_by_key, k4120, k490, 'best_multi_fitness', 'best_multi_solution'):
            upd_multi += 1

        # A=8 should be <= A=4 at same L
        if maybe_copy_metric(rows_by_key, k860, k460, 'best_fitness', 'best_solution'):
            upd_best += 1
        if maybe_copy_metric(rows_by_key, k860, k460, 'best_multi_fitness', 'best_multi_solution'):
            upd_multi += 1

        if maybe_copy_metric(rows_by_key, k890, k490, 'best_fitness', 'best_solution'):
            upd_best += 1
        if maybe_copy_metric(rows_by_key, k890, k490, 'best_multi_fitness', 'best_multi_solution'):
            upd_multi += 1

        if maybe_copy_metric(rows_by_key, k8120, k4120, 'best_fitness', 'best_solution'):
            upd_best += 1
        if maybe_copy_metric(rows_by_key, k8120, k4120, 'best_multi_fitness', 'best_multi_solution'):
            upd_multi += 1

    # Write output sorted by instance, A, L
    def sort_key(r: Dict[str, str]):
        inst = str(r.get('instance', ''))
        try:
            a = float(str(r.get('A', '')).strip() or 0)
        except Exception:
            a = 0.0
        try:
            l = float(str(r.get('L', '')).strip() or 0)
        except Exception:
            l = 0.0
        return (inst, a, l)

    out_rows = sorted(rows_by_key.values(), key=sort_key)

    with OUTPUT.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(out_rows)

    both = sum(1 for r in out_rows if str(r.get('best_solution', '')).strip() and str(r.get('best_multi_solution', '')).strip())
    only_best = sum(1 for r in out_rows if str(r.get('best_solution', '')).strip() and not str(r.get('best_multi_solution', '')).strip())
    only_multi = sum(1 for r in out_rows if (not str(r.get('best_solution', '')).strip()) and str(r.get('best_multi_solution', '')).strip())
    empty_both = sum(1 for r in out_rows if (not str(r.get('best_solution', '')).strip()) and (not str(r.get('best_multi_solution', '')).strip()))

    print(f'input_rows={len(rows)}')
    print(f'output_rows={len(out_rows)}')
    print(f'updated_best={upd_best}')
    print(f'updated_multi={upd_multi}')
    print(f'both_present={both}')
    print(f'only_best={only_best}')
    print(f'only_multi={only_multi}')
    print(f'both_empty={empty_both}')
    print(f'output_file={OUTPUT.resolve()}')


if __name__ == '__main__':
    main()
