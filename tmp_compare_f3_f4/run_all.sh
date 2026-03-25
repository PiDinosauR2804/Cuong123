#!/usr/bin/env bash
set -euo pipefail
inst="test_data/data_demand_random/10/C101_0.5.dat"
A=4
L=60
# 1 segment ATS, 1 diversification block, cap wall time
tlim=120
common_env=("SOLVER_TIME_LIMIT_SEC=$tlim" "ATS_SEG=1" "ATS_DIV=1" "PROFILE_LS=1")
run_one() {
  local algo="$1" rep="$2" bin="$3"
  local log="tmp_compare_f3_f4/${algo}_rep${rep}.log"
  (env "${common_env[@]}" "$bin" "$inst" "$A" "$L" >"$log" 2>&1) &
}
for r in 1 2 3; do
  run_one F3 "$r" C_Version/read_data_function3
  run_one F4 "$r" C_Version/read_data_function4
done
wait

python3 - <<'PY' | tee tmp_compare_f3_f4/summary.tsv
import re, glob, os, math

def parse_log(path):
    txt=open(path,'r',errors='ignore').read().splitlines()
    makes=None
    # take last 'Makespan:'
    for ln in reversed(txt):
        m=re.search(r'^Makespan:\s*([0-9]+(?:\.[0-9]+)?)', ln.strip())
        if m:
            makes=float(m.group(1));
            break
    # profile
    prof=None
    for ln in reversed(txt):
        if 'PROFILE:' in ln or '[PROFILE]' in ln:
            prof=ln.strip();
            break
    # quick counts (best effort)
    trips=None
    mult=None
    # There is no explicit print; try from solution json line or metrics; fallback None
    for ln in reversed(txt):
        if ln.startswith('Solution') and '[[[' in ln:
            break
    return makes, prof

rows=[]
for p in sorted(glob.glob('tmp_compare_f3_f4/*.log')):
    base=os.path.basename(p)
    algo, rep = base.split('_rep')
    rep=rep.split('.')[0]
    mk, prof = parse_log(p)
    rows.append((algo, int(rep), mk if mk is not None else math.inf))

# print summary
print('algo\trep\tmakespan')
for a,r,mk in rows:
    print(f'{a}\t{r}\t{mk if mk<1e100 else ""}')

# aggregate
from collections import defaultdict
agg=defaultdict(list)
for a,r,mk in rows:
    if mk<1e100:
        agg[a].append(mk)
print('\nAGG')
for a,vals in agg.items():
    vals=sorted(vals)
    mean=sum(vals)/len(vals)
    print(f'{a}: n={len(vals)} best={vals[0]:.6f} mean={mean:.6f} worst={vals[-1]:.6f}')
PY
