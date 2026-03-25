#!/usr/bin/env bash
set -euo pipefail
inst="tmp_compare_f3_f4/demand1_c101_1p5/C101_1.5_demand1.dat"
A=4
L=90
runs=3
outdir="tmp_compare_f3_f4/full_pipeline_10/logs"
rm -rf "$outdir"; mkdir -p "$outdir"

tlim=30
jobfile="$outdir/jobs.txt"
:>"$jobfile"
for r in $(seq 1 $runs); do
  echo -e "F3\t$r\tC_Version/read_data_function3" >> "$jobfile"
  echo -e "F4\t$r\tC_Version/read_data_function4" >> "$jobfile"
done

cat "$jobfile" | while IFS=$'\t' read -r algo rep bin; do
  printf '%s\0' "$algo" "$rep" "$bin"
done | xargs -0 -n 3 -P 6 bash -lc '
  algo="$1"; rep="$2"; bin="$3"
  log="'"$outdir"'/${algo}_rep${rep}.log"
  env \
    SOLVER_TIME_LIMIT_SEC="'"$tlim"'" \
    ATS_SEG=1 ATS_DIV=1 ATS_MAX_ITERS=10 \
    LS_MAX_APPLIED=10 \
    PROFILE_LS=0 \
    "$bin" "'"$inst"'" "'"$A"'" "'"$L"'" /dev/null >"$log" 2>&1
' _

python3 - <<'PY' | tee tmp_compare_f3_f4/full_pipeline_10/summary.tsv
import ast, re, glob, os
from statistics import mean

def extract_solution(lines):
    for i,ln in enumerate(lines):
        if ln.strip().startswith('solution ='):
            text='\n'.join(lines[i:])
            idx=text.find('[')
            s=text[idx:]
            d=0
            for j,ch in enumerate(s):
                if ch=='[': d+=1
                elif ch==']':
                    d-=1
                    if d==0:
                        return s[:j+1]
    return None

def parse_mk(lines):
    for ln in reversed(lines):
        m=re.match(r'^Makespan:\s*([0-9]+(?:\.[0-9]+)?)', ln.strip())
        if m:
            return float(m.group(1))
    return None

def analyze(sol):
    trucks, drones = sol
    owner={}
    for t,route in enumerate(trucks):
        for cust,_ in route:
            if cust!=0: owner[cust]=t
    trip_cnt=len(drones)
    mv=0
    legs=0
    for trip in drones:
        legs += len(trip)
        ts=set(owner.get(rv,-1) for rv,_ in trip)
        ts.discard(-1)
        if len(ts)>=2: mv+=1
    return trip_cnt, mv, legs

rows=[]
for p in sorted(glob.glob("tmp_compare_f3_f4/full_pipeline_10/logs/*.log")):
    lines=open(p,'r',errors='ignore').read().splitlines()
    mk=parse_mk(lines)
    soltxt=extract_solution(lines)
    trip=mv=legs=None
    if soltxt:
        try:
            sol=ast.literal_eval(soltxt)
            trip,mv,legs=analyze(sol)
        except Exception:
            pass
    base=os.path.basename(p)
    algo, rep = base.split('_rep')
    rep=int(rep.split('.')[0])
    rows.append((algo,rep,mk,trip,mv,legs))

print('algo\trep\tmakespan\tdrone_trips\tmulti_trips\tlegs')
for r in sorted(rows):
    print('\t'.join('' if x is None else str(x) for x in r))

print('\nAGG')
for algo in ['F3','F4']:
    vals=[mk for a,_,mk,_,_,_ in rows if a==algo and mk is not None]
    if vals:
        vals=sorted(vals)
        print(f'{algo}: n={len(vals)} best={vals[0]:.6f} mean={mean(vals):.6f} worst={vals[-1]:.6f}')
    else:
        print(f'{algo}: no data')
PY
