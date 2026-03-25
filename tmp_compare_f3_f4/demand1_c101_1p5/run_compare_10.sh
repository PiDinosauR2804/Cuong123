#!/usr/bin/env bash
set -euo pipefail
inst="tmp_compare_f3_f4/demand1_c101_1p5/C101_1.5_demand1.dat"
A=4
L=90
runs=10
outdir="tmp_compare_f3_f4/demand1_c101_1p5/logs"
rm -rf "$outdir"
mkdir -p "$outdir"

tlim=8

# create a job list
jobfile="$outdir/jobs.txt"
: > "$jobfile"
for r in $(seq 1 $runs); do
  echo -e "F3\t$r\tC_Version/read_data_function3" >> "$jobfile"
  echo -e "F4\t$r\tC_Version/read_data_function4" >> "$jobfile"
done

# run up to 6 in parallel (each job is isolated in its own shell)
cat "$jobfile" | while IFS=$'\t' read -r algo rep bin; do
  printf '%s\0' "$algo" "$rep" "$bin"
done | xargs -0 -n 3 -P 6 bash -lc '
  algo="$1"; rep="$2"; bin="$3"
  log="'"$outdir"'/${algo}_rep${rep}.log"
  env \
    SOLVER_TIME_LIMIT_SEC="'"$tlim"'" \
    ATS_SEG=1 ATS_DIV=1 PROFILE_LS=0 \
    SKIP_ATS=1 SKIP_TRUCK_2OPT=1 \
    "$bin" "'"$inst"'" "'"$A"'" "'"$L"'" /dev/null >"$log" 2>&1
' _ 

python3 - <<'PY' | tee tmp_compare_f3_f4/demand1_c101_1p5/summary10.tsv
import ast, re, glob, os, math
from statistics import mean

def extract_solution_block(lines):
    start=None
    for i,ln in enumerate(lines):
        if ln.strip().startswith('solution ='):
            start=i
            break
    if start is None:
        return None
    text='\n'.join(lines[start:])
    m=re.search(r'solution\s*=\s*(\[)', text)
    if not m:
        return None
    idx=text.find('[', m.start(1))
    s=text[idx:]
    depth=0
    end=None
    for j,ch in enumerate(s):
        if ch=='[': depth+=1
        elif ch==']':
            depth-=1
            if depth==0:
                end=j+1
                break
    if end is None:
        return None
    return s[:end]

def analyze(sol):
    trucks, drones = sol
    owner={}
    for t,route in enumerate(trucks):
        for cust,_ld in route:
            if cust!=0:
                owner[cust]=t
    trip_cnt=len(drones)
    mv_cnt=0
    legs=0
    for trip in drones:
        legs += len(trip)
        trucks_seen=set()
        for rv,_pkgs in trip:
            trucks_seen.add(owner.get(rv, -999))
        if len(trucks_seen - {-999}) >= 2:
            mv_cnt += 1
    return trip_cnt, mv_cnt, legs

def parse_makespan(lines):
    mk=None
    for ln in reversed(lines):
        m=re.match(r'^Makespan:\s*([0-9]+(?:\.[0-9]+)?)', ln.strip())
        if m:
            mk=float(m.group(1))
            break
    return mk

rows=[]
for path in sorted(glob.glob('tmp_compare_f3_f4/demand1_c101_1p5/logs/*.log')):
    lines=open(path,'r',errors='ignore').read().splitlines()
    mk=parse_makespan(lines)
    sol_txt=extract_solution_block(lines)
    trip_cnt=mv_cnt=legs=None
    if sol_txt:
        try:
            sol=ast.literal_eval(sol_txt)
            trip_cnt,mv_cnt,legs=analyze(sol)
        except Exception:
            pass
    base=os.path.basename(path)
    algo, rep = base.split('_rep')
    rep=int(rep.split('.')[0])
    rows.append((algo, rep, mk, trip_cnt, mv_cnt, legs))

print('algo\trep\tmakespan\tdrone_trips\tmulti_trips\tlegs')
for a,r,mk,tc,mv,lg in sorted(rows):
    print(f'{a}\t{r}\t{mk if mk is not None else ""}\t{tc if tc is not None else ""}\t{mv if mv is not None else ""}\t{lg if lg is not None else ""}')

print('\nAGG')
for algo in ['F3','F4']:
    vals=[mk for a,_,mk,_,_,_ in rows if a==algo and mk is not None]
    if not vals:
        print(f'{algo}: no data')
        continue
    vals=sorted(vals)
    print(f'{algo}: n={len(vals)} best={vals[0]:.6f} mean={mean(vals):.6f} worst={vals[-1]:.6f}')

# Multi-visit rate
for algo in ['F3','F4']:
    mvs=[mv for a,_,_,_,mv,_ in rows if a==algo and mv is not None]
    rate=sum(1 for x in mvs if x>0)/len(mvs) if mvs else 0
    print(f'{algo}: multi_visit_runs={sum(1 for x in mvs if x>0)}/{len(mvs)} rate={rate:.2f}')
PY
