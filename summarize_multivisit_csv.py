import csv
from collections import defaultdict

fp = r"result/multivisit_symmetric20_w4_1773448363.csv"
rows = []
with open(fp, newline="", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

ok = [r for r in rows if r.get("status") == "OK"]
mv_counts = []
for r in ok:
    v = r.get("best_sol_multi_visit_trip_count", "")
    try:
        mv_counts.append(int(v))
    except Exception:
        pass

print("rows=", len(rows), "ok=", len(ok))
if mv_counts:
    print("mv_trip_count_min=", min(mv_counts))
    print("mv_trip_count_max=", max(mv_counts))
    print("mv_trip_count_avg=", round(sum(mv_counts) / len(mv_counts), 3))

by_inst = defaultdict(list)
for r in ok:
    by_inst[r["instance"]].append(int(r["best_sol_multi_visit_trip_count"]))

for inst in sorted(by_inst):
    vals = by_inst[inst]
    print(inst, "runs=", len(vals), "min=", min(vals), "max=", max(vals), "avg=", round(sum(vals) / len(vals), 3))
