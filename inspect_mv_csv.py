import ast
import csv

fp = r"result/multivisit_symmetric20_30_100_w4_1773450302.csv"
rows = list(csv.DictReader(open(fp, encoding="utf-8")))
ok = [r for r in rows if r.get("status") == "OK"]

best_mv_col = []
found_mv_sol = 0
mv_trip_counts = []

for r in ok:
    v = r.get("best_multi_visit_sol_multi_visit_trip_count", "")
    if v not in ("", None):
        try:
            best_mv_col.append(int(v))
        except Exception:
            pass

    s = r.get("best_multi_visit_sol", "")
    if s and s != "None":
        try:
            sol = ast.literal_eval(s)
            if isinstance(sol, list) and len(sol) > 1 and isinstance(sol[1], list):
                c = sum(1 for trip in sol[1] if isinstance(trip, list) and len(trip) > 1)
                mv_trip_counts.append(c)
                if c > 0:
                    found_mv_sol += 1
        except Exception:
            pass

best_sol_counts = sorted(
    set(int(r["best_sol_multi_visit_trip_count"]) for r in ok if r.get("best_sol_multi_visit_trip_count", "") != "")
)

print("rows_ok=", len(ok))
print("best_sol_multi_visit_trip_count unique=", best_sol_counts)
print("best_multi_visit_sol_multi_visit_trip_count unique=", sorted(set(best_mv_col)) if best_mv_col else [])
print("rows_with_best_multi_visit_sol=", len([r for r in ok if r.get("best_multi_visit_sol", "") not in ("", "None")]))
print("rows_with_parsed_mv_trip_count_gt0=", found_mv_sol)
print("parsed_best_multi_visit_trip_count_unique=", sorted(set(mv_trip_counts)) if mv_trip_counts else [])
