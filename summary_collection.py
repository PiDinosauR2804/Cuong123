
from pathlib import Path
import pandas as pd

root = Path("collected")
csv_files = sorted(root.rglob("rows_*.csv"))
if not csv_files:
    raise RuntimeError("No rows_*.csv files found")

all_df = pd.concat([pd.read_csv(fp) for fp in csv_files], ignore_index=True)

for col in ["segments","tabu_iterations","fitness_best_solution","fitness_best_solution_one_visit","fitness_best_solution_multi_visit","runtime_sec","best_t_segment"]:
    all_df[col] = pd.to_numeric(all_df[col], errors="coerce")
all_df["customer"] = pd.to_numeric(all_df["customer"], errors="coerce").astype("Int64")
all_df["run"] = pd.to_numeric(all_df["run"], errors="coerce").astype("Int64")
all_df["beta"] = all_df["beta"].astype(str)

sort_cols = ["customer","beta","dataset","run"]
no_div = all_df[all_df["strategy"]=="no_div"].copy().sort_values(sort_cols)
div = all_df[all_df["strategy"]=="div"].copy().sort_values(sort_cols)

compare = no_div.merge(div, on=["customer","beta","dataset","run"], suffixes=("_no_div","_div"), how="inner")
compare["delta_segments_div_minus_no_div"] = compare["segments_div"] - compare["segments_no_div"]
compare["delta_tabu_iterations_div_minus_no_div"] = compare["tabu_iterations_div"] - compare["tabu_iterations_no_div"]

compare = compare[[
    "customer","beta","dataset","run",
    "segments_no_div","tabu_iterations_no_div",
    "segments_div","tabu_iterations_div",
    "delta_segments_div_minus_no_div","delta_tabu_iterations_div_minus_no_div",
    "fitness_best_solution_no_div","fitness_best_solution_div",
    "runtime_sec_no_div","runtime_sec_div"
]].sort_values(sort_cols)

cols = ["customer","beta","dataset","run","segments","tabu_iterations","best_solution","best_solution_one_visit","best_solution_multi_visit","fitness_best_solution","fitness_best_solution_one_visit","fitness_best_solution_multi_visit","runtime_sec","best_t_segment"]

out_dir = Path("Result/github_actions_similarity_customer_beta_compare")
out_dir.mkdir(parents=True, exist_ok=True)
out_xlsx = out_dir / "similarity_compare_report.xlsx"

with pd.ExcelWriter(out_xlsx, engine="openpyxl") as writer:
    no_div[cols].to_excel(writer, sheet_name="no_div", index=False)
    div[cols].to_excel(writer, sheet_name="div", index=False)
    compare.to_excel(writer, sheet_name="compare", index=False)

print(out_xlsx)

