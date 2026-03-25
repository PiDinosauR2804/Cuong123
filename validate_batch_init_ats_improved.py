import csv
import os
import pathlib
import subprocess
import sys
import tempfile


def run_validate_text(instance: str, a: str, l: str, solution_text: str) -> tuple[bool, str]:
    # Wrap into a seed file the parser understands (anything before '=' is ignored).
    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".txt") as f:
        f.write("solution = " + solution_text.strip() + "\n")
        seed_path = f.name

    try:
        env = dict(os.environ)
        env["VALIDATE_ONLY"] = "1"
        # Prevent accidental long runs if VALIDATE_ONLY is not respected.
        env.setdefault("SOLVER_TIME_LIMIT_SEC", "5")
        cmd = ["C_Version/read_data_function2", instance, str(a), str(l), seed_path]
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        out = (p.stdout or "").strip()
        ok = (p.returncode == 0) and ("[VALIDATE] OK" in out)
        if ok:
            # Keep a compact one-liner.
            line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
            return True, line
        else:
            line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
            if not line:
                line = f"exit={p.returncode}"
            return False, line
    finally:
        try:
            os.unlink(seed_path)
        except OSError:
            pass


def run_validate_file(instance: str, a: str, l: str, seed_file: str) -> tuple[bool, str]:
    env = dict(os.environ)
    env["VALIDATE_ONLY"] = "1"
    env.setdefault("SOLVER_TIME_LIMIT_SEC", "5")
    cmd = ["C_Version/read_data_function2", instance, str(a), str(l), seed_file]
    p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = (p.stdout or "").strip()
    ok = (p.returncode == 0) and ("[VALIDATE] OK" in out)
    if ok:
        line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
        return True, line
    else:
        line = next((ln for ln in out.splitlines() if "[VALIDATE]" in ln), out.splitlines()[-1] if out else "")
        if not line:
            line = f"exit={p.returncode}"
        return False, line


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python3 validate_batch_init_ats_improved.py <batch_init_ats_improved.csv>")
        return 2

    csv_path = pathlib.Path(sys.argv[1])
    if not csv_path.exists():
        print(f"CSV not found: {csv_path}")
        return 2

    ok_best = 0
    ok_best_multi = 0
    ok_best_file = 0
    ok_best_multi_file = 0

    fail_best: list[dict] = []
    fail_best_multi: list[dict] = []
    fail_best_file: list[dict] = []
    fail_best_multi_file: list[dict] = []
    total = 0

    with csv_path.open(newline="") as f:
        r = csv.DictReader(f)
        required = {"job_id", "instance", "A", "L", "best_solution", "best_multi_solution", "best_solution_file", "best_multi_solution_file"}
        missing = required - set(r.fieldnames or [])
        if missing:
            print(f"Missing columns: {sorted(missing)}")
            print(f"Have columns: {r.fieldnames}")
            return 2

        for row in r:
            total += 1
            inst = (row.get("instance") or "").strip()
            a = (row.get("A") or "").strip()
            l = (row.get("L") or "").strip()

            sol = (row.get("best_solution") or "").strip()
            if sol:
                ok, msg = run_validate_text(inst, a, l, sol)
                if ok:
                    ok_best += 1
                else:
                    fail_best.append({"job_id": row.get("job_id", ""), "instance": inst, "A": a, "L": l, "status": "FAIL", "reason": msg})
            else:
                fail_best.append({"job_id": row.get("job_id", ""), "instance": inst, "A": a, "L": l, "status": "FAIL", "reason": "empty best_solution"})

            solm = (row.get("best_multi_solution") or "").strip()
            if solm:
                ok, msg = run_validate_text(inst, a, l, solm)
                if ok:
                    ok_best_multi += 1
                else:
                    fail_best_multi.append({"job_id": row.get("job_id", ""), "instance": inst, "A": a, "L": l, "status": "FAIL", "reason": msg})

            sf = (row.get("best_solution_file") or "").strip()
            if sf and sf != "(not found)":
                if not pathlib.Path(sf).exists():
                    fail_best_file.append({"job_id": row.get("job_id", ""), "instance": inst, "A": a, "L": l, "status": "FAIL", "reason": f"file not found: {sf}"})
                else:
                    ok, msg = run_validate_file(inst, a, l, sf)
                    if ok:
                        ok_best_file += 1
                    else:
                        fail_best_file.append({"job_id": row.get("job_id", ""), "instance": inst, "A": a, "L": l, "status": "FAIL", "reason": msg})

            smf = (row.get("best_multi_solution_file") or "").strip()
            if smf and smf != "(not found)":
                if not pathlib.Path(smf).exists():
                    fail_best_multi_file.append({"job_id": row.get("job_id", ""), "instance": inst, "A": a, "L": l, "status": "FAIL", "reason": f"file not found: {smf}"})
                else:
                    ok, msg = run_validate_file(inst, a, l, smf)
                    if ok:
                        ok_best_multi_file += 1
                    else:
                        fail_best_multi_file.append({"job_id": row.get("job_id", ""), "instance": inst, "A": a, "L": l, "status": "FAIL", "reason": msg})

    print(f"Checked rows: {total}")
    print(f"Valid best_solution (text): {ok_best} / {total}")
    print(f"Invalid best_solution (text): {len(fail_best)}")
    print(f"Valid best_multi_solution (text): {ok_best_multi}")
    print(f"Invalid best_multi_solution (text): {len(fail_best_multi)}")
    print(f"Valid best_solution_file: {ok_best_file}")
    print(f"Invalid best_solution_file: {len(fail_best_file)}")
    print(f"Valid best_multi_solution_file: {ok_best_multi_file}")
    print(f"Invalid best_multi_solution_file: {len(fail_best_multi_file)}")

    def write_report(name: str, rows: list[dict]) -> None:
        if not rows:
            return
        out_csv = pathlib.Path(name)
        with out_csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["job_id", "instance", "A", "L", "status", "reason"])
            w.writeheader()
            w.writerows(rows)
        print(f"Wrote failures report: {out_csv}")

    write_report("batch_init_ats_improved_best_solution_validation_report.csv", fail_best)
    write_report("batch_init_ats_improved_best_multi_solution_validation_report.csv", fail_best_multi)
    write_report("batch_init_ats_improved_best_solution_file_validation_report.csv", fail_best_file)
    write_report("batch_init_ats_improved_best_multi_solution_file_validation_report.csv", fail_best_multi_file)

    any_fail = bool(fail_best or fail_best_multi or fail_best_file or fail_best_multi_file)
    return 0 if not any_fail else 1


if __name__ == "__main__":
    raise SystemExit(main())
