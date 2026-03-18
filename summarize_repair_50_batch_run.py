from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, Request, build_opener, urlopen
from zipfile import ZipFile


API_BASE = "https://api.github.com"
API_VERSION = "2022-11-28"
REDIRECT_CODES = {301, 302, 303, 307, 308}


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):  # type: ignore[override]
        return None


def _sanitize_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", name)
    return cleaned.strip("._") or "artifact"


def _detect_owner_repo() -> Tuple[Optional[str], Optional[str]]:
    try:
        result = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None, None

    url = result.stdout.strip()
    if not url:
        return None, None

    # https://github.com/owner/repo.git
    m = re.search(r"github\.com[:/](?P<owner>[^/]+)/(?P<repo>[^/]+?)(?:\.git)?$", url)
    if not m:
        return None, None
    return m.group("owner"), m.group("repo")


def _api_headers(token: str) -> Dict[str, str]:
    return {
        "Accept": "application/vnd.github+json",
        "Authorization": f"Bearer {token}",
        "X-GitHub-Api-Version": API_VERSION,
        "User-Agent": "summarize-repair-50-batch-run",
    }


def _api_get_json(url: str, token: str) -> Dict[str, Any]:
    req = Request(url=url, headers=_api_headers(token), method="GET")
    try:
        with urlopen(req) as resp:
            payload = resp.read().decode("utf-8")
    except HTTPError as e:
        detail = e.read().decode("utf-8", errors="ignore")
        raise RuntimeError(f"GitHub API HTTPError {e.code} for {url}\n{detail}") from e
    except URLError as e:
        raise RuntimeError(f"GitHub API URLError for {url}: {e}") from e
    return json.loads(payload)


def _download_artifact_zip(owner: str, repo: str, artifact_id: int, token: str, out_zip: Path) -> None:
    url = f"{API_BASE}/repos/{owner}/{repo}/actions/artifacts/{artifact_id}/zip"
    req = Request(url=url, headers=_api_headers(token), method="GET")
    opener = build_opener(_NoRedirect())
    try:
        with opener.open(req) as resp:
            # Some environments may return bytes directly without redirect.
            if int(resp.getcode() or 0) == 200:
                data = resp.read()
                out_zip.parent.mkdir(parents=True, exist_ok=True)
                out_zip.write_bytes(data)
                return
            redirect_url = (resp.headers.get("Location") or "").strip()
            if not redirect_url:
                raise RuntimeError(
                    f"Download artifact failed: missing redirect Location header (artifact_id={artifact_id})."
                )
    except HTTPError as e:
        if e.code in REDIRECT_CODES:
            redirect_url = (e.headers.get("Location") or "").strip()
            if not redirect_url:
                detail = e.read().decode("utf-8", errors="ignore")
                raise RuntimeError(
                    f"Download artifact redirect missing Location (artifact_id={artifact_id})\n{detail}"
                ) from e
        else:
            detail = e.read().decode("utf-8", errors="ignore")
            raise RuntimeError(f"Download artifact HTTPError {e.code} (artifact_id={artifact_id})\n{detail}") from e
    except URLError as e:
        raise RuntimeError(f"Download artifact URLError (artifact_id={artifact_id}): {e}") from e

    # Follow signed URL WITHOUT Authorization header to avoid blob-storage 401.
    signed_req = Request(
        url=redirect_url,
        headers={"User-Agent": "summarize-repair-50-batch-run"},
        method="GET",
    )
    try:
        with urlopen(signed_req) as resp:
            data = resp.read()
    except HTTPError as e:
        detail = e.read().decode("utf-8", errors="ignore")
        raise RuntimeError(
            f"Download signed artifact URL HTTPError {e.code} (artifact_id={artifact_id})\n{detail}"
        ) from e
    except URLError as e:
        raise RuntimeError(f"Download signed artifact URL URLError (artifact_id={artifact_id}): {e}") from e

    out_zip.parent.mkdir(parents=True, exist_ok=True)
    out_zip.write_bytes(data)


def _list_run_artifacts(owner: str, repo: str, run_id: int, token: str) -> List[Dict[str, Any]]:
    artifacts: List[Dict[str, Any]] = []
    page = 1
    while True:
        url = f"{API_BASE}/repos/{owner}/{repo}/actions/runs/{run_id}/artifacts?per_page=100&page={page}"
        payload = _api_get_json(url, token)
        items = payload.get("artifacts", []) or []
        if not items:
            break
        artifacts.extend(items)
        if len(items) < 100:
            break
        page += 1
    return artifacts


def _extract_zip(zip_path: Path, extract_dir: Path) -> None:
    extract_dir.mkdir(parents=True, exist_ok=True)
    with ZipFile(zip_path, "r") as zf:
        zf.extractall(extract_dir)


def _read_csv(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader.fieldnames or []), list(reader)


def _write_csv(path: Path, fieldnames: List[str], rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _choose_first_existing(paths: List[Path]) -> Optional[Path]:
    for p in paths:
        if p.exists():
            return p
    return None


def _merge_from_stage1_and_patches(
    artifact_dirs: List[Tuple[str, Path]],
    out_improved_csv: Path,
    out_compare_csv: Path,
) -> Dict[str, Any]:
    stage1_candidates: List[Path] = []
    for name, root in artifact_dirs:
        if name.startswith("stage1-repaired-"):
            stage1_candidates.extend(sorted(root.rglob("stage1_repaired.csv")))

    stage1_csv = _choose_first_existing(stage1_candidates)
    if stage1_csv is None:
        raise RuntimeError("Cannot rebuild final CSV: missing stage1_repaired.csv artifact.")

    fieldnames, rows = _read_csv(stage1_csv)
    patch_files: List[Path] = []
    compare_files: List[Path] = []
    for name, root in artifact_dirs:
        if name.startswith("stage2-chunk-"):
            patch_files.extend(sorted(root.rglob("stage2_chunk_*_patch.csv")))
            compare_files.extend(sorted(root.rglob("stage2_chunk_*_compare.csv")))

    if not patch_files:
        raise RuntimeError("Cannot rebuild final CSV: no stage2_chunk_*_patch.csv found.")

    applied = 0
    non_ok = 0
    for patch_path in patch_files:
        _, patch_rows = _read_csv(patch_path)
        for p in patch_rows:
            idx_text = (p.get("row_index") or "").strip()
            try:
                idx = int(float(idx_text))
            except Exception:
                continue
            if idx < 1 or idx > len(rows):
                continue
            target = rows[idx - 1]
            target["best_fitness"] = p.get("best_fitness", "")
            target["best_sol"] = p.get("best_sol", "")
            target["best_multi_visit_fitness"] = p.get("best_multi_visit_fitness", "")
            target["best_multi_visit_sol"] = p.get("best_multi_visit_sol", "")
            applied += 1
            if (p.get("status") or "").upper() != "OK":
                non_ok += 1

    _write_csv(out_improved_csv, fieldnames, rows)

    merged_compare: List[Dict[str, Any]] = []
    compare_fieldnames: List[str] = []
    for cp in compare_files:
        cp_fields, cp_rows = _read_csv(cp)
        for k in cp_fields:
            if k not in compare_fieldnames:
                compare_fieldnames.append(k)
        merged_compare.extend(cp_rows)
    merged_compare.sort(key=lambda r: int(float((r.get("row_index") or "0"))))
    if compare_fieldnames:
        _write_csv(out_compare_csv, compare_fieldnames, merged_compare)

    return {
        "mode": "rebuild_from_stage1_and_patches",
        "stage1_csv": str(stage1_csv),
        "patch_files": len(patch_files),
        "compare_files": len(compare_files),
        "patch_rows_applied": applied,
        "patch_rows_non_ok": non_ok,
    }


def _build_compact_summary(
    improved_csv: Path,
    compare_csv: Optional[Path],
    out_summary_csv: Path,
) -> Dict[str, Any]:
    _, improved_rows = _read_csv(improved_csv)

    compare_by_row: Dict[int, Dict[str, str]] = {}
    if compare_csv is not None and compare_csv.exists():
        _, compare_rows = _read_csv(compare_csv)
        for r in compare_rows:
            idx_text = (r.get("row_index") or "").strip()
            try:
                idx = int(float(idx_text))
            except Exception:
                continue
            compare_by_row[idx] = r

    summary_rows: List[Dict[str, Any]] = []
    for idx, row in enumerate(improved_rows, start=1):
        cp = compare_by_row.get(idx, {})
        summary_rows.append(
            {
                "row_index": idx,
                "instance": row.get("instance", ""),
                "run": row.get("run", ""),
                "drone_capacity": row.get("drone_capacity", ""),
                "drone_limit_time": row.get("drone_limit_time", ""),
                "best_fitness": row.get("best_fitness", ""),
                "best_multi_visit_fitness": row.get("best_multi_visit_fitness", ""),
                "best_sol": row.get("best_sol", ""),
                "best_multi_visit_sol": row.get("best_multi_visit_sol", ""),
                "status": cp.get("status", ""),
                "error": cp.get("error", ""),
                "ats_segments_run": cp.get("ats_segments_run", ""),
                "ats_time_limit_reached": cp.get("ats_time_limit_reached", ""),
                "ats_elapsed_sec": cp.get("ats_elapsed_sec", ""),
            }
        )

    fields = [
        "row_index",
        "instance",
        "run",
        "drone_capacity",
        "drone_limit_time",
        "best_fitness",
        "best_multi_visit_fitness",
        "best_sol",
        "best_multi_visit_sol",
        "status",
        "error",
        "ats_segments_run",
        "ats_time_limit_reached",
        "ats_elapsed_sec",
    ]
    _write_csv(out_summary_csv, fields, summary_rows)

    ok_rows = sum(1 for r in summary_rows if (r.get("status") or "").upper() == "OK")
    return {"summary_rows": len(summary_rows), "ok_rows": ok_rows, "error_rows": len(summary_rows) - ok_rows}


def _select_relevant_artifacts(artifacts: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    final = [a for a in artifacts if (a.get("name") or "").startswith("repair-improve-50-batch-final-")]
    if final:
        # One final artifact is enough; avoid downloading hundreds of chunk artifacts.
        return sorted(final, key=lambda a: int(a.get("id") or 0), reverse=True)[:1]

    stage1 = [a for a in artifacts if (a.get("name") or "").startswith("stage1-repaired-")]
    stage2 = [a for a in artifacts if (a.get("name") or "").startswith("stage2-chunk-")]
    if stage1 and stage2:
        return stage1 + stage2

    return artifacts


def _copy_if_exists(src: Path, dst: Path) -> bool:
    if not src.exists():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Download artifacts of a repair-50-batch GitHub Actions run, "
            "then aggregate improved solutions into one CSV."
        )
    )
    parser.add_argument("--run-id", type=int, required=True, help="GitHub Actions run ID, e.g. 23226499520.")
    parser.add_argument("--owner", default="", help="GitHub owner/org. Auto-detected from git origin if omitted.")
    parser.add_argument("--repo", default="", help="GitHub repo name. Auto-detected from git origin if omitted.")
    parser.add_argument("--token", default="", help="GitHub token (or set GH_TOKEN / GITHUB_TOKEN).")
    parser.add_argument(
        "--output-dir",
        default="Result/github_run_summaries",
        help="Output root directory for downloaded artifacts and merged CSV.",
    )
    parser.add_argument("--keep-zips", action="store_true", help="Keep downloaded .zip files.")
    args = parser.parse_args()

    token = args.token.strip() or os.environ.get("GH_TOKEN", "") or os.environ.get("GITHUB_TOKEN", "")
    if not token:
        raise SystemExit("Missing token. Use --token or set GH_TOKEN / GITHUB_TOKEN.")

    owner = args.owner.strip()
    repo = args.repo.strip()
    if not owner or not repo:
        auto_owner, auto_repo = _detect_owner_repo()
        owner = owner or (auto_owner or "")
        repo = repo or (auto_repo or "")
    if not owner or not repo:
        raise SystemExit("Cannot detect owner/repo. Pass --owner and --repo explicitly.")

    run_id = int(args.run_id)
    out_root = Path(args.output_dir).resolve() / f"run_{run_id}"
    artifacts_root = out_root / "downloaded_artifacts"
    zips_root = out_root / "artifact_zips"
    out_root.mkdir(parents=True, exist_ok=True)

    print(f"Fetching artifacts: owner={owner} repo={repo} run_id={run_id}")
    artifacts = _list_run_artifacts(owner, repo, run_id, token)
    if not artifacts:
        raise SystemExit(f"No artifacts found for run_id={run_id}.")

    selected = _select_relevant_artifacts(artifacts)
    print(f"Found artifacts={len(artifacts)} selected={len(selected)}")

    artifact_dirs: List[Tuple[str, Path]] = []
    for i, art in enumerate(selected, start=1):
        name = art.get("name") or f"artifact_{art.get('id')}"
        art_id = int(art["id"])
        zip_name = f"{i:04d}_{_sanitize_name(name)}_{art_id}.zip"
        zip_path = zips_root / zip_name
        extract_dir = artifacts_root / f"{i:04d}_{_sanitize_name(name)}"

        print(f"[{i}/{len(selected)}] Download {name} (id={art_id})")
        _download_artifact_zip(owner, repo, art_id, token, zip_path)
        _extract_zip(zip_path, extract_dir)
        artifact_dirs.append((name, extract_dir))

        if not args.keep_zips and zip_path.exists():
            zip_path.unlink()

    final_csv_out = out_root / "50_batch_improved_from_run.csv"
    final_compare_out = out_root / "50_batch_improved_compare_from_run.csv"
    compact_summary_out = out_root / "improved_solutions_summary.csv"

    final_csv_candidates: List[Path] = []
    final_compare_candidates: List[Path] = []
    for name, root in artifact_dirs:
        if name.startswith("repair-improve-50-batch-final-"):
            final_csv_candidates.extend(sorted(root.rglob("50_batch_improved.csv")))
            final_compare_candidates.extend(sorted(root.rglob("50_batch_improved_compare.csv")))

    meta: Dict[str, Any] = {
        "owner": owner,
        "repo": repo,
        "run_id": run_id,
        "artifacts_total": len(artifacts),
        "artifacts_selected": len(selected),
        "final_csv": str(final_csv_out),
        "final_compare_csv": str(final_compare_out),
        "compact_summary_csv": str(compact_summary_out),
    }

    used_final_artifact = False
    if final_csv_candidates:
        source_final = final_csv_candidates[0]
        _copy_if_exists(source_final, final_csv_out)
        if final_compare_candidates:
            _copy_if_exists(final_compare_candidates[0], final_compare_out)
        used_final_artifact = True
        meta["mode"] = "use_final_artifact"
        meta["source_final_csv"] = str(source_final)
    else:
        rebuild_meta = _merge_from_stage1_and_patches(
            artifact_dirs=artifact_dirs,
            out_improved_csv=final_csv_out,
            out_compare_csv=final_compare_out,
        )
        meta.update(rebuild_meta)

    summary_meta = _build_compact_summary(
        improved_csv=final_csv_out,
        compare_csv=final_compare_out if final_compare_out.exists() else None,
        out_summary_csv=compact_summary_out,
    )
    meta.update(summary_meta)
    meta["used_final_artifact"] = used_final_artifact

    meta_path = out_root / "summary_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")

    print(f"Done. Output directory: {out_root}")
    print(f"- improved_csv: {final_csv_out}")
    print(f"- compare_csv: {final_compare_out if final_compare_out.exists() else '(not found)'}")
    print(f"- compact_summary_csv: {compact_summary_out}")
    print(f"- meta_json: {meta_path}")


if __name__ == "__main__":
    main()
