"""Inspect or fetch pinned source assets for one task without changing its binding."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from dexbench.assets import fetch_asset, verify_asset
from integrations.unilab.task_owner import TASK_IDS, load_owner


def source_asset_entries(task_id):
    _owner, task = load_owner(task_id)
    name = task["benchmark"]["scene"].get("asset_manifest")
    if not name:
        return None, []
    manifest = json.loads((ROOT / name).read_text())
    entries = manifest.get("files")
    if not isinstance(entries, list):
        raise ValueError(f"{task_id}: source manifest has no individually pinned downloadable files")
    if any(not all(entry.get(k) for k in ("url", "local_path", "sha256")) for entry in entries):
        raise ValueError(f"{task_id}: source asset has no URL, local path, or checksum")
    return name, entries


def audit(task_id):
    manifest, entries = source_asset_entries(task_id)
    return {"task_id": task_id, "manifest": manifest, "file_count": len(entries),
            "files": [verify_asset(entry, ROOT) for entry in entries]}


def fetch(task_id, *, max_total_bytes=25_000_000):
    manifest, entries = source_asset_entries(task_id)
    if manifest is None:
        raise ValueError(f"{task_id}: no pinned source asset manifest")
    missing = [entry for entry in entries if verify_asset(entry, ROOT)["status"] == "missing"]
    damaged = [entry for entry in entries if verify_asset(entry, ROOT)["status"] not in ("missing", "verified")]
    if damaged:
        raise ValueError(f"{task_id}: existing unverified files require manual inspection")
    if any(entry.get("size_bytes") is None for entry in missing):
        raise ValueError(f"{task_id}: missing size metadata; cannot enforce total download budget")
    total = sum(entry["size_bytes"] for entry in missing)
    if total > max_total_bytes:
        raise ValueError(f"{task_id}: {total} bytes exceeds {max_total_bytes} byte budget")
    for entry in missing:
        fetch_asset(entry, ROOT)
    return audit(task_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("audit", "fetch"))
    parser.add_argument("--task", choices=TASK_IDS, required=True)
    parser.add_argument("--max-total-mb", type=float, default=25)
    args = parser.parse_args()
    result = (audit(args.task) if args.command == "audit" else
              fetch(args.task, max_total_bytes=int(args.max_total_mb * 1_000_000)))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
