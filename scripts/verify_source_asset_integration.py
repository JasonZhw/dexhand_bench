"""Rebuild and smoke-test pinned source-object diagnostics after robot setup.

This explicit integration check is intentionally outside the default unit suite:
it requires downloaded RoboDojo USD assets, a Wuji/X5 robot, and USD/CoACD.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from integrations.unilab.run_proxy_task_scene import validate_scene
from integrations.unilab.task_assets import audit, fetch
from integrations.unilab.task_owner import smoke_task


def verify(task_id: str, fetch_missing: bool) -> None:
    if fetch_missing:
        fetch(task_id)
    status = audit(task_id)
    bad = [entry for entry in status["files"] if entry["status"] != "verified"]
    if bad:
        raise ValueError(f"{task_id}: {len(bad)} source files missing or changed; run with --fetch")
    scene = ROOT / f"assets/generated/source_tasks/{task_id}/scene.xml"
    if not scene.is_file():
        subprocess.run([str(ROOT / ".venv-assets/bin/python"),
                        str(ROOT / "integrations/unilab/build_source_asset_scenes.py"),
                        "--task", task_id], cwd=ROOT, check=True)
    validate_scene(task_id)
    (ROOT / "runs").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"{task_id}-source-check-", dir=ROOT / "runs") as temporary:
        receipt = smoke_task(task_id, Path(temporary) / "smoke", steps=2)
        if (receipt["task_success"] is not None
                or receipt["independent_reset_verified"] is not True
                or receipt["scope"] != "unilab_source_asset_diagnostic"):
            raise ValueError(f"{task_id}: source-object diagnostic did not pass smoke check")
    print(f"{task_id}: pinned assets, scene integrity, and UniLab smoke verified")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("P01", "I03", "all"), default="all")
    parser.add_argument("--fetch", action="store_true", help="Download missing pinned USD assets")
    args = parser.parse_args()
    for task_id in (("P01", "I03") if args.task == "all" else (args.task,)):
        verify(task_id, args.fetch)


if __name__ == "__main__":
    main()
