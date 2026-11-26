"""Compose and audit all benchmark task owners; run bound scenes through UniLab.

The task JSON is the scoring contract. A task owner only binds that contract
to a verified scene, robot, controller and backend. Unbound owners are
inspectable but deliberately cannot start a simulation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from hydra import compose, initialize_config_dir

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "third_party/UniLab/src")]
from dexbench.evaluation import validate_spec

TASK_IDS = tuple(sorted(path.stem for path in (ROOT / "tasks").rglob("*.json")))


def load_owner(task_id: str):
    if task_id not in TASK_IDS:
        raise ValueError(f"Unknown task: {task_id}")
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "conf/unilab")):
        cfg = compose(config_name="dex_task", overrides=[f"task_catalog={task_id}"])
    entry = cfg.task_catalog
    contract_path = ROOT / str(entry.contract)
    if not contract_path.is_file() or contract_path.resolve().is_relative_to((ROOT / "tasks").resolve()) is False:
        raise ValueError(f"Invalid contract path for {task_id}: {entry.contract}")
    task = json.loads(contract_path.read_text(encoding="utf-8"))
    if entry.id != task_id or task.get("id") != task_id:
        raise ValueError(f"Task/owner ID mismatch for {task_id}")
    errors = validate_spec(task.get("benchmark"))
    if errors:
        raise ValueError(f"{task_id}: invalid scoring contract: {'; '.join(errors)}")
    binding = entry.binding
    if binding.kind not in ("local_grasp", "unilab_scene_diagnostic", "unbound"):
        raise ValueError(f"{task_id}: unknown scene binding {binding.kind}")
    if binding.kind == "local_grasp":
        if task_id not in ("G01", "G02", "G03", "G04", "G05"):
            raise ValueError(f"{task_id}: no local grasp implementation")
        if binding.backend != "mujoco" or binding.robot not in ("allegro", "wuji", "sharpa"):
            raise ValueError(f"{task_id}: unsupported robot/backend binding")
        if binding.controller not in ("probe", "hold", "ik_grasp"):
            raise ValueError(f"{task_id}: unsupported controller")
        if binding.controller == "ik_grasp" and not binding.controller_config:
            raise ValueError(f"{task_id}: IK controller configuration missing")
    elif binding.kind == "unilab_scene_diagnostic":
        if task_id not in ("I01", "I02") or tuple(str(binding[key]) for key in ("profile", "robot", "backend", "controller")) != (
                f"profiles/{task_id}-wuji.json", "wuji", "mujoco", "probe"):
            raise ValueError("Only I01/I02 Wuji scene diagnostics are implemented")
        if binding.controller_config is not None:
            raise ValueError("I01/I02 probes do not use controller configurations")
    elif any(binding[key] is not None for key in ("profile", "robot", "backend", "controller", "controller_config")):
        raise ValueError(f"{task_id}: unbound task must not claim a physical implementation")
    return cfg, task


def bound_profile(cfg):
    """Resolve a local scene only after its owner has passed validation."""
    binding = cfg.task_catalog.binding
    if binding.kind != "local_grasp":
        raise ValueError(f"{cfg.task_catalog.id}: no UniLab scene binding yet")
    from integrations.unilab.grasp_profiles import load_profile

    profile = load_profile(str(cfg.task_catalog.id), str(binding.robot))
    if Path(profile["task_file"]) != Path(str(cfg.task_catalog.contract)):
        raise ValueError("Owner/profile task contract mismatch")
    if profile["backend"] != binding.backend:
        raise ValueError("Owner/profile backend mismatch")
    if str(cfg.task_catalog.binding.profile) != f"profiles/local_grasp/{cfg.task_catalog.id}.json":
        raise ValueError("Owner/profile file mismatch")
    profile["runtime"]["controller"] = str(binding.controller)
    if binding.controller_config:
        config_path = ROOT / str(binding.controller_config)
        profile["scripted_grasp"] = json.loads(config_path.read_text())
        import math

        duration = sum(profile["scripted_grasp"]["phases_s"].values())
        profile["runtime"]["steps"] = math.ceil(duration / profile["runtime"]["ctrl_dt"])
    return profile


def audit_task(task_id: str) -> dict:
    cfg, task = load_owner(task_id)
    binding = cfg.task_catalog.binding
    manifest = task["benchmark"]["scene"].get("asset_manifest")
    row = {
        "task_id": task_id,
        "name": task["name"],
        "category": task["category"],
        "scoring_contract": "valid_draft",
        "owner_config": f"conf/unilab/task_catalog/{task_id}.yaml",
        "scene_binding": str(binding.kind),
        "backend": str(binding.backend) if binding.backend else None,
        "controller": str(binding.controller) if binding.controller else None,
        "source_asset_manifest": manifest,
        "source_manifest_present": (ROOT / manifest).is_file() if manifest else None,
        "physical_load": "not_attempted",
        "source_runtime_evidence": bool(
            task.get("smoke_evidence", {}).get("receipt")
            and (ROOT / task["smoke_evidence"]["receipt"]).is_file()
        ) if isinstance(task.get("smoke_evidence"), dict) else False,
        "score_signal_count": len(task["benchmark"]["signals"]),
    }
    if binding.kind == "local_grasp":
        profile = bound_profile(cfg)
        scene = ROOT / profile["scene"]
        row["local_scene_present"] = scene.is_file()
        row["local_scene_receipt_present"] = (scene.parent / "scene.json").is_file()
        row["robot"] = str(binding.robot)
        row["next_blocker"] = None if row["local_scene_present"] else "build_local_scene"
    elif binding.kind == "unilab_scene_diagnostic":
        profile = json.loads((ROOT / str(binding.profile)).read_text())
        if profile["task_id"] != task_id or profile["backend"] != binding.backend:
            raise ValueError("Diagnostic profile differs from its owner")
        scene = ROOT / profile["scene"]
        row["local_scene_present"] = scene.is_file()
        row["local_scene_receipt_present"] = (scene.parent / "scene.json").is_file()
        row["robot"] = str(binding.robot)
        evidence = ROOT / str(cfg.task_catalog.evidence)
        if scene.is_file() and (scene.parent / "scene.json").is_file() and evidence.is_file():
            receipt = json.loads(evidence.read_text())
            if (receipt.get("task_id") == task_id
                    and receipt.get("scope") == "unilab_scene_diagnostic"
                    and receipt.get("independent_reset_verified") is True
                    and receipt.get("scene_sha256") == json.loads((scene.parent / "scene.json").read_text())["scene_sha256"]):
                row["physical_load"] = "smoke_verified"
        row["local_runtime_receipt"] = str(cfg.task_catalog.evidence)
        row["next_blocker"] = "insertion_controller_and_source_parity"
    else:
        row["local_scene_present"] = False
        row["local_scene_receipt_present"] = False
        row["robot"] = None
        row["next_blocker"] = "bind_assets_robot_reset_observations_and_controller_in_unilab"
    if manifest:
        asset_manifest = json.loads((ROOT / manifest).read_text())
        entries = asset_manifest.get("files")
        if isinstance(entries, list):
            from dexbench.assets import verify_asset

            statuses = [verify_asset(entry, ROOT)["status"] for entry in entries
                        if entry.get("local_path")]
            row["source_assets_declared"] = len(entries)
            row["source_assets_verified"] = statuses.count("verified")
            row["source_assets_missing_or_changed"] = len(statuses) - statuses.count("verified")
        else:
            row["source_assets_declared"] = None
            row["source_assets_verified"] = None
            row["source_assets_missing_or_changed"] = None
    return row


def smoke_task(task_id: str, output: Path, steps: int = 2) -> dict:
    if steps < 1:
        raise ValueError("steps must be positive")
    cfg, _ = load_owner(task_id)
    if cfg.task_catalog.binding.kind == "unilab_scene_diagnostic":
        from integrations.unilab.run_insert_wuji_scene import run

        profile = json.loads((ROOT / str(cfg.task_catalog.binding.profile)).read_text())
        return run(profile, output, steps=steps)
    profile = bound_profile(cfg)
    profile["runtime"]["steps"] = steps
    profile.setdefault("physics_recording", {})["enabled"] = False
    profile["physics_recording"]["warmup_steps"] = 0
    from integrations.unilab.run_grasp_scene import run

    return run(profile, output, video=False, live=False)


def run_task(task_id: str, output: Path) -> dict:
    """Run the configured controller for its full local episode and score it."""
    cfg, _ = load_owner(task_id)
    if cfg.task_catalog.binding.kind == "unilab_scene_diagnostic":
        from integrations.unilab.run_insert_wuji_scene import run

        profile = json.loads((ROOT / str(cfg.task_catalog.binding.profile)).read_text())
        return run(profile, output)
    profile = bound_profile(cfg)
    from integrations.unilab.run_grasp_scene import run

    return run(profile, output, video=False, live=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("audit", "smoke", "run"))
    parser.add_argument("--task", choices=TASK_IDS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--steps", type=int, default=2)
    args = parser.parse_args()
    if args.command == "audit":
        rows = [audit_task(task_id) for task_id in (args.task,) if task_id] if args.task else [audit_task(task_id) for task_id in TASK_IDS]
        print(json.dumps({"task_count": len(rows), "tasks": rows}, ensure_ascii=False, indent=2))
        return 0
    if not args.task or not args.output:
        parser.error(f"{args.command} requires --task and --output")
    result = (smoke_task(args.task, args.output, args.steps) if args.command == "smoke"
              else run_task(args.task, args.output))
    print(json.dumps({key: result[key] for key in (
        "task_id", "scope", "controller", "task_success", "backend",
        "steps", "independent_reset_verified", "physics_force_available")},
        ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
