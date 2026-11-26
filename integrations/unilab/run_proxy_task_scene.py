"""Exercise proxy or source-object diagnostics through UniLab without claiming task success."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "third_party/UniLab/src")]

import torch
from hydra import compose, initialize_config_dir
from hydra.utils import instantiate
from omegaconf import OmegaConf
from unilab.envs import make_manager_based_rl_env

from dexbench.evaluation import evaluate
from integrations.unilab.build_proxy_task_scenes import LAYOUTS, OUTPUT_ROOT, ROBOT, digest, layout_digest

WRISTS = ("left_wuji_l_wrist", "right_wuji_r_wrist")
ROBOT_META = ROOT / "assets/generated/dual_x5_wuji/robot.json"
SOURCE_LAYOUTS = ROOT / "profiles/source_asset_scenes.json"
SOURCE_OUTPUT_ROOT = ROOT / "assets/generated/source_tasks"


def diagnostic_profile(task_id: str) -> dict:
    owner = yaml.safe_load((ROOT / f"conf/unilab/task_catalog/{task_id}.yaml").read_text())
    source = owner["binding"]["kind"] == "unilab_source_asset_diagnostic"
    layouts = json.loads((SOURCE_LAYOUTS if source else LAYOUTS).read_text())
    if task_id not in layouts:
        raise ValueError(f"No primitive diagnostic layout for {task_id}")
    layout = layouts[task_id]
    task_files = list((ROOT / "tasks").glob(f"*/{task_id}.json"))
    if len(task_files) != 1:
        raise ValueError(f"Task contract missing or ambiguous: {task_id}")
    return {
        "task_id": task_id, "task_file": str(task_files[0].relative_to(ROOT)),
        "scene": str(((SOURCE_OUTPUT_ROOT if source else OUTPUT_ROOT) / task_id / "scene.xml").relative_to(ROOT)),
        "backend": "mujoco", "controller": "probe", "robot_file": str(ROBOT_META.relative_to(ROOT)),
        "num_envs": 2, "sim_dt": layout.get("sim_dt", 0.004),
        "ctrl_dt": layout.get("ctrl_dt", 0.004), "steps": 8,
        "asset_scope": "source_object_scene_diagnostic" if source else "self_authored_geometry_proxy",
        "limitations": [
            ("Verified RoboDojo object meshes with adapted MuJoCo contacts; source reset is not ported"
             if source else "Source MJCF puck and table parameters are checked; robot, elevation, reset and score are adapted"
             if "source_mjcf" in layout else "Self-authored primitive geometry; source assets and reset are not ported"),
            "Joint probe is not a task policy",
            "Task signals have no source-calibrated adapter; success remains unknown",
            "Contact forces are not instrumented for these diagnostics",
        ],
    }


def validate_scene(task_id: str) -> tuple[Path, dict]:
    profile = diagnostic_profile(task_id)
    source = profile["asset_scope"] == "source_object_scene_diagnostic"
    scene = ROOT / profile["scene"]
    receipt_path = scene.parent / "scene.json"
    receipt = json.loads(receipt_path.read_text())
    if (receipt["task_id"] != task_id or receipt["scope"] != profile["asset_scope"]
            or receipt["backend"] != "mujoco" or receipt["robot"] != "dual_x5_wuji"):
        raise ValueError("Scene receipt does not describe this task's asset scope")
    if source:
        layouts = json.loads(SOURCE_LAYOUTS.read_text())
        if layout_digest(layouts[task_id]) != receipt["layout_sha256"]:
            raise ValueError("Changed source-object layout")
        manifest = ROOT / layouts[task_id]["manifest"]
        if digest(manifest) != receipt["source_asset_manifest_sha256"]:
            raise ValueError("Changed source asset manifest")
    else:
        layout = json.loads(LAYOUTS.read_text())[task_id]
        if layout_digest(layout) != receipt["layout_sha256"]:
            raise ValueError("Changed diagnostic task layout")
        if "source_mjcf" in layout:
            manifest = ROOT / layout["source_mjcf"]["manifest"]
            if digest(manifest) != receipt["source_mjcf"]["manifest_sha256"]:
                raise ValueError("Changed source MJCF manifest")
    checks = (
        (scene, receipt["scene_sha256"]),
        (ROBOT, receipt["robot_template_sha256"]),
        (ROBOT_META, receipt["robot_metadata_sha256"]),
    )
    for path, expected in checks:
        if digest(path) != expected:
            raise ValueError(f"Changed diagnostic scene dependency: {path}")
    for relative, expected in receipt["mesh_dependencies"].items():
        path = ROOT / relative
        if digest(path) != expected:
            raise ValueError(f"Changed scene mesh: {path}")
    for relative, expected in receipt.get("source_asset_dependencies", {}).items():
        path = ROOT / relative
        if digest(path) != expected:
            raise ValueError(f"Changed source asset: {path}")
    return scene, receipt


def environment_config(task_id: str):
    profile = diagnostic_profile(task_id)
    _scene, scene_receipt = validate_scene(task_id)
    robot = json.loads(ROBOT_META.read_text())
    task = json.loads((ROOT / profile["task_file"]).read_text())
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "conf/unilab")):
        cfg = compose(config_name="x5_control_diagnostic")
    OmegaConf.set_struct(cfg, False)
    cfg.env.scene.model_file = str((ROOT / profile["scene"]).resolve())
    for key in ("root_body_name", "joint_names", "actuator_names", "body_names"):
        cfg.env.scene.entities.robot[key] = robot[key]
    names = scene_receipt["body_names"]
    cfg.env.scene.entities.objects.root_body_name = names[0]
    cfg.env.scene.entities.objects.body_names = names
    cfg.env.observations.policy.terms.object_pose.params.body_names = names
    del cfg.env.observations.policy.terms["source_geometry"]
    del cfg.env.metrics["source_geometry_count"]
    metric = cfg.env.metrics.pop("mean_tube_height_m")
    metric.params.body_names = names
    cfg.env.metrics.mean_proxy_height_m = metric
    cfg.env.observations.policy.terms.hand_pose = {
        "_target_": "unilab.managers.ObservationTermCfg",
        "func": {"_target_": "hydra.utils.get_class",
                 "path": "integrations.unilab.object_scene_terms.ObjectPose"},
        "params": {"entity_name": "robot", "body_names": list(WRISTS)},
    }
    cfg.env.sim_dt = profile["sim_dt"]
    cfg.env.ctrl_dt = profile["ctrl_dt"]
    cfg.env.max_episode_seconds = task["benchmark"]["episode"]["timeout_s"]
    return cfg.env, robot


def run(task_id: str, output: Path, steps: int | None = None) -> dict:
    profile = diagnostic_profile(task_id)
    cfg, robot = environment_config(task_id)
    _scene, scene_receipt = validate_scene(task_id)
    task = json.loads((ROOT / profile["task_file"]).read_text())
    steps = profile["steps"] if steps is None else steps
    if not isinstance(steps, int) or steps < 1 or steps * profile["ctrl_dt"] >= cfg.max_episode_seconds:
        raise ValueError("Probe steps must be positive and below the task timeout")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "config.json").write_text(json.dumps({
        "profile": profile, "env": OmegaConf.to_container(cfg, resolve=True),
    }, indent=2) + "\n")
    env = make_manager_based_rl_env(instantiate(cfg, _convert_="all"),
                                    num_envs=profile["num_envs"], backend_type="mujoco")
    names = scene_receipt["body_names"]
    traces = [{"task_id": task_id, "protocol_version": task["benchmark"]["protocol_version"],
               "kind": "recorded", "complete": False, "frames": []}
              for _ in range(env.num_envs)]
    records = []
    independent_reset_verified = False
    try:
        env.reset()
        initial, _ = env.reset()
        initial_obs = initial["obs"].clone()
        if env.action_manager.total_action_dim != len(robot["joint_names"]):
            raise ValueError("UniLab action dimension differs from Wuji joint list")

        def capture(step: int, action: torch.Tensor, elapsed: float | None):
            objects = env.scene.read_plan.body_tensor_view(env.scene["objects"], names)
            wrists = env.scene.read_plan.body_tensor_view(env.scene["robot"], WRISTS)
            joints = env.scene.read_plan.joint_tensor_view(env.scene["robot"])
            for value in (objects.pos_w, objects.quat_w, objects.lin_vel_w,
                          wrists.pos_w, joints.joint_pos):
                if not torch.isfinite(value).all():
                    raise ValueError(f"Nonfinite diagnostic state in {task_id}")
            for env_id in range(env.num_envs):
                # A proxy scene cannot supply source-calibrated task predicates.
                # Explicit nulls keep the evaluator's success result unknown.
                values = {name: None for name in task["benchmark"]["signals"]}
                traces[env_id]["frames"].append({"t_s": step * profile["ctrl_dt"],
                                                   "signals": values})
                records.append({
                    "step": step, "env_id": env_id, "t_s": step * profile["ctrl_dt"],
                    "body_names": names, "object_pos_w": objects.pos_w[env_id].tolist(),
                    "object_quat_wxyz": objects.quat_w[env_id].tolist(),
                    "object_lin_vel_w": objects.lin_vel_w[env_id].tolist(),
                    "wrist_pos_w": wrists.pos_w[env_id].tolist(),
                    "joint_pos": joints.joint_pos[env_id].tolist(),
                    "joint_action_offset": action[env_id].tolist(),
                    "step_time_s": elapsed, "contact_force_n": None,
                    "contact_force_status": "not_instrumented",
                })

        action = torch.zeros((env.num_envs, len(robot["joint_names"])), device=env.device)
        capture(0, action, None)
        for step in range(1, steps + 1):
            action.zero_()
            amplitude = math.sin(math.pi * step / (steps + 1)) ** 2
            for name, offset in robot["probe_joint_offsets"].items():
                action[0, robot["joint_names"].index(name)] = offset * amplitude
            started = time.perf_counter()
            state = env.step(action)
            elapsed = time.perf_counter() - started
            if state.terminated.any() or state.truncated.any():
                raise ValueError(f"{task_id} diagnostic ended before scheduled steps")
            capture(step, action, elapsed)
        other_before = state.obs["obs"][1:].clone()
        after, _ = env.reset(torch.tensor([0], device=env.device))
        torch.testing.assert_close(after["obs"][0], initial_obs[0], atol=1e-6, rtol=0)
        torch.testing.assert_close(after["obs"][1:], other_before, atol=0, rtol=0)
        independent_reset_verified = True
    finally:
        env.close()
    scores = [evaluate(task_id, task["benchmark"], trace) for trace in traces]
    if any(score["success"] is not None for score in scores):
        raise ValueError("A scene probe without task signals must not claim task success or failure")
    (output / "traces.json").write_text(json.dumps(traces) + "\n")
    (output / "states.json").write_text(json.dumps(records) + "\n")
    (output / "evaluation.json").write_text(json.dumps(scores, indent=2) + "\n")
    receipt = {
        "task_id": task_id, "scope": ("unilab_source_asset_diagnostic"
                                      if profile["asset_scope"] == "source_object_scene_diagnostic"
                                      else "unilab_proxy_diagnostic"), "backend": "mujoco",
        "controller": "probe", "steps": steps, "num_envs": profile["num_envs"],
        "task_success": None, "independent_reset_verified": independent_reset_verified,
        "physics_force_available": False, "scene_sha256": scene_receipt["scene_sha256"],
        "score_signal_count": len(task["benchmark"]["signals"]),
        "score_signals_available": 0, "source_score_ported": False,
        "limitations": profile["limitations"],
    }
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt
