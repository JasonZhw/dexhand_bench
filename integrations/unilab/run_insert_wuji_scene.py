"""Run I01/I02 as UniLab scene-and-signal diagnostics, without a policy."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "third_party/UniLab/src")]

import torch
from hydra import compose, initialize_config_dir
from hydra.utils import instantiate
from omegaconf import OmegaConf
from unilab.envs import make_manager_based_rl_env

from dexbench.evaluation import evaluate
from integrations.unilab.insert_geometry import source_geometry_signals
from integrations.unilab.object_scene_terms import source_boxes


WRISTS = ("left_wuji_l_wrist", "right_wuji_r_wrist")
OBJECTS = {"I01": ("tube0", "tube1", "tube2", "slot"),
           "I02": ("charger", "socket")}


def environment_config(profile):
    if profile["task_id"] not in OBJECTS or profile["backend"] != "mujoco" or profile["controller"] != "probe":
        raise ValueError("Only I01/I02 Wuji/MuJoCo probe bindings are implemented")
    if profile["num_envs"] < 2 or profile["sim_dt"] != profile["ctrl_dt"]:
        raise ValueError("Diagnostic requires two environments and per-step observation")
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "conf/unilab")):
        cfg = compose(config_name="x5_control_diagnostic")
    OmegaConf.set_struct(cfg, False)
    robot = json.loads((ROOT / profile["robot_file"]).read_text())
    if robot["id"] != "dual_x5_wuji":
        raise ValueError("Robot metadata does not describe the Wuji assembly")
    cfg.env.scene.model_file = str((ROOT / profile["scene"]).resolve())
    for key in ("root_body_name", "joint_names", "actuator_names", "body_names"):
        cfg.env.scene.entities.robot[key] = robot[key]
    names = OBJECTS[profile["task_id"]]
    cfg.env.scene.entities.objects.root_body_name = names[0]
    cfg.env.scene.entities.objects.body_names = list(names)
    cfg.env.observations.policy.terms.object_pose.params.body_names = list(names)
    if profile["task_id"] == "I02":
        del cfg.env.observations.policy.terms["source_geometry"]
        del cfg.env.metrics["source_geometry_count"]
        height = cfg.env.metrics.pop("mean_tube_height_m")
        height.params.body_names = ["charger"]
        cfg.env.metrics.mean_charger_height_m = height
    cfg.env.observations.policy.terms.hand_pose = {
        "_target_": "unilab.managers.ObservationTermCfg",
        "func": {"_target_": "hydra.utils.get_class",
                 "path": "integrations.unilab.object_scene_terms.ObjectPose"},
        "params": {"entity_name": "robot", "body_names": list(WRISTS)},
    }
    cfg.env.sim_dt = profile["sim_dt"]
    cfg.env.ctrl_dt = profile["ctrl_dt"]
    cfg.env.max_episode_seconds = json.loads((ROOT / profile["task_file"]).read_text())["benchmark"]["episode"]["timeout_s"]
    return cfg.env, robot


def validate_scene(profile):
    scene = ROOT / profile["scene"]
    receipt = json.loads((scene.parent / "scene.json").read_text())
    if receipt["task_id"] != profile["task_id"] or receipt["robot"] != "dual_x5_wuji":
        raise ValueError("Scene receipt belongs to another task or robot")
    checks = ((scene, receipt["scene_sha256"]),
              (ROOT / "assets/generated/dual_x5_wuji/scene.xml", receipt["robot_template_sha256"]),
              (ROOT / "assets/generated/dual_x5_wuji/robot.json", receipt["robot_metadata_sha256"]))
    if profile["task_id"] == "I01":
        checks += ((ROOT / "assets/generated/I01-convex/scene.xml", receipt["object_scene_sha256"]),)
    else:
        checks += ((ROOT / "asset_manifests/I02-robodojo.json", receipt["source_asset_manifest_sha256"]),
                   (ROOT / "profiles/layouts/I02-fixed-diagnostic.json", receipt["layout_sha256"]))
    for path, digest in checks:
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Scene source changed: {path}")
    for name, digest in receipt["mesh_dependencies"].items():
        path = Path(name)
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Scene mesh changed: {path}")
    for name, digest in receipt.get("source_asset_dependencies", {}).items():
        path = Path(name)
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Source asset changed: {path}")
    return scene, receipt


def i02_source_boxes(device):
    manifest = json.loads((ROOT / "asset_manifests/I02-robodojo.json").read_text())
    boxes = {}
    for entry in manifest["files"]:
        path = ROOT / entry["local_path"]
        if path.name != "metadata.json":
            continue
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError(f"I02 metadata changed: {path}")
        boxes[path.parent.parent.name] = torch.tensor(
            json.loads(path.read_text())["geometry"]["oriented_bbox"]["vertices"],
            dtype=torch.float64, device=device)
    return boxes["charger"], boxes["socket"]


def home_predicate(current_pos, current_quat, origin_pos, origin_quat):
    """Adapt source's per-axis 0.15 m and quaternion-angle 20 degree checks."""
    per_axis = (current_pos - origin_pos).abs().le(.15).all(dim=-1)
    dot = (current_quat * origin_quat).sum(dim=-1).abs().clamp(max=1)
    angle_deg = torch.rad2deg(2 * torch.acos(dot))
    return (per_axis & angle_deg.le(20)).all(dim=-1)


def run(profile, output, steps=None):
    env_cfg, robot = environment_config(profile)
    scene, scene_receipt = validate_scene(profile)
    task = json.loads((ROOT / profile["task_file"]).read_text())
    steps = profile["steps"] if steps is None else steps
    if not isinstance(steps, int) or steps < 1 or steps * profile["ctrl_dt"] >= env_cfg.max_episode_seconds:
        raise ValueError("Diagnostic steps must be positive and shorter than task timeout")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "config.json").write_text(json.dumps({
        "profile": profile, "env": OmegaConf.to_container(env_cfg, resolve=True)}, indent=2) + "\n")
    env = make_manager_based_rl_env(instantiate(env_cfg, _convert_="all"),
                                    num_envs=profile["num_envs"], backend_type="mujoco")
    task_id = profile["task_id"]
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
            raise ValueError("UniLab action dimension differs from Wuji actuator list")
        tube_bbox, rack_bbox = (source_boxes(env.device) if task_id == "I01"
                                else i02_source_boxes(env.device))

        def body_views():
            objects = env.scene.read_plan.body_tensor_view(env.scene["objects"], OBJECTS[task_id])
            hands = env.scene.read_plan.body_tensor_view(env.scene["robot"], WRISTS)
            return objects, hands

        _objects, hands = body_views()
        origin_pos, origin_quat = hands.pos_w.clone(), hands.quat_w.clone()

        def capture(step, action, step_time_s):
            objects, hands = body_views()
            if task_id == "I01":
                positions, quaternions = objects.pos_w, objects.quat_w
            else:
                positions = torch.cat((objects.pos_w[:, :1].expand(-1, 3, -1),
                                       objects.pos_w[:, 1:]), dim=1)
                quaternions = torch.cat((objects.quat_w[:, :1].expand(-1, 3, -1),
                                         objects.quat_w[:, 1:]), dim=1)
            measured = source_geometry_signals(positions, quaternions, tube_bbox, rack_bbox)
            home = home_predicate(hands.pos_w, hands.quat_w, origin_pos, origin_quat)
            if not torch.isfinite(measured).all() or not torch.isfinite(objects.pos_w).all():
                raise ValueError("Nonfinite I01 simulation state")
            for env_id in range(env.num_envs):
                values = {}
                if task_id == "I01":
                    for tube in range(3):
                        values[f"tube{tube}_depth_m"] = float(measured[env_id, tube, 0])
                        values[f"tube{tube}_inside"] = bool(measured[env_id, tube, 1])
                        values[f"tube{tube}_upright_deg"] = float(measured[env_id, tube, 2])
                else:
                    values["charger_depth_m"] = float(measured[env_id, 0, 0])
                    values["charger_inside"] = bool(measured[env_id, 0, 1])
                    values["charger_upright_deg"] = float(measured[env_id, 0, 2])
                values["robots_home"] = bool(home[env_id])
                traces[env_id]["frames"].append({"t_s": step * env_cfg.ctrl_dt, "signals": values})
                records.append({"step": step, "env_id": env_id, "t_s": step * env_cfg.ctrl_dt,
                                "object_pos_w": objects.pos_w[env_id].tolist(),
                                "object_quat_wxyz": objects.quat_w[env_id].tolist(),
                                "wrist_pos_w": hands.pos_w[env_id].tolist(),
                                "joint_action_offset": action[env_id].tolist(),
                                "step_time_s": step_time_s,
                                "contact_force_n": None, "contact_force_status": "not_instrumented"})

        action = torch.zeros((env.num_envs, len(robot["joint_names"])), device=env.device)
        capture(0, action, None)
        for step in range(1, steps + 1):
            action.zero_()
            envelope = math.sin(math.pi * step / (steps + 1)) ** 2
            for name, offset in robot["probe_joint_offsets"].items():
                action[0, robot["joint_names"].index(name)] = offset * envelope
            start = time.perf_counter()
            state = env.step(action)
            elapsed = time.perf_counter() - start
            if state.terminated.any() or state.truncated.any():
                raise ValueError(f"{task_id} diagnostic ended before scheduled steps")
            capture(step, action, elapsed)
        before_reset = state.obs["obs"].clone()
        other_before = before_reset[1:].clone()
        after, _ = env.reset(torch.tensor([0], device=env.device))
        torch.testing.assert_close(after["obs"][0], initial_obs[0], atol=1e-6, rtol=0)
        torch.testing.assert_close(after["obs"][1:], other_before, atol=0, rtol=0)
        independent_reset_verified = True
    finally:
        env.close()
    scores = [evaluate(task_id, task["benchmark"], trace) for trace in traces]
    (output / "traces.json").write_text(json.dumps(traces) + "\n")
    (output / "states.json").write_text(json.dumps(records) + "\n")
    (output / "evaluation.json").write_text(json.dumps(scores, indent=2) + "\n")
    receipt = {"task_id": task_id, "scope": "unilab_scene_diagnostic", "backend": "mujoco",
               "controller": "probe", "steps": steps, "num_envs": profile["num_envs"],
               "task_success": None, "independent_reset_verified": independent_reset_verified,
               "physics_force_available": False, "scene_sha256": scene_receipt["scene_sha256"],
               "score_signal_count": len(task["benchmark"]["signals"]),
               "source_score_ported": False, "limitations": profile["limitations"]}
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt
