"""Play a bound dexterous-hand task through UniLab's native Viser loop.

This is the same run_viser_playback_from_cfg entrypoint used by UniLab's own
play_viser script. MuJoCo is the physics owner; Viser reads its snapshots.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "third_party/UniLab/src")]

import numpy as np
import torch
from hydra.utils import instantiate
from omegaconf import OmegaConf

from integrations.unilab.task_owner import ROOT, TASK_IDS, bound_profile, load_owner
from integrations.unilab.run_grasp_scene import environment_config, probe_action
from integrations.unilab.run_insert_wuji_scene import (
    environment_config as insertion_environment_config, validate_scene as validate_insertion_scene,
)
from integrations.unilab.build_allegro_robot import load_robot
from unilab.envs import make_manager_based_rl_env
from unilab.visualization.viser_playback import run_viser_playback_from_cfg


class GraspPlaybackSession:
    """UniLab PlaybackSession for a fixed scripted grasp or articulation probe."""

    def __init__(self, env, profile):
        self.env = env
        self.profile = profile
        self.robot = load_robot(profile)
        self.step_count = 0
        self.home = None
        self.controller = None

    def reset(self):
        obs, _info = self.env.reset()
        self.step_count = 0
        joints = self.env.scene.read_plan.joint_tensor_view(self.env.scene["robot"])
        self.home = joints.joint_pos.cpu().numpy().copy()
        if self.profile["runtime"]["controller"] == "ik_grasp":
            from integrations.unilab.grasp_ik import GraspIKController

            body = self.env.scene.read_plan.body_tensor_view(self.env.scene["objects"], ("object",))
            pose = torch.cat((body.pos_w, body.quat_w), -1)[:, 0].cpu().numpy().copy()
            self.controller = GraspIKController(
                ROOT / self.profile["scene"], self.robot, self.home, pose,
                self.profile["scripted_grasp"])
        return obs

    def advance(self, controls) -> bool:
        if not controls.consume_step_permission():
            return False
        if self.home is None:
            raise RuntimeError("Playback session must reset before advancing")
        if self.step_count >= self.profile["runtime"]["steps"]:
            self.reset()
            return True
        self.step_count += 1
        mode = self.profile["runtime"]["controller"]
        if self.controller is not None:
            t = self.step_count * self.profile["runtime"]["ctrl_dt"]
            action, _phase = self.controller.action(t)
        elif mode == "probe":
            amplitude = np.sin(np.pi * self.step_count / self.profile["runtime"]["steps"])
            action = probe_action(self.robot, self.env.num_envs, amplitude)
        else:
            action = np.zeros_like(self.home)
        result = self.env.step(torch.as_tensor(action, device=self.env.device))
        if result.terminated.any() or result.truncated.any():
            self.reset()
        return True

    def physics_state(self):
        return self.env.get_physics_state_snapshot()

    @property
    def info(self):
        state = self.env.state
        value = getattr(state, "info", None) if state is not None else None
        return value if isinstance(value, dict) else {}


class InsertionDiagnosticPlaybackSession:
    """Native UniLab Viser playback for I01/I02 scene diagnostics."""

    def __init__(self, env, profile, robot):
        self.env = env
        self.profile = profile
        self.robot = robot
        self.step_count = 0
        self.joint_index = {name: index for index, name in enumerate(robot["joint_names"])}

    def reset(self):
        obs, _info = self.env.reset()
        self.step_count = 0
        return obs

    def advance(self, controls) -> bool:
        if not controls.consume_step_permission():
            return False
        if self.step_count >= self.profile["steps"]:
            self.reset()
            return True
        self.step_count += 1
        action = torch.zeros((self.env.num_envs, len(self.joint_index)), device=self.env.device)
        amplitude = np.sin(np.pi * self.step_count / (self.profile["steps"] + 1)) ** 2
        for name, offset in self.robot["probe_joint_offsets"].items():
            action[0, self.joint_index[name]] = offset * amplitude
        result = self.env.step(action)
        if result.terminated.any() or result.truncated.any():
            self.reset()
        return True

    def physics_state(self):
        return self.env.get_physics_state_snapshot()

    @property
    def info(self):
        state = self.env.state
        value = getattr(state, "info", None) if state is not None else None
        return value if isinstance(value, dict) else {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True, choices=TASK_IDS)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    cfg, _task = load_owner(args.task)
    if cfg.task_catalog.binding.kind == "unilab_scene_diagnostic":
        import json

        profile = json.loads((ROOT / str(cfg.task_catalog.binding.profile)).read_text())
        validate_insertion_scene(profile)
        env_cfg, robot = insertion_environment_config(profile)
        num_envs = profile["num_envs"]
        session_type = InsertionDiagnosticPlaybackSession
    else:
        profile = bound_profile(cfg)
        env_cfg = environment_config(profile)
        num_envs = profile["runtime"]["num_envs"]
        session_type = GraspPlaybackSession
    env = make_manager_based_rl_env(
        instantiate(env_cfg, _convert_="all"),
        num_envs=num_envs,
        backend_type=profile["backend"],
    )
    try:
        playback_cfg = OmegaConf.create({
            "viser": {
                "port": args.port,
                "max_envs": int(cfg.viewer.max_envs),
                "env_idx": 0,
                "display_mode": str(cfg.viewer.display_mode),
            },
            "training": {"render_spacing": 1.0},
        })
        print(f"UniLab Viser task {args.task}: http://localhost:{args.port}", flush=True)
        print("Playback shows a scripted IK attempt or diagnostic probe, not a trained policy.", flush=True)
        run_viser_playback_from_cfg(
            (session_type(env, profile, robot) if session_type is InsertionDiagnosticPlaybackSession
             else session_type(env, profile)), playback_cfg,
            entrypoint="dexhand_bench play_task",
        )
    finally:
        env.close()


if __name__ == "__main__":
    main()
