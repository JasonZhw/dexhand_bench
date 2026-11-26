"""Verify actual UniLab joint actions with source X5 robots; no task policy."""
import hashlib
import json
import math
from pathlib import Path
import sys
from importlib.metadata import version

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/UniLab/src')]
import hydra
from hydra.utils import instantiate
from omegaconf import OmegaConf
import torch
from unilab.envs import make_manager_based_rl_env


@hydra.main(version_base=None, config_path='../../conf/unilab', config_name='x5_control_diagnostic')
def main(cfg):
    if cfg.scope != 'x5_control_diagnostic' or cfg.backend != 'mujoco':
        raise ValueError('Only the MuJoCo X5 control diagnostic is supported')
    if cfg.num_envs != 2 or cfg.steps < 100:
        raise ValueError('Acceptance uses two environments and at least 100 steps')
    if cfg.steps * cfg.env.ctrl_dt >= cfg.env.max_episode_seconds:
        raise ValueError('Control diagnostic must finish before episode timeout')
    output = ROOT / cfg.output
    output.mkdir(parents=True, exist_ok=False)
    env_cfg = instantiate(cfg.env, _convert_='all')
    scene = (ROOT / env_cfg.scene.model_file).resolve()
    env_cfg.scene.model_file = str(scene)
    env = make_manager_based_rl_env(env_cfg, num_envs=2, backend_type='mujoco')
    records = []
    try:
        env.reset()
        initial, _ = env.reset()
        initial = initial['obs'].clone()
        names = list(env_cfg.scene.entities['robot'].joint_names)
        if env.action_manager.total_action_dim != 16:
            raise AssertionError('Expected 16 independent joint targets')

        def joint_state():
            view = env.scene.read_plan.joint_tensor_view(env.scene['robot'])
            return view.joint_pos.clone(), view.joint_vel.clone()

        initial_q, initial_v = joint_state()
        for step in range(cfg.steps):
            action = torch.zeros((2, 16), device=env.device)
            # Smooth bounded position offsets; env1 always holds the source home.
            envelope = math.sin(math.pi * (step + 1) / cfg.steps) ** 2
            action[0, names.index('left_joint1')] = .15 * envelope
            action[0, names.index('right_joint1')] = -.15 * envelope
            for name in ('left_joint7', 'left_joint8', 'right_joint7', 'right_joint8'):
                action[0, names.index(name)] = -.015 * envelope
            state = env.step(action)
            q, v = joint_state()
            if not torch.isfinite(state.obs['obs']).all() or not torch.isfinite(v).all():
                raise AssertionError('Nonfinite control state')
            if state.terminated.any() or state.truncated.any():
                raise AssertionError('Unexpected early episode end')
            records.append(dict(step=step + 1, action=action.tolist(), qpos=q.tolist(),
                                qvel=v.tolist(), object_pose=state.obs['obs'][:, :28].tolist()))
        midpoint = records[cfg.steps // 2 - 1]
        errors = {}
        for name in ('left_joint1', 'right_joint1', 'left_joint7', 'left_joint8', 'right_joint7', 'right_joint8'):
            index = names.index(name)
            target = initial_q[0, index].item() + midpoint['action'][0][index]
            error = abs(midpoint['qpos'][0][index] - target)
            errors[name] = error
            tolerance = .02 if name.endswith('joint1') else .004
            if error > tolerance:
                raise AssertionError(f'{name} target tracking error {error} exceeds {tolerance}')
            if abs(midpoint['qpos'][0][index] - midpoint['qpos'][1][index]) < tolerance:
                raise AssertionError(f'{name} did not respond independently to environment 0 actions')
        hold_error = max(abs(r['qpos'][1][j] - initial_q[1, j].item())
                         for r in records for j in range(16))
        if hold_error > .001:
            raise AssertionError(f'Uncommanded environment failed to hold home: {hold_error}')
        before_reset = state.obs['obs'].clone()
        before_q, before_v = joint_state()
        after, _ = env.reset(torch.tensor([0], device=env.device))
        reset_q, reset_v = joint_state()
        torch.testing.assert_close(reset_q[0], initial_q[0], atol=1e-6, rtol=0)
        torch.testing.assert_close(reset_v[0], initial_v[0], atol=1e-6, rtol=0)
        torch.testing.assert_close(after['obs'][0], initial[0], atol=1e-6, rtol=0)
        torch.testing.assert_close(reset_q[1], before_q[1], atol=0, rtol=0)
        torch.testing.assert_close(reset_v[1], before_v[1], atol=0, rtol=0)
        torch.testing.assert_close(after['obs'][1], before_reset[1], atol=0, rtol=0)
    finally:
        env.close()
    (output / 'states.json').write_text(json.dumps(records) + '\n')
    (output / 'config.json').write_text(json.dumps(OmegaConf.to_container(cfg, resolve=True), indent=2) + '\n')
    if cfg.video:
        from integrations.unilab.render_x5_recording import render
        render(scene, records, names, output, cfg.env.ctrl_dt)
    receipt = dict(scope=cfg.scope, backend='mujoco', manager_environment=True,
                   task_success=None, robot_task_ready=False, num_envs=2, steps=cfg.steps,
                   joint_order=names, observation_shape=list(initial.shape),
                   midpoint_tracking_errors=errors, partial_reset_verified=True,
                   hold_environment_max_joint_error=hold_error,
                   independent_actions_verified=True, video=bool(cfg.video),
                   versions={n: version(n) for n in ('unisim-core', 'mujoco', 'torch')},
                   scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                   limitations=['Scripted joint offsets, not an insertion policy',
                                'URDF convex collisions and adapted drives; not PhysX equivalence',
                                'ARX parallel grippers, not a dexterous-hand benchmark result'])
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
