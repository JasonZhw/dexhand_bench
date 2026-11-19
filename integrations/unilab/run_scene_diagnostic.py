"""Run a real Manager-Based object scene from Hydra; not a robot task."""
import json
from pathlib import Path
import sys
import hashlib
import math
from importlib.metadata import version

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/UniLab/src')]

import hydra
from hydra.utils import instantiate
from omegaconf import DictConfig, OmegaConf
import torch
from unilab.envs import make_manager_based_rl_env


@hydra.main(version_base=None, config_path='../../conf/unilab', config_name='scene_diagnostic')
def main(cfg: DictConfig):
    if cfg.scope != 'object_scene_diagnostic' or cfg.backend != 'mujoco':
        raise ValueError('Only the verified MuJoCo object-scene route is enabled')
    if cfg.num_envs < 2 or cfg.steps < 5:
        raise ValueError('Acceptance requires at least 2 environments and 5 steps')
    if math.ceil(cfg.env.max_episode_seconds / cfg.env.ctrl_dt) <= 5:
        raise ValueError('Reset acceptance requires an episode longer than 5 steps')
    output = ROOT / cfg.output
    output.mkdir(parents=True, exist_ok=False)
    resolved = OmegaConf.to_container(cfg, resolve=True)
    env_cfg = instantiate(cfg.env, _convert_='all')
    env_cfg.scene.model_file = str((ROOT / env_cfg.scene.model_file).resolve())
    env = make_manager_based_rl_env(env_cfg, num_envs=cfg.num_envs, backend_type=cfg.backend)
    try:
        env.reset()  # Bootstrap can randomize initial episode counters upstream.
        obs, info = env.reset()
        initial = obs['obs'].clone()
        assert initial.shape == (cfg.num_envs, 37)
        def body_velocity():
            view = env.scene.read_plan.body_tensor_view(
                env.scene['objects'], ('tube0', 'tube1', 'tube2', 'slot'))
            return torch.cat((view.lin_vel_w, view.ang_vel_w), dim=-1).clone()

        initial_velocity = body_velocity()
        actions = torch.zeros((cfg.num_envs, env.action_manager.total_action_dim), device=env.device)
        records, resets = [], []
        selected_reset_verified = False
        counters = torch.zeros(cfg.num_envs, dtype=torch.long, device=env.device)
        horizon = math.ceil(env_cfg.max_episode_seconds / env_cfg.ctrl_dt)

        def reset_and_check(ids, reason, step):
            before = state.obs['obs'].clone()
            after, info = env.reset(ids)
            poses = before[ids, :28].reshape(-1, 4, 7)
            expected_height = poses[:, :3, 2].mean().item()
            expected_count = before[ids, 28:].reshape(-1, 3, 3).bool().all(dim=-1).float().sum(dim=-1).mean().item()
            log = info['log']
            if not math.isclose(log['Episode_Metrics/mean_tube_height_m'], expected_height, abs_tol=1e-6):
                raise AssertionError('Height metric disagrees with observed pose')
            if log['Episode_Metrics/source_geometry_count'] != expected_count:
                raise AssertionError('Geometry metric disagrees with observed predicates')
            torch.testing.assert_close(after['obs'][ids], initial[ids], atol=1e-6, rtol=0)
            torch.testing.assert_close(body_velocity()[ids], initial_velocity[ids], atol=1e-6, rtol=0)
            counters[ids] = 0
            resets.append({'step': step, 'reason': reason, 'env_ids': ids.tolist(), 'info': info})
            return after

        for step in range(cfg.steps):
            state = env.step(actions)
            counters += 1
            torch.testing.assert_close(state.truncated, counters >= horizon)
            if state.terminated.any():
                raise AssertionError('Diagnostic has no success/failure termination term')
            assert torch.isfinite(state.obs['obs']).all()
            records.append({'step': step + 1, 'observation': state.obs['obs'].tolist(),
                            'terminated': state.terminated.tolist(), 'truncated': state.truncated.tolist()})
            done = (state.terminated | state.truncated).clone()
            if step == 4 and not done.any():
                if torch.max(torch.abs(state.obs['obs'][0, :28] - initial[0, :28])) <= 1e-4:
                    raise AssertionError('Objects must move before testing pose restoration')
                other = state.obs['obs'][1:].clone()
                other_velocity = body_velocity()[1:]
                obs = reset_and_check(torch.tensor([0], device=env.device), 'selected_reset_check', step + 1)
                torch.testing.assert_close(obs['obs'][0], initial[0], atol=1e-6, rtol=0)
                torch.testing.assert_close(obs['obs'][1:], other, atol=0, rtol=0)
                torch.testing.assert_close(body_velocity()[1:], other_velocity, atol=0, rtol=0)
                selected_reset_verified = True
            if done.any():
                ids = done.nonzero().flatten()
                reset_and_check(ids, 'episode_end', step + 1)
        if not selected_reset_verified:
            raise ValueError('Episode must permit at least one step before timeout for reset acceptance')
        receipt = {'scope': cfg.scope, 'manager_environment': True,
                   'robot_task_ready': False, 'task_success': None,
                   'steps': cfg.steps, 'num_envs': cfg.num_envs,
                   'observation_shape': list(initial.shape),
                   'selected_pose_reset_verified': selected_reset_verified,
                   'selected_velocity_reset_verified': selected_reset_verified,
                   'reset_after_observed_motion': True, 'reset_events': resets,
                   'timeout_counters_verified': True, 'metrics_match_observations': True,
                   'versions': {name: version(name) for name in ('unisim-core', 'mujoco', 'torch')},
                   'scene_xml_sha256': hashlib.sha256(Path(env_cfg.scene.model_file).read_bytes()).hexdigest(),
                   'observation_layout': '4 x (xyz,wxyz), then 3 x (depth,inside,upright)',
                   'limitations': ['No robot or control policy', 'Geometry only; no release/home predicates',
                                   'Height is a diagnostic, not task success or physical accuracy']}
    finally:
        env.close()
    (output / 'config.json').write_text(json.dumps(resolved, indent=2) + '\n')
    (output / 'states.json').write_text(json.dumps(records) + '\n')
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
