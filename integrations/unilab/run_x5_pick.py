"""A scripted physical pick attempt, not a RoboDojo policy or full I01 task.

Planning uses a separate kinematic model. Execution only sends UniLab actions;
objects are never attached, teleported or given commanded poses.
"""
import json
from pathlib import Path
import sys
import hashlib
from importlib.metadata import version

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/UniLab/src')]
import hydra
from hydra.utils import instantiate
from omegaconf import OmegaConf
import numpy as np
import mujoco
import torch
from unilab.envs import make_manager_based_rl_env
from integrations.unilab.x5_ik import X5IK
from integrations.unilab.render_x5_recording import render
from integrations.unilab.pick_contact_probe import diagnose
from dexbench.pick import evaluate_hold


@hydra.main(version_base=None, config_path='../../conf/unilab', config_name='x5_pick_diagnostic')
def main(cfg):
    if cfg.scope != 'x5_pick_diagnostic' or cfg.backend != 'mujoco' or cfg.num_envs != 2:
        raise ValueError('This diagnostic requires two MuJoCo environments')
    if cfg.pick.target_body not in ('tube0', 'tube1', 'tube2'):
        raise ValueError('Select one of the three source tubes')
    if not 0 <= cfg.pick.close_position_m <= .044:
        raise ValueError('Finger target must be inside source joint limits [0, .044] m')
    durations = dict(cfg.pick.stages)
    if list(durations) != ['settle', 'approach', 'descend', 'close', 'lift', 'hold']:
        raise ValueError('Stage order must be settle/approach/descend/close/lift/hold')
    if any(not isinstance(v, int) or v <= 0 for v in durations.values()):
        raise ValueError('Stage durations must be positive integer steps')
    if sum(durations.values()) * cfg.env.ctrl_dt >= cfg.env.max_episode_seconds:
        raise ValueError('Diagnostic must finish before timeout')
    output = ROOT / cfg.output
    output.mkdir(parents=True, exist_ok=False)
    (output / 'config.json').write_text(json.dumps(OmegaConf.to_container(cfg, resolve=True), indent=2) + '\n')
    env_cfg = instantiate(cfg.env, _convert_='all')
    scene = (ROOT / env_cfg.scene.model_file).resolve()
    env_cfg.scene.model_file = str(scene)
    solver = X5IK(scene, cfg.pick.side, cfg.pick.tcp_offset_m)
    names = list(env_cfg.scene.entities['robot'].joint_names)
    arm = [names.index(f'{cfg.pick.side}_joint{i}') for i in range(1, 7)]
    fingers = [names.index(f'{cfg.pick.side}_joint{i}') for i in (7, 8)]
    tube = int(cfg.pick.target_body[-1])
    env = make_manager_based_rl_env(env_cfg, num_envs=2, backend_type='mujoco')
    records, stages, plan = [], [], {}
    reason, lifted, baseline = None, False, None
    try:
        env.reset()
        initial, _ = env.reset()
        initial_obs = initial['obs'].clone()
        view = env.scene.read_plan.joint_tensor_view(env.scene['robot'])
        home = view.joint_pos.clone()
        target = home[0].cpu().numpy().copy()
        for phase, count in durations.items():
            start = target.copy()
            if phase == 'approach':
                poses = np.array(records[-1]['object_pose'][0]).reshape(4, 7)
                baseline = poses[tube, :3].copy()
                matrix = np.zeros(9)
                mujoco.mju_quat2Mat(matrix, poses[tube, 3:])
                axis = matrix.reshape(3, 3)[:, 1].copy()
                axis[2] = 0
                axis /= np.linalg.norm(axis)
                down = np.array([0., 0., -1.])
                rotation = np.column_stack((down, np.cross(axis, down), axis))
                quat = np.zeros(4)
                mujoco.mju_mat2Quat(quat, rotation.flatten())
                seed = target[arm]
                try:
                    for label, height in [('approach', cfg.pick.approach_height_m),
                                          ('descend', 0.), ('lift', cfg.pick.lift_height_m)]:
                        point = baseline + np.array([0., 0., height])
                        seed, residual = solver.solve(point, quat, seed)
                        plan[label] = dict(q=seed.tolist(), tcp=point.tolist(), residual=residual)
                except ValueError as exc:
                    reason = 'ik_unreachable: ' + str(exc)
                    break
            if phase in plan:
                target[arm] = plan[phase]['q']
            if phase == 'close':
                target[fingers] = cfg.pick.close_position_m
            for frame in range(count):
                alpha = (frame + 1) / count
                alpha = alpha * alpha * (3 - 2 * alpha)
                qtarget = start + (target - start) * alpha
                actions = torch.zeros((2, 16), device=env.device)
                actions[0] = torch.as_tensor(qtarget, device=env.device) - home[0]
                state = env.step(actions)
                joints = env.scene.read_plan.joint_tensor_view(env.scene['robot'])
                if not torch.isfinite(state.obs['obs']).all() or not torch.isfinite(joints.joint_vel).all():
                    raise RuntimeError('Nonfinite simulation state')
                if state.terminated.any() or state.truncated.any():
                    raise RuntimeError('Unexpected episode end')
                records.append(dict(step=len(records) + 1, phase=phase,
                                    action=actions.tolist(), qpos=joints.joint_pos.tolist(),
                                    qvel=joints.joint_vel.tolist(), object_pose=state.obs['obs'][:, :28].tolist()))
            event = dict(phase=phase, end_step=len(records))
            if phase in plan:
                actual, _ = solver.pose(np.array(records[-1]['qpos'][0])[arm])
                event['tcp_error_m'] = float(np.linalg.norm(actual - np.array(plan[phase]['tcp'])))
                if event['tcp_error_m'] > .015:
                    reason = phase + '_tracking_failed'
            stages.append(event)
            if reason:
                break
        if reason is None:
            hold = np.array([r['object_pose'][0] for r in records if r['phase'] == 'hold']).reshape(-1, 4, 7)[:, tube, :3]
            evaluation = evaluate_hold(hold, baseline, cfg.pick.minimum_lift_m, cfg.pick.maximum_hold_drift_m)
            lifted = evaluation['lift_observed']
            reason = 'sustained_lift' if lifted else 'object_not_held_above_threshold'
            stages[-1].update(evaluation)
        before = state.obs['obs'].clone()
        after, _ = env.reset(torch.tensor([0], device=env.device))
        torch.testing.assert_close(after['obs'][0], initial_obs[0], atol=1e-6, rtol=0)
        torch.testing.assert_close(after['obs'][1], before[1], atol=0, rtol=0)
    finally:
        env.close()
        (output / 'states.json').write_text(json.dumps(records) + '\n')
        (output / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    if cfg.video and len(records) >= 10:
        render(scene, records, names, output, cfg.env.ctrl_dt)
    contact_probe = diagnose(scene, records, names, cfg.pick.side, cfg.pick.target_body)
    (output / 'contact_geometry.json').write_text(json.dumps(contact_probe, indent=2) + '\n')
    receipt = dict(scope=cfg.scope, lift_observed=lifted, reason=reason, stages=stages,
                   task_success=None, robot_task_ready=False, physical_execution='UniLab ManagerBasedRlEnv / MuJoCo',
                   steps=len(records), num_envs=2, sim_dt=cfg.env.sim_dt,
                   versions={n: version(n) for n in ('unisim-core', 'mujoco', 'torch')},
                   joint_order=names, selected_reset_verified=True,
                   scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                   contact_force_available=False, contact_force=None,
                   contact_geometry={k: v for k, v in contact_probe.items() if k != 'samples'},
                   limitations=['Self-designed scripted attempt, not official task policy',
                                'IK is pose reachability, not collision-free trajectory planning',
                                'Lift predicate is height and hold stability, not verified two-finger contact',
                                'No attachment or object pose writes during execution',
                                'No insertion attempt or full task success'])
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
