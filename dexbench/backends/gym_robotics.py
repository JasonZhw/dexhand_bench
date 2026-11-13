"""Original Farama task environments. Not a UniLab or robot-retargeting adapter."""
import hashlib
from importlib.metadata import version
import inspect
from pathlib import Path
import platform

from ..physics import validate_options, contact_metrics

SUPPORTED_ENVS = {
    'HandManipulateBlockRotateXYZ-v1',
    'HandManipulateEggRotate-v1',
    'AdroitHandPen-v1',
    'AdroitHandDoor-v1',
    'FetchSlide-v4',
}


def plain(value):
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if hasattr(value, 'tolist'):
        return value.tolist()
    return value


class GymRoboticsBackend:
    def __init__(self, env_id, *, physics=None, render=False):
        if env_id not in SUPPORTED_ENVS:
            raise ValueError(f'Unsupported source environment: {env_id}')
        physics = validate_options(physics or {})
        import numpy as np
        import mujoco
        import gymnasium as gym
        import gymnasium_robotics
        gym.register_envs(gymnasium_robotics)
        self.np = np
        self.mj = mujoco
        self.env_id = env_id
        self.env = gym.make(
            env_id, render_mode='rgb_array' if render else None)
        self.base = self.env.unwrapped
        self.model = self.base.model
        self.data = self.base.data
        self.render_enabled = render
        self.original_physics = self.physics_config()
        for key, value in physics.items():
            if key == 'solver':
                value = getattr(mujoco.mjtSolver, 'mjSOL_' + value.upper())
            elif key == 'integrator':
                value = getattr(mujoco.mjtIntegrator, 'mjINT_' + value.upper())
            setattr(self.model.opt, key, value)
        self.physics_overrides = physics
        self._meta = None

    def physics_config(self):
        return {k: float(getattr(self.model.opt, k)) if k in ('timestep', 'tolerance') else int(getattr(self.model.opt, k))
                for k in ('timestep', 'tolerance', 'iterations', 'solver', 'integrator')}

    def metadata(self):
        if self._meta is not None:
            return self._meta
        import gymnasium_robotics
        root = Path(gymnasium_robotics.__file__).parent
        # Hash installed source and all bundled assets; cold path only,
        # portable relative paths.
        hashed = {}
        for folder in (root / 'envs/assets',):
            for path in sorted(folder.rglob('*')):
                if path.is_file():
                    hashed[str(path.relative_to(root))] = hashlib.sha256(
                        path.read_bytes()).hexdigest()
        source_files = {}
        for cls in type(self.base).__mro__:
            try:
                p = Path(inspect.getfile(cls))
            except TypeError:
                continue
            if root in p.parents and p.is_file():
                source_files[str(p.relative_to(root))] = hashlib.sha256(
                    p.read_bytes()).hexdigest()
        self._meta = {
            'backend': 'mujoco', 'environment_id': self.env_id,
            'packages': {p: version(p) for p in ('mujoco', 'gymnasium', 'gymnasium-robotics', 'numpy')},
            'python': platform.python_version(), 'platform': platform.platform(), 'processor': platform.processor(),
            'device': 'cpu', 'physics': self.physics_config(), 'original_physics': self.original_physics,
            'physics_overrides': self.physics_overrides, 'physics_dt': float(self.model.opt.timestep),
            'control_dt': float(self.base.dt), 'source_horizon': self.env.spec.max_episode_steps,
            'render_mode': 'rgb_array' if self.render_enabled else None,
            'capabilities': {'contacts': True, 'contact_forces': True, 'qpos_qvel': True, 'body_poses': True,
                             'batched_gpu': False, 'exact_checkpoint_restore': False},
            'asset_hashes': hashed, 'source_hashes': source_files,
            'coordinate_convention': 'MuJoCo world; body quaternion wxyz; SI except normalized action',
            'action_shape': list(self.env.action_space.shape),
            'action_low': plain(self.env.action_space.low), 'action_high': plain(self.env.action_space.high),
            'joint_mapping': [{'name': self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_JOINT, i),
                               'qpos_address': int(self.model.jnt_qposadr[i]), 'qvel_address': int(self.model.jnt_dofadr[i]),
                               'type': int(self.model.jnt_type[i])} for i in range(self.model.njnt)],
            'body_names': [self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_BODY, i) for i in range(self.model.nbody)],
            'actuator_names': [self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(self.model.nu)],
            'contact_sampling': 'instantaneous solver contact data at each control-step end, not substep maxima',
        }
        return self._meta

    def reset(self, seed):
        self.env.action_space.seed(seed)
        obs, info = self.env.reset(seed=seed)
        return {'observation': plain(obs), 'info': plain(info)}

    def action(self, mode):
        if mode == 'zero':
            return self.np.zeros(self.env.action_space.shape,
                                 dtype=self.env.action_space.dtype)
        if mode == 'random':
            return self.env.action_space.sample()
        raise ValueError(f'Unknown action mode: {mode}')

    def step(self, action):
        requested = self.np.asarray(action, dtype=self.env.action_space.dtype)
        if requested.shape != self.env.action_space.shape or not self.np.isfinite(
                requested).all():
            raise ValueError(
                f'Action must be finite with shape {self.env.action_space.shape}')
        applied = self.np.clip(
            requested,
            self.env.action_space.low,
            self.env.action_space.high)
        obs, reward, terminated, truncated, info = self.env.step(applied)
        raw = info.get('success', info.get('is_success'))
        success = None if raw is None else bool(raw)
        return {'observation': plain(obs), 'reward': float(reward), 'terminated': bool(terminated),
                'truncated': bool(truncated), 'success': success, 'info': plain(info),
                'applied_action': plain(applied), 'actuator_control': plain(self.data.ctrl.copy())}

    def snapshot(self, contacts=False):
        contact_rows = None
        if contacts:
            contact_rows = []
            for i in range(self.data.ncon):
                c = self.data.contact[i]
                force = self.np.zeros(6)
                self.mj.mj_contactForce(self.model, self.data, i, force)
                contact_rows.append({'geom_ids': [int(c.geom1), int(c.geom2)],
                                     'geom_names': [self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_GEOM, int(g)) for g in (c.geom1, c.geom2)],
                                     'position_world_m': plain(c.pos.copy()), 'frame_world_rows': plain(c.frame.reshape(3, 3).copy()),
                                     'distance_m': float(c.dist), 'force_local_N_Nm': plain(force)})
        return {'sim_time': float(self.data.time), 'qpos': plain(self.data.qpos.copy()), 'qvel': plain(self.data.qvel.copy()),
                'body_position_world_m': plain(self.data.xpos.copy()), 'body_quaternion_wxyz': plain(self.data.xquat.copy()),
                'actuator_force': plain(self.data.actuator_force.copy()),
                'contacts': contact_rows, 'contacts_unavailable_reason': None if contacts else 'not_requested',
                'contact_metrics': contact_metrics(contact_rows),
                'finite_state': bool(self.np.isfinite(self.data.qpos).all() and self.np.isfinite(self.data.qvel).all())}

    def synchronize(self):
        # MuJoCo CPU step returns synchronously; future GPU adapters must
        # implement a real fence.
        pass

    def render(self): return self.env.render()
    def close(self): self.env.close()
