"""Execution profiles are distinct from task catalog implementation status."""
import hashlib
import json
from pathlib import Path
from .physics import validate_options
from .backends.gym_robotics import SUPPORTED_ENVS


SOURCE_ENV_BY_TASK = {'H01': 'HandManipulateBlockRotateXYZ-v1', 'H02': 'AdroitHandPen-v1',
                      'H03': 'HandManipulateEggRotate-v1', 'R03': 'AdroitHandDoor-v1', 'P02': 'FetchSlide-v4'}


def validate_profile(p):
    allowed = {
        'id',
        'task_id',
        'backend',
        'env_id',
        'protocol',
        'physics',
        'notes'}
    if not isinstance(p, dict) or set(p) - allowed:
        raise ValueError('Unknown execution profile fields')
    for key in ('id', 'task_id', 'backend', 'env_id', 'protocol'):
        if not isinstance(p.get(key), str) or not p[key]:
            raise ValueError(f'Profile requires {key}')
    if p['backend'] != 'gym_robotics_mujoco':
        raise ValueError('Backend not implemented; no silent fallback')
    if p['env_id'] not in SUPPORTED_ENVS:
        raise ValueError('Unsupported source environment')
    if SOURCE_ENV_BY_TASK.get(p['task_id']) != p['env_id']:
        raise ValueError('Source environment does not match catalog task')
    if p['protocol'] != 'source-final-predicate-v1':
        raise ValueError('Unsupported outcome protocol')
    validate_options(p.get('physics', {}))
    return p


def load_profile(path): return validate_profile(
    json.loads(Path(path).read_text(encoding='utf-8')))


def make_backend(profile, render=False):
    validate_profile(profile)
    from .backends.gym_robotics import GymRoboticsBackend
    return GymRoboticsBackend(
        profile['env_id'], physics=profile.get('physics'), render=render)


def provenance(profile):
    root = Path(__file__).parent
    return {'profile': profile, 'profile_sha256': hashlib.sha256(json.dumps(profile, sort_keys=True).encode()).hexdigest(),
            'runner_sources': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*.py'))},
            'adaptation': 'none; original source embodiment',
            'controller_note': 'zero/random or caller-provided replay; no trained success baseline'}
