"""Paired recorded trajectories, not an accuracy score against reality."""
import json
import math
from pathlib import Path


def finite_vector(value, size=None):
    return (isinstance(value, list) and (size is None or len(value) == size)
            and all(type(v) in (int, float) and math.isfinite(v) for v in value))


def validate_poses(state, count):
    for field, size in (('body_position_world_m', 3),
                        ('body_quaternion_wxyz', 4)):
        values = state.get(field)
        if not isinstance(values, list) or len(values) != count or not all(
                finite_vector(v, size) for v in values):
            raise ValueError('Malformed or nonfinite ' + field)
        if size == 4 and any(math.hypot(*v) == 0 for v in values):
            raise ValueError('Invalid zero quaternion')


def compare_runs(left, right):
    left, right = Path(left), Path(right)
    def load(p, f): return json.loads((p / f).read_text())
    a, b = load(left, 'manifest.json'), load(right, 'manifest.json')
    for key in ('task_id', 'seed'):
        if a[key] != b[key]:
            raise ValueError(f'Cannot compare different {key}')
    for key in ('environment_id', 'control_dt', 'body_names', 'asset_hashes'):
        if a['backend'].get(key) != b['backend'].get(key):
            raise ValueError(f'Cannot compare mismatched {key}')
    if a['provenance']['profile']['protocol'] != b['provenance']['profile']['protocol']:
        raise ValueError('Cannot compare different protocols')
    if load(left, 'actions.json') != load(right, 'actions.json'):
        raise ValueError('Actions differ')
    if not all(load(p, 'summary.json')[
               'execution_completed'] for p in (left, right)):
        raise ValueError('Incomplete execution cannot be compared')
    if not all((p / 'steps.jsonl').exists() for p in (left, right)):
        raise ValueError(
            'Diagnostic traces required; throughput runs have no state trace')
    traces = [[json.loads(x) for x in (
        p / 'steps.jsonl').read_text().splitlines()] for p in (left, right)]
    if not traces[0] or len(traces[0]) != len(traces[1]):
        raise ValueError('Need nonempty equal-length traces')
    names = a['backend']['body_names']
    if not names or len(set(names)) != len(names):
        raise ValueError('Body names must be nonempty and unique')
    stats = {str(n if n is not None else f'body_{i}'): {'max_position_difference_m': 0.,
                                                        'max_orientation_difference_rad': 0.} for i, n in enumerate(names)}
    for x, y in zip(*traces):
        sx, sy = x['state'], y['state']
        for state in (sx, sy):
            validate_poses(state, len(names))
            if not finite_vector([state.get('sim_time')]):
                raise ValueError('Nonfinite simulation time')
        if x['step_index'] != y['step_index'] or not math.isclose(
                sx['sim_time'], sy['sim_time'], abs_tol=1e-9):
            raise ValueError('Trace samples are not time aligned')
        for i, name in enumerate(stats):
            pa, pb = sx['body_position_world_m'][i], sy['body_position_world_m'][i]
            qa, qb = sx['body_quaternion_wxyz'][i], sy['body_quaternion_wxyz'][i]
            distance = math.sqrt(sum((u - v)**2 for u, v in zip(pa, pb)))
            norm = math.sqrt(sum(v * v for v in qa) * sum(v * v for v in qb))
            if norm == 0:
                raise ValueError('Invalid zero quaternion')
            angle = 2 * \
                math.acos(
                    min(1., abs(sum(u * v for u, v in zip(qa, qb)) / norm)))
            if not math.isfinite(distance + angle):
                raise ValueError('Nonfinite trajectory')
            stats[name]['max_position_difference_m'] = max(
                stats[name]['max_position_difference_m'], distance)
            stats[name]['max_orientation_difference_rad'] = max(
                stats[name]['max_orientation_difference_rad'], angle)
    initial_identical = None
    if all((p / 'initial_state.json').is_file() for p in (left, right)):
        sa, sb = load(
            left, 'initial_state.json'), load(
            right, 'initial_state.json')
        keys = (
            'qpos',
            'qvel',
            'body_position_world_m',
            'body_quaternion_wxyz')
        if all(all(k in state for k in keys) for state in (sa, sb)):
            for state in (sa, sb):
                validate_poses(state, len(names))
                if not all(finite_vector(state[k]) for k in ('qpos', 'qvel')):
                    raise ValueError('Invalid initial joint state')
            initial_identical = all(sa[k] == sb[k] for k in keys)
    variation = {key: {'left': a['backend'].get(key), 'right': b['backend'].get(key)}
                 for key in ('physics', 'packages', 'source_hashes')
                 if a['backend'].get(key) != b['backend'].get(key)}
    return {'initial_conditions': {'recorded_state_identical': initial_identical,
                                   'confounded_by_reset': None if initial_identical is None else not initial_identical,
                                   'scope': 'recorded qpos/qvel/body poses only; not full solver state'},
            'configuration_differences': variation,
            'interpretation': 'simulation_consistency_not_physical_accuracy', 'samples': len(traces[0]),
            'left': str(left), 'right': str(right), 'bodies': stats,
            'note': 'Same commands and task configuration do not isolate a causal solver error; use controlled experiments and physical reference measurements'}
