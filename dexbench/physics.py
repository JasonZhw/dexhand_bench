"""Explicit MuJoCo knobs and descriptive probes, not causal attribution."""
import math


def validate_options(options):
    allowed = {'timestep', 'iterations', 'tolerance', 'solver', 'integrator'}
    if set(options) - allowed:
        raise ValueError(
            f'Unsupported physics options: {sorted(set(options)-allowed)}')
    for key, value in options.items():
        if key in ('timestep', 'tolerance'):
            if type(value) not in (int, float) or not math.isfinite(
                    value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        elif key == 'iterations':
            if type(value) is not int or value <= 0:
                raise ValueError('iterations must be a positive integer')
        elif key == 'solver' and value not in ('PGS', 'CG', 'Newton'):
            raise ValueError('Unknown solver')
        elif key == 'integrator' and value not in ('Euler', 'RK4', 'implicit', 'implicitfast'):
            raise ValueError('Unknown integrator')
    return dict(options)


def contact_metrics(contacts):
    if contacts is None:
        return {'contact_count': None, 'max_penetration_m': None,
                'normal_force_sum_N': None}
    return {'contact_count': len(contacts),
            'max_penetration_m': max([max(0., -c['distance_m']) for c in contacts], default=0.),
            'normal_force_sum_N': sum(abs(c['force_local_N_Nm'][0]) for c in contacts)}
