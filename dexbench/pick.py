"""Backend-independent evidence for the lift/hold substage, not grasp quality."""
import math


def evaluate_hold(positions, baseline, minimum_lift, maximum_drift):
    points = [tuple(p) for p in positions]
    baseline = tuple(baseline)
    if not points or any(len(p) != 3 for p in [baseline, *points]):
        raise ValueError('Require nonempty xyz samples and an xyz baseline')
    if not all(math.isfinite(v) for p in [baseline, *points] for v in p):
        raise ValueError('Positions must be finite')
    if not math.isfinite(minimum_lift) or minimum_lift <= 0:
        raise ValueError('Minimum lift must be positive and finite')
    if not math.isfinite(maximum_drift) or maximum_drift < 0:
        raise ValueError('Maximum drift must be nonnegative and finite')
    gain = float(min(p[2] for p in points) - baseline[2])
    drift = max(math.dist(p, points[0]) for p in points)
    return dict(lift_observed=gain >= minimum_lift and drift <= maximum_drift,
                minimum_height_gain_m=gain, maximum_hold_drift_m=drift,
                hold_samples=len(points))
