import unittest
from pathlib import Path
try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]

@unittest.skipIf(mujoco is None, 'Requires scene runtime')
class X5IKTests(unittest.TestCase):
    def test_reachable_pose_and_unreachable_rejection(self):
        from integrations.unilab.x5_ik import X5IK
        path = ROOT / 'assets/generated/I01-x5-control/scene.xml'
        if not path.exists():
            self.skipTest('Generate X5 scene first')
        solver = X5IK(path, 'right', .145)
        target_q = np.array([.1, -.4, .5, -.2, .2, .1])
        pos, quat = solver.pose(target_q)
        actual, residual = solver.solve(pos, quat, target_q + .05)
        self.assertLess(residual['position_m'], .001)
        self.assertLess(residual['rotation_rad'], .02)
        with self.assertRaises(ValueError):
            solver.solve(np.array([100., 100., 100.]), quat, actual, attempts=1)

if __name__ == '__main__': unittest.main()
