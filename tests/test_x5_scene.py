"""Asset integration checks; run with the scene runtime and fetched X5 assets."""
import tempfile
import unittest
from pathlib import Path

try:
    import mujoco
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]

@unittest.skipIf(mujoco is None, 'Requires MuJoCo scene runtime')
class X5SceneTests(unittest.TestCase):
    def test_official_robot_compiles_with_disjoint_controls_and_preserved_objects(self):
        from integrations.unilab.build_x5_scene import build
        if not (ROOT / 'assets/robodojo/Assets/Robots/x5/meshes/link8.STL').exists():
            self.skipTest('Fetch I01-x5 manifest first')
        with tempfile.TemporaryDirectory() as tmp:
            path = build(Path(tmp) / 'scene')
            model = mujoco.MjModel.from_xml_path(str(path))
            self.assertEqual(model.nu, 16)
            self.assertEqual(model.nq, 16 + 3 * 7)
            self.assertEqual(model.nv, 16 + 3 * 6)
            for side in ('left', 'right'):
                for index in range(1, 9):
                    joint = model.joint(f'{side}_joint{index}')
                    self.assertLess(joint.qposadr[0], 16, 'Controlled scalar joints must precede free objects')
                    self.assertEqual(model.actuator(f'{side}_joint{index}_drive').trnid[0], joint.id)
                    if index >= 7:
                        self.assertAlmostEqual(model.qpos0[joint.qposadr[0]], .044)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            self.assertTrue(all(abs(data.body(f'{s}_base_link').xpos[0] - x) < 1e-6
                                for s, x in [('left', -.3), ('right', .3)]))
            import numpy as np
            original = mujoco.MjModel.from_xml_path(str(path.parent / 'source_robot.xml'))
            source_data = mujoco.MjData(original)
            source_data.qpos[6:8] = .044
            mujoco.mj_forward(original, source_data)
            for side in ('left', 'right'):
                base = data.body(side + '_base_link')
                rotation = base.xmat.reshape(3, 3)
                for link in ('link6', 'link7', 'link8'):
                    expected = rotation @ source_data.body(link).xpos + base.xpos
                    np.testing.assert_allclose(data.body(side + '_' + link).xpos, expected, atol=1e-7)
            data.ctrl[:] = model.qpos0[:16]
            data.ctrl[0] = .15
            for _ in range(250):
                mujoco.mj_step(model, data)
            self.assertLess(abs(data.qpos[0] - .15), .02,
                            'Adjacent link collision must not lock the base joint')

if __name__ == '__main__':
    unittest.main()
