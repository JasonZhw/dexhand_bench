import json
from pathlib import Path
import tempfile
import unittest

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]


class AllegroProfileTests(unittest.TestCase):
    def test_all_five_scenes_select_the_dexterous_robot(self):
        for tid in ('G01', 'G02', 'G03', 'G04', 'G05'):
            profile = json.loads((ROOT/'profiles/local_grasp'/f'{tid}.json').read_text())
            self.assertEqual(profile.get('robot_config'), 'profiles/robots/dual_x5_allegro.json')


@unittest.skipIf(mujoco is None, 'Requires MuJoCo scene runtime')
class AllegroRobotTests(unittest.TestCase):
    def setUp(self):
        if not (ROOT/'assets/menagerie/wonik_allegro/left_hand.xml').is_file():
            self.skipTest('Fetch pinned Allegro assets first')

    def test_both_hands_replace_grippers_with_ordered_independent_controls(self):
        from integrations.unilab.build_allegro_robot import build
        with tempfile.TemporaryDirectory() as tmp:
            path = build(Path(tmp)/'robot')
            model = mujoco.MjModel.from_xml_path(str(path))
            spec = json.loads((path.parent/'robot.json').read_text())
            self.assertEqual(model.nu, 44)
            self.assertEqual(model.nq, 44)
            np.testing.assert_array_equal(model.actuator_trnid[:, 0], np.arange(44))
            self.assertTrue(np.all(model.qpos0 >= model.jnt_range[:, 0]))
            self.assertTrue(np.all(model.qpos0 <= model.jnt_range[:, 1]))
            for side in ('left', 'right'):
                self.assertEqual(len(spec['hand_joint_names'][side]), 16)
                self.assertEqual(len(spec['hands'][side]['fingers']), 4)
                for index in (7, 8):
                    self.assertNotIn(f'{side}_joint{index}', spec['joint_names'])
                    self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'{side}_link{index}'), -1)
            data = mujoco.MjData(model)
            data.ctrl[:] = model.qpos0
            mujoco.mj_forward(model, data)
            self.assertFalse(any(c.dist < -1e-6 for c in data.contact), 'No initial robot collisions')
            for _ in range(250):
                mujoco.mj_step(model, data)
            self.assertTrue(np.isfinite(data.qpos).all())
            np.testing.assert_allclose(data.qpos, model.qpos0, atol=.005)

    def test_hand_joint_frames_match_source_forward_kinematics(self):
        from integrations.unilab.build_allegro_robot import build
        with tempfile.TemporaryDirectory() as tmp:
            path = build(Path(tmp)/'robot')
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
            for side in ('left', 'right'):
                source = mujoco.MjModel.from_xml_path(str(ROOT/'assets/menagerie/wonik_allegro'/f'{side}_hand.xml'))
                original = mujoco.MjData(source)
                for j in range(source.njnt):
                    name = source.joint(j).name
                    value = np.clip(.2, *source.jnt_range[j])
                    original.qpos[source.jnt_qposadr[j]] = value
                    data.qpos[model.joint(f'{side}_allegro_{name}').qposadr[0]] = value
                mujoco.mj_forward(source, original)
                mujoco.mj_forward(model, data)
                source_palm, palm = original.body('palm'), data.body(f'{side}_allegro_palm')
                for finger in ('ff', 'mf', 'rf', 'th'):
                    expected = (original.body(f'{finger}_tip').xpos-source_palm.xpos) @ source_palm.xmat.reshape(3,3)
                    actual = (data.body(f'{side}_allegro_{finger}_tip').xpos-palm.xpos) @ palm.xmat.reshape(3,3)
                    np.testing.assert_allclose(actual, expected, atol=1e-7)


if __name__ == '__main__':
    unittest.main()
