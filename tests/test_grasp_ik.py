import copy
import json
from pathlib import Path
import unittest

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(mujoco is None, 'Requires scene runtime')
class GraspIKTests(unittest.TestCase):
    def test_runner_accepts_script_only_for_supported_task_and_hand(self):
        from integrations.unilab.grasp_profiles import load_profile
        from integrations.unilab.run_grasp_scene import environment_config
        for task in ('G01', 'G02'):
            profile = load_profile(task, 'wuji')
            profile['runtime'].update(controller='ik_grasp', steps=2500)
            try:
                cfg = environment_config(profile)
            except ValueError as exc:
                self.fail(f'{task} IK runner mode is missing: {exc}')
            self.assertEqual(cfg.ctrl_dt, .004)
        for task, hand in [('G03', 'wuji'), ('G01', 'allegro')]:
            invalid = load_profile(task, hand)
            invalid['runtime']['controller'] = 'ik_grasp'
            with self.assertRaisesRegex(ValueError, 'G01.*G02.*wuji'):
                environment_config(invalid)

    def test_waypoints_reach_palm_targets_and_actions_leave_control_untouched(self):
        for task in ('G01', 'G02'):
            with self.subTest(task=task):
                self.check_waypoints(task)

    def check_waypoints(self, task):
        from integrations.unilab.grasp_ik import GraspIKController
        scene = ROOT/f'assets/generated/wuji_grasp/{task}/scene.xml'
        if not scene.exists():
            self.skipTest(f'Build Wuji {task} scene first')
        robot = json.loads((scene.parent/'scene.json').read_text())['robot']
        settings = json.loads((ROOT/f'profiles/controllers/{task}_wuji_ik.json').read_text())
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        home = np.tile(model.qpos0[:model.nu], (2, 1))
        pose = np.tile(np.r_[data.body('object').xpos, data.body('object').xquat], (2, 1))
        original = home.copy(), pose.copy()
        palm_home_rotation = data.body(robot['hands']['right']['palm']).xmat.reshape(3, 3).copy()
        controller = GraspIKController(scene, robot, home, pose, settings)
        for waypoint in controller.plan['waypoints']:
            for name, q in zip(robot['joint_names'], waypoint['qpos']):
                data.qpos[model.joint(name).qposadr[0]] = q
            mujoco.mj_forward(model, data)
            np.testing.assert_allclose(data.body(robot['hands']['right']['palm']).xpos,
                                       waypoint['palm_position_m'], atol=.001)
            if task == 'G02' and waypoint['phase'] in ('close', 'hold'):
                # Bottle config turns the palm 90 degrees about world Y for a side grasp.
                expected_rotation = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]) @ palm_home_rotation
                np.testing.assert_allclose(data.body(robot['hands']['right']['palm']).xmat.reshape(3, 3),
                                           expected_rotation, atol=.005)
        left = [i for i, name in enumerate(robot['joint_names']) if name.startswith('left_')]
        actions = [controller.action(t)[0] for t in np.arange(0, controller.duration_s+.004, .004)]
        self.assertLess(float(np.max(np.abs(np.diff(actions, axis=0)))), .04)
        for t in (0, controller.duration_s/2, controller.duration_s):
            action, stage = controller.action(t)
            np.testing.assert_array_equal(action[1], 0)
            np.testing.assert_array_equal(action[0, left], 0)
        self.assertEqual(controller.action(controller.duration_s)[1], 'hold')
        self.assertGreater(np.max(np.abs(actions[-1])), .1)
        np.testing.assert_array_equal(home, original[0])
        np.testing.assert_array_equal(pose, original[1])
        bad = copy.deepcopy(settings)
        bad['object_center_in_palm_m'] = [50, 50, 50]
        with self.assertRaisesRegex(ValueError, 'reachable'):
            GraspIKController(scene, robot, home, pose, bad)


if __name__ == '__main__':
    unittest.main()
