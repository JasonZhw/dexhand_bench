import importlib.util
import unittest
HAS_GYM = bool(importlib.util.find_spec('gymnasium_robotics'))


@unittest.skipUnless(HAS_GYM, 'optional simulation dependencies not installed')
class GymAdapterTests(unittest.TestCase):
    def make(self, **kw):
        from dexbench.backends.gym_robotics import GymRoboticsBackend
        b = GymRoboticsBackend('HandManipulateBlockRotateXYZ-v1', **kw)
        self.addCleanup(b.close)
        return b

    def test_reset_reproduces_initial_state(self):
        b = self.make()
        b.reset(17)
        first = b.snapshot()
        b.step(b.action('random'))
        b.reset(17)
        self.assertEqual(first, b.snapshot())

    def test_action_shape_and_nonfinite_rejected(self):
        b = self.make()
        b.reset(1)
        for action in ([0], [float('nan')] * 20):
            with self.assertRaises(ValueError):
                b.step(action)

    def test_step_has_boolean_success_and_applied_control(self):
        b = self.make()
        b.reset(1)
        r = b.step([2.] * 20)
        self.assertIs(type(r['success']), bool)
        self.assertEqual(r['applied_action'], [1.] * 20)
        self.assertEqual(len(r['actuator_control']), 20)

    def test_explicit_parameters_and_contacts(self):
        b = self.make(physics={'iterations': 17, 'solver': 'Newton'})
        b.reset(4)
        b.step(b.action('zero'))
        s = b.snapshot(contacts=True)
        self.assertIsInstance(s['contacts'], list)
        self.assertEqual(b.metadata()['physics']['iterations'], 17)
        self.assertIsNone(b.snapshot()['contacts'])

    def test_unsupported_env_is_rejected(self):
        from dexbench.backends.gym_robotics import GymRoboticsBackend
        with self.assertRaises(ValueError):
            GymRoboticsBackend('CartPole-v1')

    def test_each_profile_environment_is_registered(self):
        import gymnasium as gym
        import gymnasium_robotics
        from dexbench.backends.gym_robotics import SUPPORTED_ENVS
        gym.register_envs(gymnasium_robotics)
        for env_id in SUPPORTED_ENVS:
            self.assertEqual(gym.spec(env_id).id, env_id)

    def test_all_source_environments_reset_repeatably(self):
        from dexbench.backends.gym_robotics import GymRoboticsBackend, SUPPORTED_ENVS
        for env_id in sorted(SUPPORTED_ENVS):
            with self.subTest(env_id=env_id):
                backend = GymRoboticsBackend(env_id)
                try:
                    backend.reset(9)
                    first = backend.snapshot()
                    backend.step(backend.action('random'))
                    backend.reset(9)
                    self.assertEqual(first, backend.snapshot())
                finally:
                    backend.close()
