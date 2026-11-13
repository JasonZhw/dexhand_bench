import unittest
from dexbench.profiles import validate_profile


class ProfileTests(unittest.TestCase):
    def good(self): return {
        'id': 'H01-source',
        'task_id': 'H01',
        'backend': 'gym_robotics_mujoco',
        'env_id': 'HandManipulateBlockRotateXYZ-v1',
        'protocol': 'source-final-predicate-v1',
        'physics': {}}

    def test_valid_source_profile(self): validate_profile(self.good())

    def test_reject_unimplemented_backend(self):
        p = self.good()
        p['backend'] = 'unilab'
        with self.assertRaises(ValueError):
            validate_profile(p)

    def test_unknown_keys_fail_closed(self):
        p = self.good()
        p['phsyics'] = {}
        with self.assertRaises(ValueError):
            validate_profile(p)

    def test_physics_changes_are_validated(self):
        p = self.good()
        p['physics'] = {'timestep': -1}
        with self.assertRaises(ValueError):
            validate_profile(p)

    def test_missing_protocol_rejected(self):
        p = self.good()
        del p['protocol']
        with self.assertRaises(ValueError):
            validate_profile(p)

    def test_task_cannot_be_mislabeled_as_other_environment(self):
        p = self.good()
        p['env_id'] = 'AdroitHandDoor-v1'
        with self.assertRaises(ValueError):
            validate_profile(p)
