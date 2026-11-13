import unittest
from dexbench.physics import validate_options, contact_metrics


class PhysicsTests(unittest.TestCase):
    def test_reject_unknown_parameter(self):
        with self.assertRaises(ValueError):
            validate_options({'slovre': 'Newton'})

    def test_reject_nonphysical_timestep(self):
        for value in (0, -1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                validate_options({'timestep': value})

    def test_iterations_must_be_positive_integer(self):
        for value in (0, 1.5, False):
            with self.assertRaises(ValueError):
                validate_options({'iterations': value})

    def test_solver_choices_are_explicit(self):
        with self.assertRaises(ValueError):
            validate_options({'solver': 'fast'})
        self.assertEqual(validate_options({'solver': 'Newton', 'iterations': 30}), {
                         'solver': 'Newton', 'iterations': 30})

    def test_unavailable_is_not_zero(self):
        m = contact_metrics(None)
        self.assertIsNone(m['max_penetration_m'])
        self.assertIsNone(m['normal_force_sum_N'])

    def test_available_no_contact_is_zero(self):
        m = contact_metrics([])
        self.assertEqual(m['max_penetration_m'], 0)
        self.assertEqual(m['contact_count'], 0)

    def test_contact_force_sum_is_not_net_force(self):
        m = contact_metrics([{'distance_m': -.002,
                              'force_local_N_Nm': [3,
                                                   1,
                                                   0,
                                                   0,
                                                   0,
                                                   0]},
                             {'distance_m': .001,
                              'force_local_N_Nm': [4,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0]}])
        self.assertEqual(m['max_penetration_m'], .002)
        self.assertEqual(m['normal_force_sum_N'], 7)
        self.assertNotIn('net_force', m)
