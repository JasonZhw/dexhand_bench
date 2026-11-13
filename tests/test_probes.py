import unittest
from dexbench.probes import ContactEnvelope


class ProbeTests(unittest.TestCase):
    def test_unknown_contact_capability_does_not_produce_zero_error(self):
        p = ContactEnvelope()
        p.reset({})
        p.observe({'contacts': None}, {})
        self.assertIsNone(p.result()['max_penetration_m'])
        self.assertEqual(p.result()['samples_available'], 0)

    def test_observed_contacts_accumulate_envelope(self):
        p = ContactEnvelope()
        p.reset({})
        p.observe(
            {'contacts': [{'distance_m': -.001, 'force_local_N_Nm': [5, 0, 0, 0, 0, 0]}]}, {})
        p.observe({'contacts': []}, {})
        self.assertEqual(p.result()['max_penetration_m'], .001)
        self.assertEqual(p.result()['max_normal_force_sum_N'], 5)
        self.assertEqual(p.result()['samples_available'], 2)

    def test_reset_clears_previous_episode(self):
        p = ContactEnvelope()
        p.reset({})
        p.observe({'contacts': []}, {})
        p.reset({})
        self.assertEqual(p.result()['samples_available'], 0)
