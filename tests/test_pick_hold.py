import unittest

class PickHoldTests(unittest.TestCase):
    def test_lift_requires_every_sample_high_and_stable(self):
        from dexbench.pick import evaluate_hold
        self.assertTrue(evaluate_hold([[0, 0, .16], [0, 0, .161]], [0, 0, .1], .05, .01)['lift_observed'])
        self.assertFalse(evaluate_hold([[0, 0, .16], [0, 0, .12]], [0, 0, .1], .05, .01)['lift_observed'])
        self.assertFalse(evaluate_hold([[0, 0, .16], [.03, 0, .16]], [0, 0, .1], .05, .01)['lift_observed'])

    def test_missing_and_invalid_measurements_cannot_pass(self):
        from dexbench.pick import evaluate_hold
        for samples in ([], [[0, 0, float('nan')]], [[0, 0]], [[0, 0, float('inf')]]):
            with self.assertRaises(ValueError): evaluate_hold(samples, [0, 0, 0], .05, .01)

    def test_failed_numpy_hold_is_json_serializable(self):
        import json
        try:
            import numpy as np
        except ImportError:
            self.skipTest('Requires NumPy')
        from dexbench.pick import evaluate_hold
        result = evaluate_hold(np.array([[0., 0., .1]]), np.array([0., 0., .1]), .05, .01)
        self.assertIs(result['lift_observed'], False)
        json.dumps(result)

if __name__ == '__main__': unittest.main()
