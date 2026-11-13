import unittest
from dexbench.batch import batch_summary, validate_batch


class BatchTests(unittest.TestCase):
    def test_batch_separates_unknown_and_execution_failure(self):
        rows = [{'execution_completed': True,
                 'task_outcome': 'success',
                 'steps': 10},
                {'execution_completed': True,
                 'task_outcome': 'unknown',
                 'steps': 10},
                {'execution_completed': False,
                 'task_outcome': 'unknown',
                 'steps': 3}]
        r = batch_summary(rows, 2., 2)
        self.assertEqual(
            r['outcomes'], {
                'success': 1, 'failure': 0, 'unknown': 2})
        self.assertEqual(r['completed_episodes'], 2)
        self.assertEqual(r['total_control_steps'], 23)
        self.assertEqual(r['batch_control_steps_per_second'], 11.5)
        self.assertFalse(r['native_vectorized'])

    def test_invalid_workers_rejected(self):
        for n in (0, -1, True, 1.5):
            with self.assertRaises(ValueError):
                validate_batch([0], n)

    def test_empty_seeds_rejected(self):
        with self.assertRaises(ValueError):
            validate_batch([], 1)
