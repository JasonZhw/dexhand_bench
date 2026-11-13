import json
import tempfile
import unittest
from pathlib import Path
from dexbench.compare import compare_runs


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def run_data(self, name, offset=0., dt=.02, actions=None):
        p = self.root / name
        p.mkdir()
        manifest = {
            'task_id': 'H01',
            'seed': 0,
            'provenance': {
                'profile': {
                    'protocol': 'source-final-predicate-v1'}},
            'backend': {
                'environment_id': 'test',
                'control_dt': dt,
                'body_names': ['object'],
                'asset_hashes': {
                    'a': 'hash'}}}
        (p / 'manifest.json').write_text(json.dumps(manifest))
        (p /
         'summary.json').write_text(json.dumps({'execution_completed': True}))
        (p / 'actions.json').write_text(json.dumps(actions or [[0], [0]]))
        rows = [{'step_index': i + 1,
                 'state': {'sim_time': (i + 1) * dt,
                           'body_position_world_m': [[offset,
                                                      0,
                                                      0]],
                           'body_quaternion_wxyz': [[1,
                                                     0,
                                                     0,
                                                     0]]}} for i in range(2)]
        (p / 'steps.jsonl').write_text('\n'.join(json.dumps(x) for x in rows))
        return p

    def test_translation_difference_is_not_accuracy_claim(self):
        r = compare_runs(self.run_data('a'), self.run_data('b', offset=.01))
        self.assertAlmostEqual(
            r['bodies']['object']['max_position_difference_m'], .01)
        self.assertEqual(
            r['interpretation'],
            'simulation_consistency_not_physical_accuracy')

    def test_changed_time_step_cannot_compare_by_index(self):
        with self.assertRaises(ValueError):
            compare_runs(self.run_data('a'), self.run_data('b', dt=.03))

    def test_different_actions_rejected(self):
        with self.assertRaises(ValueError):
            compare_runs(
                self.run_data('a'),
                self.run_data(
                    'b',
                    actions=[
                        [1],
                        [0]]))

    def test_missing_trace_is_explicit(self):
        a = self.run_data('a')
        b = self.run_data('b')
        (b / 'steps.jsonl').unlink()
        with self.assertRaises(ValueError):
            compare_runs(a, b)

    def test_quaternion_sign_does_not_change_orientation(self):
        a = self.run_data('a')
        b = self.run_data('b')
        p = b / 'steps.jsonl'
        p.write_text(p.read_text().replace('[1, 0, 0, 0]', '[-1, 0, 0, 0]'))
        r = compare_runs(a, b)
        self.assertEqual(r['bodies']['object']
                         ['max_orientation_difference_rad'], 0)

    def test_same_seed_does_not_hide_changed_initial_state(self):
        a = self.run_data('a')
        b = self.run_data('b')
        (a /
         'initial_state.json').write_text(json.dumps({'qpos': [0.], 'qvel': [0.], 'body_position_world_m': [[0, 0, 0]], 'body_quaternion_wxyz': [[1, 0, 0, 0]]}))
        (b /
         'initial_state.json').write_text(json.dumps({'qpos': [.1], 'qvel': [0.], 'body_position_world_m': [[0, 0, 0]], 'body_quaternion_wxyz': [[1, 0, 0, 0]]}))
        r = compare_runs(a, b)
        self.assertFalse(r['initial_conditions']['recorded_state_identical'])
        self.assertTrue(r['initial_conditions']['confounded_by_reset'])

    def test_malformed_poses_rejected(self):
        for i, value in enumerate(
                ([float('nan'), 0, 0, 0], [float('inf'), 0, 0, 0], [1, 0, 0], [0, 0, 0, 0])):
            with self.subTest(value=value):
                a, b = self.run_data(f'a{i}'), self.run_data(f'b{i}')
                path = b / 'steps.jsonl'
                rows = [json.loads(x) for x in path.read_text().splitlines()]
                rows[0]['state']['body_quaternion_wxyz'] = [value]
                path.write_text('\n'.join(json.dumps(x) for x in rows))
                with self.assertRaises(ValueError):
                    compare_runs(a, b)

    def test_missing_initial_measurements_are_unknown(self):
        a, b = self.run_data('a'), self.run_data('b')
        for p in (a, b):
            (p / 'initial_state.json').write_text('{}')
        result = compare_runs(a, b)
        self.assertIsNone(result['initial_conditions']
                          ['recorded_state_identical'])

    def test_pose_shape_and_body_count_rejected(self):
        for i, value in enumerate(
                ([[0, 0]], [], [[0, 0, float('nan')]], [[0, 0, 0], [1, 1, 1]])):
            with self.subTest(value=value):
                a, b = self.run_data(f'a{i}'), self.run_data(f'b{i}')
                p = b / 'steps.jsonl'
                rows = [json.loads(x) for x in p.read_text().splitlines()]
                rows[0]['state']['body_position_world_m'] = value
                p.write_text('\n'.join(json.dumps(x) for x in rows))
                with self.assertRaises(ValueError):
                    compare_runs(a, b)
