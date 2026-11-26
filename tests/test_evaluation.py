import json
from pathlib import Path
import unittest

from dexbench.evaluation import evaluate, validate_spec


ROOT = Path(__file__).resolve().parents[1]


def spec():
    return {
        'protocol_version': '0.1-draft',
        'status': 'draft',
        'signals': {
            'height_m': {'type': 'number', 'unit': 'm', 'definition': 'Height gain'},
            'dropped': {'type': 'boolean', 'unit': 'bool', 'definition': 'Drop event'},
        },
        'initial': None,
        'success': {'mode': 'hold', 'hold_s': 1.0, 'rule':
                    {'signal': 'height_m', 'op': 'ge', 'value': 0.05}},
        'failures': [{'name': 'drop', 'rule':
                      {'signal': 'dropped', 'op': 'eq', 'value': True}}],
        'stages': [],
        'metrics': [{'name': 'height_final', 'signal': 'height_m', 'reduce': 'final'}],
        'episode': {'timeout_s': 10.0, 'max_sample_gap_s': 0.5},
    }


def trace(values, task='TEST'):
    return {'task_id': task, 'protocol_version': '0.1-draft',
            'kind': 'synthetic', 'complete': True,
            'frames': [{'t_s': i * 0.5, 'signals': {'height_m': v, 'dropped': False}}
                       for i, v in enumerate(values)]}


class EvaluationTests(unittest.TestCase):
    def test_hold_requires_duration_and_resets_after_break(self):
        self.assertFalse(evaluate('TEST', spec(), trace([0.05, 0.05]))['success'])
        result = evaluate('TEST', spec(), trace([0.05, 0.05, 0.05]))
        self.assertTrue(result['success'])
        self.assertEqual(result['completion_time_s'], 1.0)
        self.assertFalse(evaluate('TEST', spec(), trace([0.1, 0.1, 0.0, 0.1]))['success'])

    def test_terminal_failure_and_known_failure_override_lift(self):
        data = trace([0.1, 0.1, 0.1, 0.0])
        self.assertFalse(evaluate('TEST', spec(), data)['success'])
        data = trace([0.1, 0.1, 0.1])
        data['frames'][1]['signals']['dropped'] = True
        result = evaluate('TEST', spec(), data)
        self.assertFalse(result['success'])
        self.assertEqual(result['failure_reason'], 'drop')

    def test_missing_required_signal_is_unknown_not_pass_or_zero(self):
        data = trace([0.1, 0.1, 0.1])
        del data['frames'][1]['signals']['dropped']
        result = evaluate('TEST', spec(), data)
        self.assertIsNone(result['success'])
        self.assertIn('dropped', result['missing_signals'])
        data['complete'] = False
        self.assertIsNone(evaluate('TEST', spec(), data)['success'])

    def test_invalid_or_sparse_data_rejected(self):
        for edit in ['nan', 'bool_number', 'backward', 'gap', 'wrong_task', 'wrong_version', 'empty']:
            data = trace([0.1, 0.1, 0.1])
            if edit == 'nan': data['frames'][1]['signals']['height_m'] = float('nan')
            if edit == 'bool_number': data['frames'][1]['signals']['height_m'] = True
            if edit == 'backward': data['frames'][1]['t_s'] = 0.0
            if edit == 'gap': data['frames'][1]['t_s'] = 0.8
            if edit == 'wrong_task': data['task_id'] = 'OTHER'
            if edit == 'wrong_version': data['protocol_version'] = 'unknown'
            if edit == 'empty': data['frames'] = []
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                evaluate('TEST', spec(), data)

    def test_strict_threshold_and_initial_state(self):
        cfg = spec(); cfg['success'] = {'mode': 'final', 'rule': {'signal': 'height_m', 'op': 'lt', 'value': 0.05}}
        self.assertFalse(evaluate('TEST', cfg, trace([0.05]))['success'])
        self.assertTrue(evaluate('TEST', cfg, trace([0.049]))['success'])
        cfg['initial'] = {'signal': 'height_m', 'op': 'eq', 'value': 0.0}
        result = evaluate('TEST', cfg, trace([0.049]))
        self.assertIsNone(result['success'])
        self.assertEqual(result['failure_reason'], 'invalid_initial_state')

    def test_sequence_requires_open_then_close_and_terminal_close(self):
        cfg = spec(); cfg['failures'] = []
        opened = {'signal': 'height_m', 'op': 'ge', 'value': 1.0}
        closed = {'signal': 'height_m', 'op': 'lt', 'value': 0.52}
        cfg['initial'] = closed
        cfg['success'] = {'mode': 'sequence', 'rule': closed, 'sequence': [opened, closed]}
        self.assertFalse(evaluate('TEST', cfg, trace([0.0, 0.0]))['success'])
        self.assertTrue(evaluate('TEST', cfg, trace([0.0, 1.0, 0.5]))['success'])
        self.assertFalse(evaluate('TEST', cfg, trace([0.0, 1.0, 0.5, 1.0]))['success'])

    def test_optional_metric_missing_stays_null_and_timeout_is_not_crash(self):
        cfg = spec(); cfg['signals']['force_n'] = {'type': 'number', 'unit': 'N', 'definition': 'Optional force'}
        cfg['metrics'].append({'name': 'force_peak', 'signal': 'force_n', 'reduce': 'max'})
        result = evaluate('TEST', cfg, trace([0.1, 0.1, 0.1]))
        self.assertTrue(result['success']); self.assertIsNone(result['metrics']['force_peak'])
        cfg['episode']['timeout_s'] = 1.0
        result = evaluate('TEST', cfg, trace([0.0, 0.0, 0.0]))
        self.assertEqual(result['failure_reason'], 'timeout')

    def test_validator_rejects_unknown_operator_empty_rule_and_bad_units(self):
        for change in ['op', 'empty', 'unit', 'threshold', 'duration']:
            cfg = spec()
            if change == 'op': cfg['success']['rule']['op'] = 'approx'
            if change == 'empty': cfg['success']['rule'] = {'all': []}
            if change == 'unit': cfg['signals']['height_m']['unit'] = ''
            if change == 'threshold': cfg['success']['rule']['value'] = True
            if change == 'duration': cfg['success']['hold_s'] = -1
            with self.subTest(change=change): self.assertTrue(validate_spec(cfg))

    def test_malformed_json_fields_report_validation_errors(self):
        for name in ['op', 'signal', 'unit', 'definition', 'name']:
            cfg = spec()
            if name in ('op', 'signal'): cfg['success']['rule'][name] = []
            elif name in ('unit', 'definition'): cfg['signals']['height_m'][name] = 7
            else: cfg['metrics'][0]['name'] = ['bad']
            with self.subTest(name=name): self.assertTrue(validate_spec(cfg))

    def test_success_mode_rejects_unrelated_options(self):
        for mode, extra in [('final', {'sequence': None}), ('final', {'hold_s': 1}),
                            ('hold', {'sequence': []}), ('sequence', {'hold_s': 1})]:
            cfg = spec()
            cfg['success'] = {'mode': mode, 'rule': cfg['success']['rule'], **extra}
            with self.subTest(mode=mode, extra=extra): self.assertTrue(validate_spec(cfg))


class CatalogEvaluationTests(unittest.TestCase):
    def test_every_task_has_auditable_draft_spec(self):
        paths = list((ROOT / 'tasks').rglob('*.json'))
        self.assertEqual(len(paths), 21)
        for path in paths:
            task = json.loads(path.read_text())
            with self.subTest(task=task['id']):
                cfg = task['benchmark']
                self.assertEqual(validate_spec(cfg), [])
                self.assertFalse(cfg['readiness']['live_adapter_verified'])
                self.assertTrue(cfg['provenance']['episode_protocol'])
                self.assertTrue(cfg['scene']['initial_state'])

    def test_source_boundaries_on_real_configs(self):
        tasks = {json.loads(p.read_text())['id']: json.loads(p.read_text())
                 for p in (ROOT / 'tasks').rglob('*.json')}
        pen = tasks['H02']['benchmark']
        sample = {'task_id': 'H02', 'protocol_version': pen['protocol_version'], 'kind': 'synthetic', 'complete': True,
                  'frames': [{'t_s': 0.0, 'signals': {'position_error_m': 0.074, 'axis_dot': 0.95}}]}
        self.assertFalse(evaluate('H02', pen, sample)['success'])
        sample['frames'][0]['signals']['axis_dot'] = 0.951
        self.assertTrue(evaluate('H02', pen, sample)['success'])
        sample['frames'][0]['signals']['position_error_m'] = 0.075
        self.assertFalse(evaluate('H02', pen, sample)['success'])

    def test_checked_in_examples_are_explicitly_synthetic(self):
        tasks = {json.loads(p.read_text())['id']: json.loads(p.read_text())
                 for p in (ROOT / 'tasks').rglob('*.json')}
        for path in (ROOT / 'examples/evaluation').glob('*.json'):
            data = json.loads(path.read_text())
            with self.subTest(example=path.name):
                self.assertEqual(data['kind'], 'synthetic')
                result = evaluate(data['task_id'], tasks[data['task_id']]['benchmark'], data)
                expected = None if 'unknown' in path.stem else 'pass' in path.stem
                self.assertIs(result['success'], expected)
                self.assertIsNone(result['official_source_score'])


if __name__ == '__main__':
    unittest.main()
