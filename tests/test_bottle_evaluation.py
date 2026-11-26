"""Characterize the existing G02 contract using explicitly synthetic traces."""
import copy
import json
from pathlib import Path
import unittest

from dexbench.evaluation import evaluate

ROOT = Path(__file__).resolve().parents[1]


class BottleEvaluationTests(unittest.TestCase):
    def test_lift_alone_cannot_pass_without_upright_stable_terminal_hold(self):
        spec = json.loads((ROOT/'tasks/grasp/G02.json').read_text())['benchmark']
        trace = {'task_id': 'G02', 'protocol_version': spec['protocol_version'],
                 'kind': 'synthetic', 'complete': True, 'frames': []}
        for index in range(25):
            trace['frames'].append({'t_s': index/20, 'signals': {
                'height_gain_m': .06 if index else 0., 'speed_m_s': .01,
                'hand_contact': bool(index), 'dropped': False, 'tilt_deg': 15.}})
        self.assertTrue(evaluate('G02', spec, trace)['success'])
        for field, value in [('height_gain_m', .049), ('tilt_deg', 15.01),
                             ('speed_m_s', .051), ('hand_contact', False),
                             ('dropped', True)]:
            with self.subTest(field=field):
                failed = copy.deepcopy(trace)
                failed['frames'][-1]['signals'][field] = value
                self.assertFalse(evaluate('G02', spec, failed)['success'])
        short = copy.deepcopy(trace)
        short['frames'] = short['frames'][:20]
        self.assertFalse(evaluate('G02', spec, short)['success'])


if __name__ == '__main__':
    unittest.main()
