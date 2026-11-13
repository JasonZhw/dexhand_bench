import json
from pathlib import Path
import tempfile
import unittest
from dexbench.external_replay import summarize_recording

class ReplayEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.rows=[{'phase':'initial','t_s':0,'object_pose_wxyz':[0,0,0,1,0,0,0]},
                   {'phase':'replay','t_s':.01,'object_pose_wxyz':[0,0,.1,1,0,0,0]}]
    def prepare(self):
        (self.root/'manifest.json').write_text(json.dumps({'expected_steps':1}))
        (self.root/'completion.json').write_text(json.dumps({'completed':True,'recorded_replay_steps':1}))
        (self.root/'states.jsonl').write_text('\n'.join(json.dumps(r) for r in self.rows))
    def test_execution_is_not_success(self):
        self.prepare();r=summarize_recording(self.root,0)
        self.assertTrue(r['execution_completed']);self.assertEqual(r['task_outcome'],'unknown')
        self.assertAlmostEqual(r['final_origin_rise_m'],.1)
    def test_missing_receipt_or_nonzero_exit_is_incomplete(self):
        self.assertFalse(summarize_recording(self.root,1)['execution_completed'])
        self.prepare();self.assertFalse(summarize_recording(self.root,1)['execution_completed'])
    def test_nonfinite_state_is_incomplete(self):
        self.rows[-1]['object_pose_wxyz'][2]=float('nan');self.prepare()
        self.assertFalse(summarize_recording(self.root,0)['execution_completed'])
    def test_truncated_recording_is_incomplete(self):
        self.prepare();(self.root/'states.jsonl').write_text(json.dumps(self.rows[0]))
        self.assertFalse(summarize_recording(self.root,0)['execution_completed'])
    def test_bad_time_is_incomplete(self):
        self.rows[-1]['t_s']=-1;self.prepare()
        self.assertFalse(summarize_recording(self.root,0)['execution_completed'])

    def test_malformed_receipt_is_incomplete(self):
        self.prepare()
        for receipt in ([], None):
            (self.root/'completion.json').write_text(json.dumps(receipt))
            self.assertFalse(summarize_recording(self.root,0)['execution_completed'])

    def test_nonfinite_timeout_rejected(self):
        from unittest.mock import patch
        from dexbench.external_replay import main
        for value in ('nan','inf'):
            argv=['replay','--source-root','missing','--python','missing',
                  '--hdf5','missing','--scene','missing','--output','missing',
                  '--target-object','apple','--timeout',value]
            with patch('sys.argv',argv), patch('dexbench.external_replay.Path.is_file') as check:
                with self.assertRaises(SystemExit): main()
                check.assert_not_called()
