import json
from pathlib import Path
import tempfile
import unittest
from dexbench.runner import run_episode


class CounterBackend:
    """Deterministic contract fixture; never a production simulation backend."""

    def __init__(self, success=None, end_at=None, crash_at=None):
        self.success = success
        self.end_at = end_at
        self.crash_at = crash_at
        self.closed = False

    def metadata(self): return {
        'backend': 'test_fixture',
        'physics_dt': 0.01,
        'control_dt': 0.02,
        'capabilities': {
            'contacts': False}}

    def reset(self, seed): self.n = 0; return {'obs': [seed]}
    def action(self, mode): return [0.0]

    def step(self, action):
        self.n += 1
        if self.n == self.crash_at:
            raise RuntimeError('simulation error')
        return {'reward': 0., 'terminated': False, 'truncated': self.n ==
                self.end_at, 'success': self.success, 'applied_action': list(action)}

    def snapshot(
        self,
        contacts=False): return {
        'sim_time': self.n * .02,
        'qpos': [
            self.n],
        'contacts': None,
        'contacts_unavailable_reason': 'unsupported'}

    def synchronize(self): pass
    def close(self): self.closed = True
    def render(self): return None


class RunnerTests(unittest.TestCase):
    def run_case(self, backend, **kwargs):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'run'
        return run_episode(backend, self.path, task_id='H01',
                           seed=7, max_steps=3, **kwargs)

    def test_bounded_smoke_is_not_failure(self):
        r = self.run_case(CounterBackend(False))
        self.assertTrue(r['execution_completed'])
        self.assertEqual(r['task_outcome'], 'unknown')
        self.assertEqual(r['termination_reason'], 'step_budget')

    def test_source_horizon_with_false_is_failure(self):
        r = self.run_case(CounterBackend(False, end_at=2))
        self.assertEqual(r['task_outcome'], 'failure')
        self.assertEqual(r['steps'], 2)

    def test_unknown_success_stays_unknown(self):
        r = self.run_case(CounterBackend(None, end_at=2))
        self.assertEqual(r['task_outcome'], 'unknown')

    def test_final_predicate_true_is_explicit_success(self):
        r = self.run_case(CounterBackend(True))
        self.assertEqual(r['task_outcome'], 'success')
        self.assertEqual(
            r['outcome_rule'],
            'final_sample_of_source_success_predicate')

    def test_missing_contacts_remain_null(self):
        self.run_case(CounterBackend(), contacts=True)
        rows = [
            json.loads(x) for x in (
                self.path /
                'steps.jsonl').read_text().splitlines()]
        self.assertIsNone(rows[0]['state']['contacts'])
        self.assertEqual(rows[0]['step_index'], 1)

    def test_crash_writes_incomplete_result_and_closes(self):
        b = CounterBackend(crash_at=2)
        r = self.run_case(b)
        self.assertFalse(r['execution_completed'])
        self.assertTrue(b.closed)
        self.assertEqual(r['steps'], 1)
        self.assertEqual(r['task_outcome'], 'unknown')
        self.assertIn(
            'simulation error',
            (self.path / 'summary.json').read_text())

    def test_existing_output_not_overwritten(self):
        self.run_case(CounterBackend())
        with self.assertRaises(FileExistsError):
            run_episode(
                CounterBackend(),
                self.path,
                task_id='H01',
                seed=0,
                max_steps=1)

    def test_invalid_budget_rejected(self):
        with self.assertRaises(ValueError):
            self.run_case(CounterBackend(), warmup_steps=-1)

    def test_warmup_is_reset_before_measured_episode(self):
        r = self.run_case(CounterBackend(), warmup_steps=2)
        self.assertEqual(r['steps'], 3)
        self.assertEqual(
            json.loads(
                (self.path / 'initial_state.json').read_text())['qpos'],
            [0])

    def test_replay_length_limits_execution_without_success_invention(self):
        r = self.run_case(CounterBackend(False), actions=[[.2], [.4]])
        self.assertEqual(r['steps'], 2)
        self.assertEqual(r['termination_reason'], 'action_sequence_exhausted')
        self.assertEqual(r['task_outcome'], 'unknown')

    def test_throughput_does_not_emit_state_trace(self):
        r = self.run_case(CounterBackend(), recording_mode='throughput')
        self.assertFalse((self.path / 'steps.jsonl').exists())
        self.assertIn('step_and_synchronize', r['timing']['scope'])

    def test_probe_receives_states_and_saves_results(self):
        class CountProbe:
            name = 'count'
            requires_contacts = False
            def reset(self, state): self.count = 0
            def observe(self, state, transition): self.count += 1
            def result(self): return {'samples': self.count}
        r = self.run_case(CounterBackend(), probes=[CountProbe()])
        self.assertEqual(r['diagnostic_probes']['count']['samples'], 3)

    def test_transient_success_does_not_override_final_failure(self):
        class Flicker(CounterBackend):
            def step(self, action):
                result = super().step(action)
                result['success'] = self.n == 1
                return result
        r = self.run_case(Flicker(end_at=2))
        self.assertTrue(r['any_source_success'])
        self.assertFalse(r['final_source_success'])
        self.assertEqual(r['task_outcome'], 'failure')

    def test_initialization_failure_leaves_evidence(self):
        def factory(): raise RuntimeError('bad XML')
        r = self.run_case(
            None,
            backend_factory=factory,
            provenance={
                'profile': {
                    'name': 'requested'}})
        self.assertFalse(r['execution_completed'])
        self.assertEqual(r['steps'], 0)
        manifest = json.loads((self.path / 'manifest.json').read_text())
        self.assertIsNone(manifest['backend'])
        self.assertEqual(
            manifest['provenance']['profile']['name'],
            'requested')
        self.assertIn('bad XML', (self.path / 'error.txt').read_text())

    def test_recording_finalize_failure_is_incomplete(self):
        def finish(): raise RuntimeError('encoder close failed')
        r = self.run_case(CounterBackend(True), recording_finalize=finish)
        self.assertFalse(r['execution_completed'])
        self.assertEqual(r['task_outcome'], 'unknown')
        self.assertIn(
            'encoder close failed',
            (self.path / 'error.txt').read_text())
