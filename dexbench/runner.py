"""One episode, explicit provenance and no invented task outcomes."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import traceback
from typing import Protocol


class Backend(Protocol):
    def metadata(self) -> dict: ...
    def reset(self, seed: int) -> dict: ...
    def action(self, mode: str): ...
    def step(self, action) -> dict: ...
    def snapshot(self, contacts: bool = False) -> dict: ...
    def synchronize(self) -> None: ...
    def close(self) -> None: ...


def write_json(path, data):
    Path(path).write_text(
        json.dumps(
            data,
            ensure_ascii=False,
            indent=2,
            allow_nan=False) +
        '\n',
        encoding='utf-8')


def run_episode(backend: Backend, output, *, task_id, seed, max_steps, action_mode='zero',
                actions=None, contacts=False, warmup_steps=0, recording_mode='diagnostic',
                provenance=None, frame_callback=None, probes=None, backend_factory=None, recording_finalize=None):
    probes = list(probes or [])
    if len({p.name for p in probes}) != len(probes):
        raise ValueError('Probe names must be unique')
    contacts = contacts or any(p.requires_contacts for p in probes)
    if type(max_steps) is not int or max_steps <= 0 or type(
            warmup_steps) is not int or warmup_steps < 0:
        raise ValueError(
            'max_steps must be positive and warmup_steps nonnegative integers')
    if recording_mode not in ('diagnostic', 'throughput'):
        raise ValueError('recording_mode must be diagnostic or throughput')
    if recording_mode == 'throughput' and (
            contacts or frame_callback or probes):
        raise ValueError(
            'throughput mode cannot request contacts or rendering')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    summary = {'schema_version': '0.2', 'task_id': task_id, 'seed': seed, 'steps': 0,
               'execution_completed': False, 'task_outcome': 'unknown', 'final_source_success': None,
               'any_source_success': False, 'termination_reason': None, 'error': None,
               'outcome_rule': 'final_sample_of_source_success_predicate', 'diagnostic_probes': {}}
    trace = None
    measured = []
    executed_actions = []
    try:
        manifest = {'created_utc': datetime.now(timezone.utc).isoformat(),
                    'task_id': task_id, 'seed': seed, 'max_steps': max_steps, 'warmup_steps': warmup_steps,
                    'action_mode': 'recorded' if actions is not None else action_mode,
                    'recording_mode': recording_mode, 'contacts_requested': contacts,
                    'backend': None, 'provenance': provenance or {}, 'probes': [p.name for p in probes]}
        write_json(output / 'manifest.json', manifest)
        if backend_factory is not None:
            backend = backend_factory()
        manifest['backend'] = backend.metadata()
        write_json(output / 'manifest.json', manifest)
        backend.reset(seed)
        for _ in range(warmup_steps):
            warm = backend.step(backend.action(action_mode))
            if warm['terminated'] or warm['truncated']:
                backend.reset(seed)
        backend.synchronize()
        # Warmup must never consume the scored episode or perturb its RNG
        # state.
        initial_observation = backend.reset(seed)
        initial_state = backend.snapshot(contacts=contacts)
        write_json(output / 'initial_state.json', initial_state)
        for probe in probes:
            probe.reset(initial_state)
        write_json(output / 'initial_observation.json', initial_observation)
        if recording_mode == 'diagnostic':
            trace = (output / 'steps.jsonl').open('x', encoding='utf-8')
        if frame_callback:
            frame_callback(backend)
        reason = 'step_budget'
        for index in range(max_steps):
            if actions is not None and index >= len(actions):
                reason = 'action_sequence_exhausted'
                break
            action = actions[index] if actions is not None else backend.action(
                action_mode)
            backend.synchronize()
            t0 = time.perf_counter()
            result = backend.step(action)
            backend.synchronize()
            measured.append(time.perf_counter() - t0)
            success = result.get('success')
            if success is not None and type(success) is not bool:
                raise TypeError('backend success must be bool or None')
            executed_actions.append(
                action.tolist() if hasattr(
                    action, 'tolist') else action)
            summary['steps'] = index + 1
            summary['final_source_success'] = success
            summary['any_source_success'] |= success is True
            state = backend.snapshot(
                contacts=contacts) if trace or probes else None
            for probe in probes:
                probe.observe(state, result)
            if trace:
                row = {'step_index': index + 1, 'requested_action': executed_actions[-1],
                       'transition': result, 'state': state}
                trace.write(
                    json.dumps(
                        row,
                        ensure_ascii=False,
                        allow_nan=False) +
                    '\n')
            if frame_callback:
                frame_callback(backend)
            if result['terminated'] or result['truncated']:
                reason = 'source_terminated' if result['terminated'] else 'source_time_limit'
                break
        summary['diagnostic_probes'] = {p.name: p.result() for p in probes}
        summary['termination_reason'] = reason
        summary['execution_completed'] = True
        if summary['final_source_success'] is True:
            summary['task_outcome'] = 'success'
        elif summary['final_source_success'] is False and reason in ('source_terminated', 'source_time_limit'):
            summary['task_outcome'] = 'failure'
    except Exception as exc:
        summary['error'] = {'type': type(exc).__name__, 'message': str(exc)}
        summary['termination_reason'] = 'exception'
        (output / 'error.txt').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        if trace:
            trace.close()
        if recording_finalize:
            try:
                recording_finalize()
            except Exception as exc:
                summary['recording_error'] = str(exc)
                summary['execution_completed'] = False
                summary['task_outcome'] = 'unknown'
                with (output / 'error.txt').open('a', encoding='utf-8') as error_file:
                    error_file.write(traceback.format_exc())
        try:
            if backend is not None:
                backend.close()
        except Exception as exc:
            summary['close_error'] = str(exc)
            summary['execution_completed'] = False
            summary['task_outcome'] = 'unknown'
        elapsed = time.perf_counter() - started
        summary['timing'] = {'scope': 'environment.step_and_synchronize_only; excludes reset, policy, snapshot, logging, render',
                             'step_seconds': sum(measured), 'steps_measured': len(measured),
                             'environment_steps_per_second': len(measured) / sum(measured) if sum(measured) > 0 else None,
                             'wall_seconds_including_setup_and_recording': elapsed,
                             'num_envs': 1, 'parallelism': 'single_environment'}
        write_json(output / 'actions.json', executed_actions)
        write_json(output / 'summary.json', summary)
    return summary
