"""Independent CPU processes for concurrency plumbing, not a GPU vector-env benchmark."""
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
from pathlib import Path
import time
from .profiles import make_backend, validate_profile, provenance
from .runner import run_episode, write_json


def validate_batch(seeds, workers):
    if type(workers) is not int or workers <= 0:
        raise ValueError('workers must be a positive integer')
    if not seeds or any(type(s) is not int or s < 0 for s in seeds):
        raise ValueError('nonnegative integer seeds required')


def batch_summary(rows, elapsed, workers):
    outcomes = {k: sum(r['task_outcome'] == k for r in rows)
                for k in ('success', 'failure', 'unknown')}
    total = sum(r['steps'] for r in rows)
    return {'episodes': len(rows), 'completed_episodes': sum(r['execution_completed'] for r in rows),
            'outcomes': outcomes, 'total_control_steps': total, 'wall_seconds': elapsed,
            'batch_control_steps_per_second': total / elapsed if elapsed > 0 else None,
            'workers_requested': workers, 'native_vectorized': False,
            'scope': 'independent CPU processes; end-to-end includes process startup, model loading, reset, stepping and output',
            'not_solver_only': True}


def _job(args):
    profile, path, seed, steps, warmup, mode, action = args
    try:
        result = run_episode(None, path, task_id=profile['task_id'], seed=seed, max_steps=steps,
                             warmup_steps=warmup, recording_mode=mode, action_mode=action, provenance=provenance(profile), backend_factory=lambda: make_backend(profile))
    except Exception as exc:
        result = {'execution_completed': False, 'task_outcome': 'unknown', 'steps': 0,
                  'error': {'type': type(exc).__name__, 'message': str(exc)}}
    return {'seed': seed, 'output': str(path), **result}


def run_batch(profile, output, *, seeds, workers=1, steps=100,
              warmup=0, mode='throughput', action='zero'):
    validate_profile(profile)
    validate_batch(seeds, workers)
    if steps <= 0 or warmup < 0:
        raise ValueError('Invalid step budget')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    jobs = [
        (profile,
         output /
         f'episode_{i:03d}_seed_{seed}',
         seed,
         steps,
         warmup,
         mode,
         action) for i,
        seed in enumerate(seeds)]
    rows = []
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers, mp_context=multiprocessing.get_context('spawn')) as pool:
        futures = {pool.submit(_job, j): i for i, j in enumerate(jobs)}
        for future in as_completed(futures):
            i = futures[future]
            try:
                r = future.result()
            except Exception as exc:
                r = {
                    'seed': seeds[i],
                    'execution_completed': False,
                    'task_outcome': 'unknown',
                    'steps': 0,
                    'error': str(exc)}
            rows.append({'episode_index': i, **r})
    rows.sort(key=lambda r: r['episode_index'])
    report = {**batch_summary(rows, time.perf_counter() - started, workers), 'results': rows, 'profile': profile,
              'mode': mode, 'action_mode': action, 'seeds': seeds, 'warmup_steps': warmup, 'requested_steps': steps}
    write_json(output / 'batch-summary.json', report)
    return report
