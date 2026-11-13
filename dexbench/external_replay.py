"""Supervise an upstream replay; execution evidence is not a task success score."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys


def summarize_recording(directory, returncode):
    result = {'execution_completed': False, 'task_outcome': 'unknown',
              'returncode': returncode, 'final_origin_rise_m': None}
    try:
        root = Path(directory)
        manifest = json.loads((root / 'manifest.json').read_text())
        completion = json.loads((root / 'completion.json').read_text())
        if not isinstance(manifest, dict) or not isinstance(completion, dict):
            raise ValueError('Invalid manifest or receipt shape')
        rows = [json.loads(line) for line in (root / 'states.jsonl').read_text().splitlines()]
        expected = manifest['expected_steps']
        if type(expected) is not int or expected <= 0:
            raise ValueError('Invalid expected step count')
        clocks = {}
        def finite(value):
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('Nonfinite state')
            if isinstance(value, dict):
                for item in value.values(): finite(item)
            elif isinstance(value, list):
                for item in value: finite(item)
        for row in rows:
            finite(row)
            phase, time = row['phase'], row['t_s']
            if phase not in ('initial', 'warmup', 'replay') or type(time) not in (int, float) or time < 0:
                raise ValueError('Invalid phase/time')
            if phase in clocks and time <= clocks[phase]:
                raise ValueError('Nonincreasing phase clock')
            clocks[phase] = time
            pose = row['object_pose_wxyz']
            if len(pose) != 7 or any(type(x) not in (int, float) for x in pose):
                raise ValueError('Invalid pose')
        replay = [row for row in rows if row['phase'] == 'replay']
        if not rows or rows[0]['phase'] != 'initial' or len(replay) != expected:
            raise ValueError('Missing initial state or truncated recording')
        if sum(row['phase'] == 'initial' for row in rows) != 1:
            raise ValueError('Ambiguous initial state')
        if completion.get('completed') is not True or completion.get('recorded_replay_steps') != expected:
            raise ValueError('Incomplete receipt')
        result['execution_completed'] = returncode == 0
        result['recorded_replay_steps'] = len(replay)
        result['final_origin_rise_m'] = replay[-1]['object_pose_wxyz'][2] - rows[0]['object_pose_wxyz'][2]
    except (OSError, ValueError, TypeError, KeyError, IndexError) as exc:
        result['error'] = str(exc)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('source-root', 'python', 'hdf5', 'scene', 'output', 'target-object'):
        parser.add_argument('--' + key, required=True)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--max-frames', type=int, default=71)
    parser.add_argument('--timeout', type=float, default=300)
    args = parser.parse_args()
    if args.max_frames <= 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Frame limit and timeout must be positive')
    source = Path(args.source_root).resolve()
    script = Path(__file__).resolve().parents[1] / 'integrations/bench2dex/replay_recorded.py'
    for path in (source / 'tools/replay/replay_commanded.py', Path(args.python), Path(args.hdf5), Path(args.scene), script):
        if not path.is_file(): parser.error(f'Missing file: {path}')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    command = [str(Path(args.python).resolve()), str(script), '--hdf5', str(Path(args.hdf5).resolve()),
               '--scene', str(Path(args.scene).resolve()), '--record-dir', str(output / 'recording'),
               '--target-object', args.target_object, '--headless', '--device', 'cpu',
               '--restore-generalization', '--warmup-steps', '0', '--max-frames', str(args.max_frames)]
    if args.video:
        command.append('--video')
    # Keep the venv executable path: resolving a symlink can bypass its pyvenv.cfg.
    command[0] = str(Path(args.python).absolute())
    env = dict(os.environ, DEXBENCH_BENCH2DEX_ROOT=str(source))
    (output / 'invocation.json').write_text(json.dumps({'command': command, 'cwd': str(source),
        'timeout_s': args.timeout, 'integration': 'external_upstream_replay'}, indent=2))
    with (output / 'replay.log').open('x') as log:
        try:
            process = subprocess.run(command, cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            code = process.returncode
        except subprocess.TimeoutExpired:
            code = 124
        except OSError as exc:
            log.write(str(exc)); code = 127
    result = summarize_recording(output / 'recording', code)
    (output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['execution_completed'] else 1


if __name__ == '__main__':
    sys.exit(main())
