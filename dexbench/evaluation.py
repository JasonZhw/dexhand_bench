"""Evaluate declared observations, without loading a simulator or a policy.

Geometric features and source-helper booleans must be supplied by an adapter.
This module does not claim that an unverified adapter reproduces the source.
"""
import argparse
import json
import math
import operator
from pathlib import Path


OPS = {'lt': operator.lt, 'le': operator.le, 'gt': operator.gt,
       'ge': operator.ge, 'eq': operator.eq}


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_spec(spec):
    """Return human-readable errors; reject ambiguous or empty criteria."""
    errors = []
    if not isinstance(spec, dict):
        return ['benchmark must be an object']
    signals = spec.get('signals')
    if not isinstance(signals, dict) or not signals:
        return ['signals must be a nonempty object']
    for name, info in signals.items():
        if not isinstance(info, dict) or info.get('type') not in ('number', 'boolean'):
            errors.append(f'{name}: invalid signal type')
            continue
        if any(not isinstance(info.get(k), str) or not info[k].strip() for k in ('unit', 'definition')):
            errors.append(f'{name}: unit and definition required')
    if not isinstance(spec.get('protocol_version'), str) or not spec['protocol_version'].strip() or spec.get('status') != 'draft':
        errors.append('explicit draft protocol version required')

    def rule_check(rule):
        if not isinstance(rule, dict):
            errors.append('rule must be an object')
            return
        groups = [key for key in ('all', 'any') if key in rule]
        if groups:
            if len(rule) != 1 or not isinstance(rule[groups[0]], list) or not rule[groups[0]]:
                errors.append('all/any requires a nonempty rule list')
                return
            for child in rule[groups[0]]:
                rule_check(child)
            return
        if set(rule) != {'signal', 'op', 'value'}:
            errors.append('leaf requires signal, op, value only')
            return
        if not isinstance(rule['signal'], str) or not isinstance(rule['op'], str):
            errors.append('signal and operator must be strings')
            return
        info = signals.get(rule['signal'], {})
        if not isinstance(info, dict):
            info = {}
        if rule['op'] not in OPS or not info:
            errors.append('unknown signal or operator')
        elif info.get('type') == 'boolean':
            if rule['op'] != 'eq' or type(rule['value']) is not bool:
                errors.append('boolean rule requires eq and a boolean value')
        elif not finite_number(rule['value']):
            errors.append('numeric rule requires a finite numeric threshold')

    success = spec.get('success', {})
    if not isinstance(success, dict):
        return errors + ['success must be an object']
    mode = success.get('mode')
    if mode not in ('final', 'hold', 'sequence'):
        errors.append('invalid success mode')
    allowed = {'mode', 'rule'}
    if mode == 'hold':
        allowed.add('hold_s')
    elif mode == 'sequence':
        allowed.add('sequence')
    if set(success) - allowed:
        errors.append('success contains options unrelated to its mode')
    rule_check(success.get('rule'))
    if mode == 'hold' and (not finite_number(success.get('hold_s')) or success['hold_s'] <= 0):
        errors.append('hold duration must be positive')
    if mode == 'sequence':
        sequence = success.get('sequence')
        if not isinstance(sequence, list) or not sequence:
            errors.append('sequence requires ordered rules')
        else:
            for rule in sequence:
                rule_check(rule)
    if spec.get('initial') is not None:
        rule_check(spec['initial'])
    for section in ('failures', 'stages', 'metrics'):
        items = spec.get(section)
        if not isinstance(items, list):
            errors.append(f'{section} must be a list')
            continue
        names = set()
        for item in items:
            if not isinstance(item, dict) or not isinstance(item.get('name'), str) or not item['name'].strip() or item['name'] in names:
                errors.append(f'{section}: each item needs a unique name')
                continue
            names.add(item['name'])
            if section != 'metrics':
                rule_check(item.get('rule'))
            elif not isinstance(item.get('signal'), str) or item['signal'] not in signals or item.get('reduce') not in ('final', 'min', 'max'):
                errors.append('invalid metric signal or reduction')
    episode = spec.get('episode', {})
    for key in ('timeout_s', 'max_sample_gap_s'):
        if not isinstance(episode, dict) or not finite_number(episode.get(key)) or episode[key] <= 0:
            errors.append(f'{key} must be positive seconds')
    return errors


def rule_signals(rule):
    if 'signal' in rule:
        return {rule['signal']}
    return set().union(*(rule_signals(child) for children in rule.values() for child in children))


def check(rule, values):
    if 'signal' in rule:
        value = values.get(rule['signal'])
        return None if value is None else OPS[rule['op']](value, rule['value'])
    key = next(iter(rule))
    results = [check(child, values) for child in rule[key]]
    if key == 'all' and False in results:
        return False
    if key == 'any' and True in results:
        return True
    if None in results:
        return None
    return all(results) if key == 'all' else any(results)


def evaluate(task_id, spec, trace):
    errors = validate_spec(spec)
    if errors:
        raise ValueError('; '.join(errors))
    if not isinstance(trace, dict) or trace.get('task_id') != task_id or trace.get('protocol_version') != spec['protocol_version']:
        raise ValueError('Trace task_id/protocol_version does not match configuration')
    if trace.get('kind') not in ('synthetic', 'recorded') or type(trace.get('complete')) is not bool:
        raise ValueError('Trace requires kind=synthetic/recorded and boolean complete')
    frames = trace.get('frames')
    if not isinstance(frames, list) or not frames:
        raise ValueError('Trace requires nonempty frames')
    previous = None
    for frame in frames:
        if not isinstance(frame, dict) or not finite_number(frame.get('t_s')) or frame['t_s'] < 0:
            raise ValueError('Frame requires finite nonnegative time')
        now = frame['t_s']
        if previous is None and now != 0:
            raise ValueError('Trace must start at episode time zero')
        if previous is not None and (now <= previous or now - previous > spec['episode']['max_sample_gap_s'] + 1e-9):
            raise ValueError('Times must increase without exceeding max_sample_gap_s')
        previous = now
        if now > spec['episode']['timeout_s'] + 1e-9:
            raise ValueError('Trace extends past the configured episode deadline')
        values = frame.get('signals')
        if not isinstance(values, dict):
            raise ValueError('Frame signals must be an object')
        for name, value in values.items():
            if name not in spec['signals']:
                raise ValueError(f'Undeclared signal: {name}')
            if value is None:
                continue
            kind = spec['signals'][name]['type']
            if (kind == 'boolean' and type(value) is not bool) or (kind == 'number' and not finite_number(value)):
                raise ValueError(f'Invalid value for {name}')

    success_cfg = spec['success']
    required = rule_signals(success_cfg['rule'])
    for item in spec['failures']:
        required |= rule_signals(item['rule'])
    for rule in success_cfg.get('sequence', []):
        required |= rule_signals(rule)
    missing = {name for frame in frames for name in required if frame['signals'].get(name) is None}
    initial = check(spec['initial'], frames[0]['signals']) if spec.get('initial') else True
    if spec.get('initial'):
        missing |= {name for name in rule_signals(spec['initial']) if frames[0]['signals'].get(name) is None}
    failure = None
    failure_time = None
    hold_start = None
    sequence_index = 0
    completion = None
    passed = False
    for frame in frames:
        now, values = frame['t_s'], frame['signals']
        for item in spec['failures']:
            if failure is None and check(item['rule'], values) is True:
                failure, failure_time = item['name'], now
        condition = check(success_cfg['rule'], values)
        mode = success_cfg['mode']
        if mode == 'hold':
            hold_start = (now if hold_start is None else hold_start) if condition is True else None
            passed = hold_start is not None and now - hold_start + 1e-9 >= success_cfg['hold_s']
        elif mode == 'sequence':
            sequence = success_cfg['sequence']
            # At most one transition per observed frame; no fabricated simultaneous cycle.
            if sequence_index < len(sequence) and check(sequence[sequence_index], values) is True:
                sequence_index += 1
            passed = sequence_index == len(sequence) and condition is True
        else:
            passed = condition is True
        if passed and completion is None:
            completion = now
        if not passed:
            completion = None

    success = passed
    reason = None
    if initial is not True:
        success, reason = None, 'invalid_initial_state' if initial is False else 'missing_initial_state'
    elif failure:
        success, reason = False, failure
    elif missing or not trace['complete']:
        success, reason = None, 'missing_observation' if missing else 'incomplete_trace'
    elif not success:
        reason = 'timeout' if frames[-1]['t_s'] >= spec['episode']['timeout_s'] else 'criteria_not_met'
    metrics = {}
    for metric in spec['metrics']:
        values = [frame['signals'].get(metric['signal']) for frame in frames]
        if metric['reduce'] == 'final':
            value = values[-1]
        elif any(v is None for v in values):
            value = None
        else:
            value = (min if metric['reduce'] == 'min' else max)(values)
        metrics[metric['name']] = value
    stages = {stage['name']: check(stage['rule'], frames[-1]['signals']) for stage in spec['stages']}
    return {
        'task_id': task_id, 'protocol_version': spec['protocol_version'],
        'input_kind': trace['kind'], 'evaluation_scope': 'declared_observations_only',
        'success': success, 'completion_time_s': completion if success is True else None,
        'failure_reason': reason, 'first_failure_time_s': failure_time,
        'missing_signals': sorted(missing), 'metrics': metrics,
        'terminal_stages': stages,
        'terminal_stage_fraction': sum(v is True for v in stages.values()) / len(stages)
        if stages and all(v is not None for v in stages.values()) else None,
        'official_source_score': None,
        'note': 'Draft protocol result; synthetic input is not a robot result. Stage fraction is not a source benchmark score.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('audit', 'score'))
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--task')
    parser.add_argument('--trace', type=Path)
    args = parser.parse_args()
    from .catalog import load_tasks
    try:
        tasks = load_tasks(args.root)
        if args.command == 'audit':
            results = [{'task_id': t['id'], 'errors': validate_spec(t.get('benchmark'))} for t in tasks]
            print(json.dumps({'task_count': len(tasks), 'tasks': results,
                              'note': 'Configuration validation only; not scene or policy readiness'}, ensure_ascii=False, indent=2))
            return 2 if any(row['errors'] for row in results) else 0
        if args.task is None or args.trace is None:
            parser.error('score requires --task and --trace')
        task = next((t for t in tasks if t['id'] == args.task), None)
        if task is None:
            raise ValueError('Unknown task ID')
        result = evaluate(task['id'], task['benchmark'], json.loads(args.trace.read_text()))
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(2, str(exc) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
