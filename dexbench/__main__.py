import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from .catalog import load_tasks, validate_tasks, readiness_gaps


def main():
    p = argparse.ArgumentParser(
        description='Task catalog and explicitly supported source-task runs')
    p.add_argument(
        'command',
        choices=[
            'list',
            'audit',
            'run',
            'profiles',
            'batch',
            'compare',
            'assets'])
    p.add_argument(
        '--root',
        type=Path,
        default=Path(__file__).resolve().parents[1])
    p.add_argument('--json', action='store_true')
    p.add_argument('--profile', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--steps', type=int, default=100)
    p.add_argument('--warmup', type=int, default=0)
    p.add_argument('--action', choices=['zero', 'random'], default='zero')
    p.add_argument(
        '--actions',
        type=Path,
        help='JSON array of normalized source actions')
    p.add_argument('--contacts', action='store_true')
    p.add_argument(
        '--mode',
        choices=[
            'diagnostic',
            'throughput'],
        default='diagnostic')
    p.add_argument('--video', action='store_true')
    p.add_argument('--seeds', default='0,1,2')
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--left', type=Path)
    p.add_argument('--right', type=Path)
    p.add_argument('--manifest', type=Path)
    p.add_argument(
        '--fetch',
        action='store_true',
        help='Fetch manifest files with known SHA256; no overwrite')
    a = p.parse_args()
    if a.command == 'assets':
        from .assets import verify_asset, fetch_asset
        if a.manifest is None:
            p.error('assets requires --manifest')
        try:
            manifest = json.loads(a.manifest.read_text())
            rows = []
            for entry in manifest['files']:
                if 'local_path' not in entry:
                    rows.append(
                        {'path': entry['path'], 'status': 'unresolved_manifest'})
                    continue
                rows.append(
                    fetch_asset(
                        entry,
                        a.root) if a.fetch else verify_asset(
                        entry,
                        a.root))
            print(json.dumps(rows, ensure_ascii=False, indent=2))
            return 0 if all(r['status'] == 'verified' for r in rows) else 1
        except (OSError, ValueError, KeyError) as exc:
            print(str(exc), file=sys.stderr)
            return 2
    if a.command == 'compare':
        from .compare import compare_runs
        if a.left is None or a.right is None:
            p.error('compare requires --left and --right')
        try:
            report = compare_runs(a.left, a.right)
        except (OSError, ValueError, KeyError) as exc:
            print(str(exc), file=sys.stderr)
            return 2
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    if a.command == 'batch':
        from .profiles import load_profile
        from .batch import run_batch
        if a.profile is None or a.output is None:
            p.error('batch requires --profile and --output')
        if a.contacts or a.video or a.actions:
            p.error('batch does not support video, contact traces or replay actions')
        try:
            profile = load_profile(a.profile)
            report = run_batch(profile, a.output, seeds=[int(s) for s in a.seeds.split(',')], workers=a.workers,
                               steps=a.steps, warmup=a.warmup, mode=a.mode, action=a.action)
        except (OSError, ValueError) as exc:
            print(str(exc), file=sys.stderr)
            return 2
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report['completed_episodes'] == report['episodes'] else 1
    if a.command == 'profiles':
        for path in sorted((a.root / 'profiles').glob('*.json')):
            print(path.name)
        return 0
    if a.command == 'run':
        if a.profile is None or a.output is None:
            p.error('run requires --profile and --output')
        try:
            return run_command(a)
        except (OSError, ValueError, ImportError) as exc:
            print(str(exc), file=sys.stderr)
            return 2
    try:
        tasks = load_tasks(a.root)
        errors = validate_tasks(tasks)
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 2
    if a.command == 'list':
        if a.json:
            print(json.dumps(tasks, ensure_ascii=False, indent=2))
        else:
            for t in tasks:
                print(
                    f"{t['id']:4} {t['category']:14} {t['status']:18} {t['name']}")
    else:
        report = {'catalog_valid': True, 'task_count': len(tasks), 'categories': dict(Counter(t['category'] for t in tasks)),
                  'statuses': dict(Counter(t['status'] for t in tasks)),
                  'tasks': [{'id': t['id'], 'missing_for_full_readiness': readiness_gaps(t)} for t in tasks],
                  'note': 'Valid catalog does not mean runnable, migrated or successful tasks; declared evidence is not independently executed by this checker'}
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


def run_command(a):
    from .profiles import load_profile, make_backend, provenance
    from .runner import run_episode
    from .probes import ContactEnvelope
    profile = load_profile(a.profile)
    tasks = load_tasks(a.root)
    if profile['task_id'] not in {t['id'] for t in tasks}:
        raise ValueError('Profile task is absent from catalog')
    if a.output.exists():
        raise FileExistsError(f'Output already exists: {a.output}')
    if a.steps <= 0 or a.warmup < 0:
        raise ValueError('Invalid step budget')
    if a.mode == 'throughput' and (a.video or a.contacts):
        raise ValueError('Throughput excludes video/contact diagnostics')
    actions = json.loads(a.actions.read_text()) if a.actions else None
    if actions is not None and not isinstance(actions, list):
        raise ValueError('Actions must be a JSON list')
    writer = None

    def frame(b):
        nonlocal writer
        import imageio.v2 as imageio
        if writer is None:
            writer = imageio.get_writer(
                a.output / 'video.mp4',
                fps=1.0 / b.metadata()['control_dt'])
        writer.append_data(b.render())

    def finalize():
        if writer is not None:
            writer.close()

    report = run_episode(None, a.output, task_id=profile['task_id'], seed=a.seed, max_steps=a.steps,
                         action_mode=a.action, actions=actions, contacts=a.contacts, warmup_steps=a.warmup,
                         recording_mode=a.mode, provenance=provenance(profile), frame_callback=frame if a.video else None,
                         probes=[ContactEnvelope()] if a.contacts else [],
                         backend_factory=lambda: make_backend(profile, render=a.video), recording_finalize=finalize)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['execution_completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
