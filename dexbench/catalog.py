import json
from pathlib import Path

CATEGORIES = (
    'grasp',
    'transport',
    'push',
    'tool_rotation',
    'in_hand',
    'insertion')
STATUSES = (
    'selected',
    'source_reviewed',
    'scene_ready',
    'smoke_executed',
    'baseline_verified')
SOURCE_KINDS = (
    'original',
    'subtask',
    'adapted_subtask',
    'internal',
    'proposed')


def load_tasks(root):
    paths = sorted((Path(root) / 'tasks').rglob('*.json'))
    if not paths:
        raise ValueError('No task files found')
    return [json.loads(path.read_text(encoding='utf-8')) for path in paths]


def validate_tasks(tasks):
    errors = []
    seen = set()
    for index, t in enumerate(tasks):
        if not isinstance(t, dict):
            errors.append(f'row {index}: expected task object')
            continue
        ident = t.get('id')
        if not isinstance(ident, str) or not ident:
            errors.append(f'row {index}: missing string id')
            continue
        if ident in seen:
            errors.append(f'{ident}: duplicate id')
        seen.add(ident)
        if t.get('category') not in CATEGORIES:
            errors.append(f'{ident}: invalid category')
        if not isinstance(t.get('name'), str) or not t['name'].strip():
            errors.append(f'{ident}: missing name')
        status = t.get('status')
        if status not in STATUSES:
            errors.append(f'{ident}: invalid status')
        source = t.get('source')
        if not isinstance(source, dict):
            errors.append(f'{ident}: missing source')
            source = {}
        kind = source.get('kind')
        if kind not in SOURCE_KINDS:
            errors.append(f'{ident}: invalid source kind')
        if kind in ('original', 'subtask', 'adapted_subtask'):
            url = source.get('url')
            if not isinstance(url, str) or not url.startswith(
                    ('https://', 'http://')):
                errors.append(f'{ident}: source URL required')
        level = STATUSES.index(status) if status in STATUSES else 0
        if level >= 1 and (not source.get('revision') or not source.get(
                'scoring_code_path') or not t.get('original_success')):
            errors.append(
                f'{ident}: reviewed source requires revision, scoring code location and scoring description')
        if level >= 2 and (not t.get('assets') or not t.get(
                'robot_mapping') or not t.get('scene_evidence')):
            errors.append(
                f'{ident}: scene requires assets, robot mapping and load/reset evidence')
        if level >= 3 and not t.get('smoke_evidence'):
            errors.append(f'{ident}: execution requires smoke evidence')
        if level >= 4 and not t.get('baseline_evidence'):
            errors.append(f'{ident}: baseline requires evidence')
        u = t.get('unilab')
        if not isinstance(u, dict) or type(u.get('integrated')) is not bool:
            errors.append(
                f'{ident}: explicit UniLab integration flag required')
        elif u['integrated'] and (not u.get('revision') or not u.get('evidence')):
            errors.append(
                f'{ident}: integration requires revision and evidence')
    return errors


def readiness_gaps(t):
    missing = []
    for label, value in [('source_revision', t['source'].get('revision')), ('scoring_code_path', t['source'].get('scoring_code_path')), ('original_success', t.get('original_success')), ('assets', t.get(
            'assets')), ('robot_mapping', t.get('robot_mapping')), ('scene_evidence', t.get('scene_evidence')), ('smoke_evidence', t.get('smoke_evidence')), ('baseline_evidence', t.get('baseline_evidence'))]:
        if not value:
            missing.append(label)
    return missing
