import json
import subprocess
import sys
import tempfile
import shutil
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


class CliTests(unittest.TestCase):
    def test_catalog_and_experiment_references(self):
        result = subprocess.run([sys.executable,
                                 '-m',
                                 'dexbench',
                                 'list',
                                 '--json'],
                                cwd=ROOT,
                                text=True,
                                capture_output=True,
                                check=True)
        tasks = json.loads(result.stdout)
        self.assertEqual(len(tasks), 21)
        self.assertTrue(all(t['status'] in ('selected',
                                            'source_reviewed',
                                            'scene_ready',
                                            'smoke_executed') for t in tasks))
        self.assertTrue(all(t['baseline_evidence'] is None for t in tasks))
        for t in tasks:
            for eid in t['related_experiment_ids']:
                e = json.loads(
                    (ROOT / 'experiments' / (eid + '.json')).read_text())
                self.assertFalse(e['proves_selected_task_complete'])

    def test_relocated_directory_without_local_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / 'future repo'
            shutil.copytree(
                ROOT,
                dest,
                ignore=shutil.ignore_patterns(
                    '.local',
                    '__pycache__',
                    '.venv',
                    '.venv-*',
                    '.git',
                    '.clone-incomplete',
                    'third_party',
                    'runs',
                    'assets',
                    'data'))
            result = subprocess.run([sys.executable,
                                     '-m',
                                     'dexbench',
                                     'audit'],
                                    cwd=dest,
                                    text=True,
                                    capture_output=True,
                                    check=True)
            report = json.loads(result.stdout)
            self.assertEqual(report['task_count'], 21)
            self.assertEqual(sum(report['statuses'].values()), 21)
            self.assertNotIn('baseline_verified', report['statuses'])

    def test_empty_catalog_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = subprocess.run([sys.executable,
                                '-m',
                                'dexbench',
                                'audit',
                                '--root',
                                tmp],
                               cwd=ROOT,
                               text=True,
                               capture_output=True)
            self.assertEqual(r.returncode, 2)
