import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


class RunCliTests(unittest.TestCase):
    def invoke(self,
               *args): return subprocess.run([sys.executable,
                                              '-m',
                                              'dexbench',
                                              *args],
                                             cwd=ROOT,
                                             capture_output=True,
                                             text=True)

    def test_profiles_list(self):
        r = self.invoke('profiles')
        self.assertEqual(r.returncode, 0)
        self.assertIn('H01-source.json', r.stdout)

    def test_run_requires_explicit_output(self):
        r = self.invoke('run', '--profile', 'profiles/H01-source.json')
        self.assertEqual(r.returncode, 2)

    def test_throughput_rejects_recording_before_loading_simulator(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = self.invoke('run',
                            '--profile',
                            'profiles/H01-source.json',
                            '--output',
                            str(Path(tmp) / 'run'),
                            '--mode',
                            'throughput',
                            '--video')
            self.assertEqual(r.returncode, 2)
            self.assertIn('Throughput excludes', r.stderr)

    def test_unknown_profile_not_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'profile.json'
            p.write_text('{"backend":"unilab"}')
            r = self.invoke('run', '--profile', str(p),
                            '--output', str(Path(tmp) / 'run'))
            self.assertEqual(r.returncode, 2)
            self.assertFalse((Path(tmp) / 'run').exists())
