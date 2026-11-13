import hashlib
import io
import tempfile
import unittest
from pathlib import Path
from dexbench.assets import verify_asset, fetch_asset


class AssetTests(unittest.TestCase):
    def setUp(self):
        self.t = tempfile.TemporaryDirectory()
        self.addCleanup(self.t.cleanup)
        self.root = Path(self.t.name)
        self.entry = {
            'local_path': 'assets/a.bin',
            'url': 'https://huggingface.co/datasets/a/b/resolve/abc/a.bin',
            'sha256': hashlib.sha256(b'asset').hexdigest(),
            'size_bytes': 5}

    def test_missing_file_not_downloaded(self): self.assertEqual(
        verify_asset(self.entry, self.root)['status'], 'missing')

    def test_hash_checked_after_download(self):
        fetch_asset(
            self.entry,
            self.root,
            opener=lambda *a,
            **kw: io.BytesIO(b'asset'))
        self.assertEqual(
            verify_asset(
                self.entry,
                self.root)['status'],
            'verified')

    def test_bad_download_removed(self):
        with self.assertRaises(ValueError):
            fetch_asset(
                self.entry,
                self.root,
                opener=lambda *a,
                **kw: io.BytesIO(b'wrong'))
        self.assertFalse((self.root / 'assets/a.bin').exists())

    def test_path_cannot_escape_root(self):
        self.entry['local_path'] = '../outside'
        with self.assertRaises(ValueError):
            verify_asset(self.entry, self.root)

    def test_unknown_checksum_not_called_verified(self):
        self.entry['sha256'] = None
        (self.root / 'assets').mkdir()
        (self.root / 'assets/a.bin').write_bytes(b'asset')
        self.assertEqual(
            verify_asset(
                self.entry,
                self.root)['status'],
            'checksum_unknown')

    def test_existing_corrupt_file_not_overwritten(self):
        p = self.root / 'assets/a.bin'
        p.parent.mkdir()
        p.write_bytes(b'wrong')
        with self.assertRaises(FileExistsError):
            fetch_asset(
                self.entry,
                self.root,
                opener=lambda *a,
                **kw: io.BytesIO(b'asset'))
        self.assertEqual(p.read_bytes(), b'wrong')
