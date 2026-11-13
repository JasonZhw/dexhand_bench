"""Explicit manifest downloads, bounded size, checksum verification and no clobbering."""
import hashlib
from pathlib import Path
import urllib.request
from urllib.parse import urlparse


def local_path(entry, root):
    root = Path(root).resolve()
    relative = Path(entry['local_path'])
    path = (root / relative).resolve()
    if relative.is_absolute() or root not in path.parents:
        raise ValueError('Asset path must remain inside project')
    return path


def verify_asset(entry, root):
    path = local_path(entry, root)
    if not path.is_file():
        return {'path': entry['local_path'], 'status': 'missing'}
    checksum = hashlib.sha256(path.read_bytes()).hexdigest()
    expected = entry.get('sha256')
    status = 'checksum_unknown' if expected is None else 'verified' if checksum == expected else 'checksum_mismatch'
    return {'path': entry['local_path'], 'status': status,
            'sha256': checksum, 'size_bytes': path.stat().st_size}


def fetch_asset(entry, root, *, max_bytes=100_000_000, opener=None):
    path = local_path(entry, root)
    if path.exists():
        if verify_asset(entry, root)['status'] == 'verified':
            return verify_asset(entry, root)
        raise FileExistsError(
            f'Unverified existing asset; not overwriting: {path}')
    url = urlparse(entry['url'])
    if url.scheme != 'https' or url.hostname != 'huggingface.co':
        raise ValueError(
            'This downloader supports official Hugging Face HTTPS manifests only')
    if not entry.get('sha256'):
        raise ValueError(
            'Known checksum required before reproducible download')
    if entry.get('size_bytes', 0) > max_bytes:
        raise ValueError('Asset exceeds download size cap')
    path.parent.mkdir(parents=True, exist_ok=True)
    created = False
    try:
        with (opener or urllib.request.urlopen)(entry['url'], timeout=30) as response, path.open('xb') as stream:
            created = True
            total = 0
            digest = hashlib.sha256()
            while True:
                block = response.read(1024 * 1024)
                if not block:
                    break
                total += len(block)
                if total > max_bytes:
                    raise ValueError('Asset exceeds download size cap')
                stream.write(block)
                digest.update(block)
        if digest.hexdigest() != entry['sha256']:
            raise ValueError('Downloaded checksum mismatch')
        if entry.get(
                'size_bytes') is not None and total != entry['size_bytes']:
            raise ValueError('Downloaded size mismatch')
    except Exception:
        if created:
            path.unlink(missing_ok=True)
        raise
    return verify_asset(entry, root)
