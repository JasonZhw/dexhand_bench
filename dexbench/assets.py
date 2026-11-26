"""Explicit manifest downloads, bounded size, checksum verification and no clobbering."""
import hashlib
import re
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
    pinned_menagerie = (url.hostname == 'raw.githubusercontent.com'
        and re.fullmatch(r'/google-deepmind/mujoco_menagerie/[0-9a-f]{40}/wonik_allegro/[^?#]+', url.path))
    pinned_vendor = (url.hostname == 'raw.githubusercontent.com'
        and re.fullmatch(r'/(wuji-technology/wuji-description|sharpa-robotics/sharpa-urdf-usd-xml)/[0-9a-f]{40}/[^?#]+', url.path))
    pinned_farama_mjcf = (url.hostname == 'raw.githubusercontent.com'
        and re.fullmatch(r'/Farama-Foundation/Gymnasium-Robotics/[0-9a-f]{40}/gymnasium_robotics/envs/assets/(fetch/slide\.xml|hand/manipulate_block\.xml|LICENSE\.md)', url.path))
    if url.scheme != 'https' or not (url.hostname == 'huggingface.co' or pinned_menagerie or pinned_vendor or pinned_farama_mjcf):
        raise ValueError(
            'Only Hugging Face or pinned approved asset paths over HTTPS are supported')
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
