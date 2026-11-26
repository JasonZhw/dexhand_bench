"""Extract pinned Bench2Dex geometry; textures and source solver equivalence excluded."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from pxr import Usd, UsdGeom, UsdPhysics
import coacd
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from dexbench.assets import verify_asset


def prepare(cfg, output):
    manifest = json.loads((ROOT / cfg['object']['source_manifest']).read_text())
    entries = [x for x in manifest['files'] if x['task_id'] == cfg['task_id']]
    for entry in entries:
        if verify_asset(entry, ROOT)['status'] != 'verified':
            raise ValueError(f"Missing or changed source: {entry['local_path']}")
    stage = Usd.Stage.Open(str((ROOT / entries[0]['local_path']).resolve()))
    if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != 'Z':
        raise ValueError('Expected metre-scale Z-up source stage')
    cache, meshes = UsdGeom.XformCache(), []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        schema = UsdGeom.Mesh(prim)
        transform = cache.GetLocalToWorldTransform(prim)
        vertices = np.array([tuple(transform.Transform(p)) for p in schema.GetPointsAttr().Get()])
        indices, faces, offset = list(schema.GetFaceVertexIndicesAttr().Get()), [], 0
        for count in schema.GetFaceVertexCountsAttr().Get():
            poly = indices[offset:offset + count]
            faces.extend([poly[0], poly[i], poly[i + 1]] for i in range(1, count - 1))
            offset += count
        if offset != len(indices):
            raise ValueError('Invalid source topology')
        meshes.append(trimesh.Trimesh(vertices, faces, process=True))
    if not meshes:
        raise ValueError('No enabled collision meshes, including instance proxies')
    mesh = trimesh.util.concatenate(meshes)
    mesh.vertices *= np.asarray(cfg['object']['source_scale'])
    angle = np.deg2rad(cfg['object']['rotate_x_deg'])
    rotation = np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])
    mesh.vertices = mesh.vertices @ rotation.T
    low, high = mesh.bounds
    offset = np.array([(low[0]+high[0])/2, (low[1]+high[1])/2, low[2]])
    mesh.vertices -= offset
    if np.max(mesh.extents) > .7 or np.min(mesh.extents) < .005:
        raise ValueError(f'Unexpected scaled object extent: {mesh.extents}')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    mesh.export(output / 'visual.stl')
    if cfg['object']['collision'] == 'single_convex_hull':
        parts = [mesh.convex_hull]
    else:
        coacd.set_log_level('warn')
        pieces = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=.05,
                                resolution=1000, mcts_iterations=30, seed=0)
        parts = [trimesh.Trimesh(v, f, process=True) for v, f in pieces]
    files = []
    for index, part in enumerate(parts):
        path = output / f'collision_{index}.stl'
        part.export(path)
        files.append({'file': path.name, 'volume_m3': abs(float(part.volume)),
                      'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    receipt = {'task_id': cfg['task_id'], 'source_revision': manifest['revision'],
               'source_files': entries, 'source_scale': cfg['object']['source_scale'],
               'rotate_x_deg': cfg['object']['rotate_x_deg'], 'collision_kind': cfg['object']['collision'],
               'rotation_matrix': rotation.tolist(), 'subtracted_offset_m': offset.tolist(),
               'bounds_m': mesh.bounds.tolist(), 'extents_m': mesh.extents.tolist(),
               'collision_parts': files, 'visual_sha256': hashlib.sha256((output/'visual.stl').read_bytes()).hexdigest(),
               'limitations': ['Generated convex collision; no equivalence to PhysX decomposition',
                               'Uniform-volume approximate inertia', 'Plain colour preview; textures not downloaded'],
               'coacd': {'threshold': .05, 'resolution': 1000, 'mcts_iterations': 30, 'seed': 0}}
    (output/'geometry.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(cfg['task_id'], 'extents_m', mesh.extents, 'collision_parts', len(parts), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=['G01','G02','G03','G04','G05','all'], default='all')
    args = parser.parse_args()
    for p in sorted((ROOT/'profiles/local_grasp').glob('*.json')):
        cfg = json.loads(p.read_text())
        if args.task in ('all', cfg['task_id']):
            prepare(cfg, ROOT/'assets/generated/grasp_objects'/cfg['task_id'])
