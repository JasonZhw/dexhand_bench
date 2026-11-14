"""Build an explicitly approximate MJCF object scene from verified RoboDojo USD.

This asset experiment is not a converted robot or an executable insertion task.
CoACD convex pieces replace source SDF collisions; physical equivalence is unproven.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

# Load USD first: CoACD 1.0.7 loaded first crashes USD 25.11's Tf import
# on the validated Linux CPython 3.10 environment.
from pxr import Usd, UsdGeom, UsdPhysics
import coacd
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]


def collision_mesh(path):
    stage = Usd.Stage.Open(str(path.resolve()))
    if UsdGeom.GetStageUpAxis(stage) != 'Z' or UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError('Expected source Z-up, meter-scale USD')
    cache = UsdGeom.XformCache()
    meshes = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        transform = cache.GetLocalToWorldTransform(prim)
        vertices = np.array([tuple(transform.Transform(p)) for p in points])
        indices = list(mesh.GetFaceVertexIndicesAttr().Get())
        faces, offset = [], 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            polygon = indices[offset:offset + count]
            for i in range(1, count - 1):
                faces.append([polygon[0], polygon[i], polygon[i + 1]])
            offset += count
        if offset != len(indices):
            raise ValueError('Invalid source mesh topology')
        meshes.append(trimesh.Trimesh(vertices, faces, process=True))
    if not meshes:
        raise ValueError(f'No enabled collision mesh in {path}')
    return trimesh.util.concatenate(meshes)


def build(output):
    manifest = json.loads((ROOT / 'asset_manifests/I01-robodojo.json').read_text())
    layout_manifest = json.loads((ROOT / 'asset_manifests/I01-fixed-layout-dependencies.json').read_text())
    layout_path = ROOT / layout_manifest['layout']
    layout_hash = hashlib.sha256(layout_path.read_bytes()).hexdigest()
    if layout_hash != layout_manifest['layout_sha256']:
        raise ValueError('Fixed layout checksum mismatch')
    for item in manifest['files']:
        path = ROOT / item['local_path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise ValueError(f'Asset hash mismatch: {path}')
    output.mkdir(parents=True, exist_ok=False)
    root = ET.Element('mujoco', model='I01_object_scene_only')
    ET.SubElement(root, 'compiler', angle='radian')
    ET.SubElement(root, 'option', timestep='0.004', gravity='0 0 -9.81')
    assets = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 0 2')
    layout = json.loads(layout_path.read_text())
    table = layout['Table']
    ET.SubElement(world, 'geom', name='diagnostic_table', type='box',
                  pos=' '.join(map(str, table['default_pos'])),
                  quat=' '.join(map(str, table['default_ori'])),
                  size=' '.join(str(v / 2) for v in table['scale']),
                  friction='.8 .005 .0001', rgba='.5 .5 .5 1')
    receipt = {'scope': 'object scene only; no robot or task success', 'assets': {},
               'limitations': ['SDF replaced with approximate convex decomposition',
                               'Box table follows source layout dimensions; contact material equivalence unverified',
                               'Inertia inferred from converted collision volume',
                               'No robot, controller or task success predicates'],
               'physics_dt': .004, 'dt_source': 'RoboDojo env_cfg/sim/sim_config.yml',
               'source_code_revision': 'f57e1633ffed59761bc8e568cfe7530c4b33381c',
               'dataset_revision': manifest['revision'], 'layout_sha256': layout_hash,
               'decomposition': {'threshold': .02, 'resolution': 2000, 'mcts_iterations': 100, 'seed': 0}}
    for kind, category in [('Geometry', 'test_tube_slot'), ('Rigid', 'test_tube')]:
        items = layout[kind][category]
        source = ROOT / 'assets/robodojo/Assets/Object/RoboDojo' / kind / category / f"{items[0]['category_idx']:05d}" / 'object.usdz'
        mesh = collision_mesh(source)
        parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=.02,
                               resolution=2000, mcts_iterations=100, seed=0)
        names, volumes = [], []
        for index, (vertices, faces) in enumerate(parts):
            name = f'{category}_{index}'
            part = trimesh.Trimesh(vertices, faces, process=True)
            part.export(output / f'{name}.stl')
            ET.SubElement(assets, 'mesh', name=name, file=f'{name}.stl')
            names.append(name)
            volumes.append(abs(part.volume))
        if not names or sum(volumes) <= 0:
            raise ValueError('Convex decomposition produced no valid volume')
        receipt['assets'][category] = {'parts': len(names), 'bounds': mesh.bounds.tolist(),
                                      'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
        for item in items:
            body = ET.SubElement(world, 'body', name=item['label'],
                                 pos=' '.join(map(str, item['default_pos'])),
                                 quat=' '.join(map(str, item['default_ori'])))
            if kind == 'Rigid':
                ET.SubElement(body, 'freejoint', name=item['label'] + '_free')
            for name, volume in zip(names, volumes):
                options = dict(type='mesh', mesh=name, friction=f"{item['physics'].get('friction', .45)} .005 .0001",
                               rgba='.2 .5 .9 1' if kind == 'Rigid' else '.8 .6 .2 1')
                if kind == 'Rigid':
                    options['mass'] = str(item['physics']['mass'] * volume / sum(volumes))
                ET.SubElement(body, 'geom', **options)
    ET.indent(root)
    ET.ElementTree(root).write(output / 'scene.xml', encoding='unicode')
    (output / 'conversion.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    build(parser.parse_args().output)
