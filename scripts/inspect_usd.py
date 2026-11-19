from pxr import Usd, UsdGeom, UsdPhysics
import sys
import json
from pathlib import Path
import argparse
parser = argparse.ArgumentParser(
    description='Inspect USD composition and authored collision metadata; not a physics test')
parser.add_argument('asset_directory', type=Path)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
out = []
for p in sorted(args.asset_directory.rglob('*.usdz')):
    try:
        stage = Usd.Stage.Open(str(p.resolve()))
        if not stage:
            raise ValueError('Stage did not open')
        row = {
            'file': str(p),
            'stage_opened': True,
            'meters_per_unit': UsdGeom.GetStageMetersPerUnit(stage),
            'up_axis': str(
                UsdGeom.GetStageUpAxis(stage)),
            'default_prim': str(
                stage.GetDefaultPrim().GetPath()),
            'mesh_count': 0,
            'collision_prims': [],
            'rigid_bodies': [],
            'mass_properties': []}
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                row['mesh_count'] += 1
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                row['collision_prims'].append({'path': str(prim.GetPath()), 'approximation': str(prim.GetAttribute(
                    'physics:approximation').Get()), 'enabled': prim.GetAttribute('physics:collisionEnabled').Get()})
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                row['rigid_bodies'].append(str(prim.GetPath()))
            if prim.HasAPI(UsdPhysics.MassAPI):
                mass = UsdPhysics.MassAPI(prim)
                row['mass_properties'].append({'path': str(prim.GetPath(
                )), 'mass': mass.GetMassAttr().Get(), 'density': mass.GetDensityAttr().Get()})
        row['note'] = 'USD composition/load inspection only; not a collision simulation, not all authored properties necessarily active in runtime'
    except Exception as exc:
        row = {'file': str(p), 'stage_opened': False, 'error': str(exc)}
    out.append(row)
args.output.write_text(json.dumps(out, ensure_ascii=False, indent=2) + '\n')
print([(x['file'], x['stage_opened'], x.get('mesh_count'),
      len(x.get('collision_prims', []))) for x in out])
