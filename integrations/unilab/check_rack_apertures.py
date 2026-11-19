"""Probe converted rack mouths with a small sphere; not full tube clearance."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def check(scene):
    xml = ET.parse(scene)
    for mesh in xml.findall('./asset/mesh'):
        mesh.set('file', str((scene.parent / mesh.get('file')).resolve()))
    probe = ET.SubElement(xml.find('worldbody'), 'body', name='aperture_probe')
    ET.SubElement(probe, 'freejoint', name='probe_free')
    ET.SubElement(probe, 'geom', name='probe_geom', type='sphere', size='.003', mass='.001')
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml.getroot(), encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rack = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'slot')
    probe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'probe_geom')
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'probe_free')
    address = model.jnt_qposadr[joint]
    origin = data.xpos[rack].copy()
    rotation = data.xmat[rack].reshape(3, 3).copy()
    metadata = json.loads((ROOT / 'assets/robodojo/Assets/Object/RoboDojo/Geometry/test_tube_slot/00000/metadata.json').read_text())
    points = {name: value['center'][0][:3] for name, value in metadata['passive']['support'].items()}
    points['solid_between_holes'] = [.018, -.018, .028]
    results = {}
    for name, point in points.items():
        data.qpos[address:address + 3] = origin + rotation @ np.array(point)
        data.qpos[address + 3:address + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(model, data)
        distances = []
        for contact in data.contact[:data.ncon]:
            ids = (contact.geom1, contact.geom2)
            if probe_id in ids and any(model.geom_bodyid[g] == rack for g in ids):
                distances.append(float(contact.dist))
        results[name] = {'point_rack_frame': point, 'contact_distances': distances,
                         'penetrating': any(d < -1e-5 for d in distances)}
    passed = (all(not v['penetrating'] for k, v in results.items() if k != 'solid_between_holes')
              and results['solid_between_holes']['penetrating'])
    receipt = {'passed': passed, 'probe_radius_m': .003, 'results': results,
               'scope': 'rack mouth point probes only; not full tube insertion or SDF equivalence'}
    (scene.parent / 'apertures.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))
    if not passed:
        raise SystemExit('Converted rack failed aperture acceptance')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    check(parser.parse_args().scene)
