"""Compose source X5 URDFs with the converted object scene for control checks.

URDF mesh collision uses MuJoCo convex hulls, not the original USD physics.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from dexbench.assets import verify_asset


def build(output):
    output = Path(output).resolve()
    manifest = json.loads((ROOT / 'asset_manifests/I01-x5.json').read_text())
    for entry in manifest['files']:
        if verify_asset(entry, ROOT)['status'] != 'verified':
            raise ValueError(f"Missing or altered source asset: {entry['local_path']}")
    source_scene = ROOT / 'assets/generated/I01-convex/scene.xml'
    root = ET.parse(source_scene).getroot()
    output.mkdir(parents=True, exist_ok=False)
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    compiler.set('angle', 'radian')
    for mesh in root.findall('./asset/mesh'):
        mesh.set('file', str((source_scene.parent / mesh.get('file')).resolve()))
    robot_dir = ROOT / 'assets/robodojo/Assets/Robots/x5'
    urdf = ET.parse(robot_dir / 'X5A.urdf').getroot()
    # Cameras have visual-only GLB references. Collision geometry is retained.
    for link in urdf.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    for mesh in urdf.findall('.//mesh'):
        mesh.set('filename', str(robot_dir / mesh.get('filename')))
    extension = ET.SubElement(urdf, 'mujoco')
    ET.SubElement(extension, 'compiler', fusestatic='false', discardvisual='true', strippath='false')
    robot = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding='unicode'))
    intermediate = output / 'source_robot.xml'
    mujoco.mj_saveLastXML(str(intermediate), robot)
    source = ET.parse(intermediate).getroot()
    # The URDF compiler emits radians; preserve joint frames and inertial data.
    world = root.find('worldbody')
    asset = root.find('asset')
    actuators = ET.SubElement(root, 'actuator')
    contact = ET.SubElement(root, 'contact')
    control_names = []
    for side, x in [('left', -.3), ('right', .3)]:
        # URDF joint neighbours overlap by construction. MuJoCo's static-base
        # parent filter does not suppress base_link/link1; exclude only direct
        # parent-child pairs, retaining other self and object collisions.
        for joint in urdf.findall('joint'):
            ET.SubElement(contact, 'exclude',
                          body1=side + '_' + joint.find('parent').get('link'),
                          body2=side + '_' + joint.find('child').get('link'))
        part = copy.deepcopy(source)
        for node in part.iter():
            for attr in ('name', 'mesh'):
                if attr in node.attrib:
                    node.set(attr, side + '_' + node.get(attr))
        for mesh in part.findall('./asset/mesh'):
            # URDF importer can retain or strip the mesh path in saved MJCF.
            mesh.set('file', str(robot_dir / 'meshes' / Path(mesh.get('file')).name))
            asset.append(mesh)
        base = part.find('./worldbody/body')
        if base is None or base.get('name') != side + '_base_link':
            raise ValueError('Unexpected source robot root')
        base.set('pos', f'{x} -0.45 0.765')
        base.set('quat', '.7071067811865476 0 0 .7071067811865476')
        for body in base.iter('body'):
            body.set('gravcomp', '1')  # Source Isaac configuration disables robot gravity.
        for joint in base.iter('joint'):
            index = int(joint.get('name').split('joint')[-1])
            joint.set('armature', '.01' if index <= 6 else '0')
            joint.set('damping', '40' if index <= 6 else '100')
            if index >= 7:
                # ref sets qpos0 while retaining the URDF physical transform.
                # Change the body position to represent the opened finger at ref.
                joint.set('ref', '.044')
                parent = next(b for b in base.iter('body') if joint in list(b))
                pos = [float(v) for v in parent.get('pos', '0 0 0').split()]
                axis = [float(v) for v in joint.get('axis').split()]
                parent.set('pos', ' '.join(str(p + .044 * a) for p, a in zip(pos, axis)))
            name = joint.get('name')
            control_names.append(name)
            ET.SubElement(actuators, 'position', name=name + '_drive', joint=name,
                          kp='4400' if index <= 6 else '2300',
                          forcelimited='true', forcerange='-100 100',
                          ctrllimited='true', ctrlrange=joint.get('range'))
        # Pinned UniLab's noncomposed limit indexing requires controlled scalar
        # joints before free objects. Keep both arms in actuator order.
        world.insert(0 if side == 'left' else 1, base)
    root.find('option').set('integrator', 'implicitfast')
    path = output / 'scene.xml'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(path))
    if model.nu != 16:
        raise ValueError('Expected sixteen independent source joint drives')
    receipt = dict(scope='dual X5 control diagnostic, not insertion',
                   dataset_revision=manifest['revision'], joint_order=control_names,
                   scene_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                   nq=model.nq, nv=model.nv, nu=model.nu,
                   limitations=['URDF mesh convex hulls are not original USD collision shapes',
                                'PD stiffness/damping and force caps adapted from source; no controller equivalence',
                                'Robot gravity compensated; implicitfast integrator chosen for stiff drives',
                                'Controlled joints precede free objects for pinned UniLab limit indexing',
                                'Direct URDF parent-child collisions explicitly excluded; other collisions retained',
                                'Cameras/visual-only meshes omitted; no task policy or insertion success'])
    (output / 'conversion.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(build(parser.parse_args().output))
