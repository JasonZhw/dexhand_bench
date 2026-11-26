"""Replace X5 gripper fingers with pinned left/right Allegro V3 MJCF hands."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from dexbench.assets import verify_asset

CONFIG = 'profiles/robots/dual_x5_allegro.json'
FINGERS = {'index': 'ff', 'middle': 'mf', 'ring': 'rf', 'thumb': 'th'}


def build(output):
    cfg = json.loads((ROOT/CONFIG).read_text())
    for key in ('source_arm_manifest', 'source_hand_manifest'):
        manifest = json.loads((ROOT/cfg[key]).read_text())
        for entry in manifest['files']:
            if verify_asset(entry, ROOT)['status'] != 'verified':
                raise ValueError(f"Missing or altered source asset: {entry['local_path']}")
    template = ROOT/cfg['source_arm_template']
    expected = json.loads((template.parent/'conversion.json').read_text())['scene_sha256']
    if hashlib.sha256(template.read_bytes()).hexdigest() != expected:
        raise ValueError('X5 source template differs from its conversion receipt')
    root = ET.parse(template).getroot()
    root.set('model', cfg['id'])
    root.find('compiler').set('autolimits', 'true')
    root.find('option').set('cone', 'elliptic')
    root.find('option').set('impratio', '10')
    world, assets = root.find('worldbody'), root.find('asset')
    actuator, contact = root.find('actuator'), root.find('contact')
    for body in list(world.findall('body')):
        if body.get('name') not in ('left_base_link', 'right_base_link'):
            world.remove(body)
    for side in ('left', 'right'):
        wrist = world.find(f".//body[@name='{side}_link6']")
        for finger in list(wrist.findall('body')):
            wrist.remove(finger)
        for act in list(actuator):
            if act.get('joint') in (f'{side}_joint7', f'{side}_joint8'):
                actuator.remove(act)
    used_meshes = {g.get('mesh') for g in world.iter('geom') if g.get('mesh')}
    for mesh in list(assets.findall('mesh')):
        if mesh.get('name') not in used_meshes:
            assets.remove(mesh)
    bodies = {b.get('name') for b in world.iter('body')}
    for exclusion in list(contact):
        if not {exclusion.get('body1'), exclusion.get('body2')} <= bodies:
            contact.remove(exclusion)
    defaults = ET.SubElement(root, 'default')
    hands = {}
    source_dir = ROOT/'assets/menagerie/wonik_allegro'
    for side in ('left', 'right'):
        hand = ET.parse(source_dir/f'{side}_hand.xml').getroot()
        prefix = f'{side}_allegro_'
        for mesh in hand.findall('./asset/mesh'):
            mesh.set('name', mesh.get('name', Path(mesh.get('file')).stem))
            mesh.set('file', str((source_dir/'assets'/mesh.get('file')).resolve()))
        for node in hand.iter():
            for attr in ('name', 'class', 'childclass', 'mesh', 'material', 'joint', 'body1', 'body2'):
                if attr in node.attrib:
                    node.set(attr, prefix+node.get(attr))
        palm = hand.find('./worldbody/body')
        palm.set('pos', ' '.join(map(str, cfg['hand_mount']['position_m'])))
        palm.set('quat', ' '.join(map(str, cfg['hand_mount']['quaternion_wxyz'])))
        # qpos0 must respect the positive thumb abduction limit. Rotating its
        # body by the same ref preserves source forward kinematics at every q.
        thumb = palm.find(f".//body[@name='{prefix}th_base']")
        joint = thumb.find('joint')
        axis = np.array([-1., 0., 0.]) if side == 'right' else np.array([1., 0., 0.])
        delta, rotated = np.empty(4), np.empty(4)
        mujoco.mju_axisAngle2Quat(delta, axis, cfg['thumb_rest_rad'])
        original = np.array([float(v) for v in thumb.get('quat').split()])
        original /= np.linalg.norm(original)
        mujoco.mju_mulQuat(rotated, original, delta)
        thumb.set('quat', ' '.join(map(str, rotated)))
        joint.set('ref', str(cfg['thumb_rest_rad']))
        for body in palm.iter('body'):
            body.set('gravcomp', '1' if cfg['gravity_compensation'] else '0')
        world.find(f".//body[@name='{side}_link6']").append(palm)
        defaults.extend(hand.find('default'))
        assets.extend(hand.find('asset'))
        contact.extend(hand.find('contact'))
        actuator.extend(hand.find('actuator'))
        hands[side] = {'palm': prefix+'palm',
            'fingers': {label: [prefix+finger+'_'+segment for segment in
                               ('base', 'proximal', 'medial', 'distal', 'tip')]
                        for label, finger in FINGERS.items()},
            'tips': {label: prefix+finger+'_tip' for label, finger in FINGERS.items()}}
    # Left-hand source body order and source actuator order differ. Align all
    # scalar joints and actuators explicitly for the pinned UniLab read plan.
    joint_names = [j.get('name') for j in world.iter('joint')]
    by_joint = {a.get('joint'): a for a in actuator}
    if set(by_joint) != set(joint_names):
        raise ValueError('Each scalar robot joint must have exactly one drive')
    actuator[:] = [by_joint[name] for name in joint_names]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    path = output/'scene.xml'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    if model.nu != 44 or model.nq != 44:
        raise ValueError('Expected 12 arm plus 32 Allegro joints')
    np.testing.assert_array_equal(model.actuator_trnid[:, 0], np.arange(model.nu))
    metadata = {'id':cfg['id'], 'configuration':cfg,
        'template_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'root_body_name':'left_base_link', 'root_bodies':['left_base_link','right_base_link'],
        'joint_names':joint_names, 'actuator_names':[a.get('name') for a in actuator],
        'body_names':[b.get('name') for b in world.iter('body')],
        'arm_joint_names':[f'{s}_joint{i}' for s in ('left','right') for i in range(1,7)],
        'hand_joint_names':{s:[n for n in joint_names if n.startswith(s+'_allegro_')] for s in ('left','right')},
        'hands':hands, 'designated_hand':cfg['designated_hand'],
        'hand_source_revision':json.loads((ROOT/cfg['source_hand_manifest']).read_text())['revision'],
        'default_joint_positions':model.qpos0.tolist(),
        'limitations':cfg['limitations']}
    (output/'robot.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return path


def load_robot(profile):
    cfg = json.loads((ROOT/profile['robot_config']).read_text())
    if profile['robot_template'] != cfg['template']:
        raise ValueError('Robot template differs from selected robot configuration')
    template = ROOT/cfg['template']
    metadata = json.loads((template.parent/'robot.json').read_text())
    if metadata['configuration'] != cfg or hashlib.sha256(template.read_bytes()).hexdigest() != metadata['template_sha256']:
        raise ValueError('Robot configuration or template changed; rebuild the robot')
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'assets/generated/dual_x5_allegro')
    print(build(parser.parse_args().output))
