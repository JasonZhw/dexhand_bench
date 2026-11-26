"""Compose one official object and the validated X5 robot template."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from integrations.unilab.build_allegro_robot import load_robot
from integrations.unilab.grasp_profiles import load_profile, ROBOTS, TASKS


def build(cfg, output):
    if cfg['backend'] != 'mujoco':
        raise ValueError('Only the MuJoCo asset route is validated')
    template = ROOT / cfg['robot_template']
    robot = load_robot(cfg)
    root = ET.parse(template).getroot()
    geometry_dir = ROOT/'assets/generated/grasp_objects'/cfg['task_id']
    geometry = json.loads((geometry_dir/'geometry.json').read_text())
    if (geometry['task_id'] != cfg['task_id'] or geometry['source_scale'] != cfg['object']['source_scale']
            or geometry.get('rotate_x_deg') != cfg['object']['rotate_x_deg']
            or geometry.get('collision_kind') != cfg['object']['collision']):
        raise ValueError('Object geometry configuration changed; prepare the geometry again')
    world, assets = root.find('worldbody'), root.find('asset')
    for body in list(world.findall('body')):
        if body.get('name') not in robot['root_bodies']:
            world.remove(body)
    for mesh in list(assets.findall('mesh')):
        if not mesh.get('name').startswith(('left_', 'right_')):
            assets.remove(mesh)
    table = world.find("geom[@name='diagnostic_table']")
    top = cfg['initial']['table_top_m']
    table.set('pos', f'0 -.12 {top-.03}')
    table.set('size', '.65 .5 .03')
    table.set('rgba', '.75 .78 .8 1')
    mass = cfg['object']['mass_kg']
    volumes = [x['volume_m3'] for x in geometry['collision_parts']]
    if not volumes or min(volumes) <= 0 or mass <= 0:
        raise ValueError('Positive mass and convex volumes required')
    x, y = cfg['initial']['xy_m']
    body = ET.SubElement(world, 'body', name='object', pos=f'{x} {y} {top+cfg["initial"]["clearance_m"]}')
    ET.SubElement(body, 'freejoint', name='object_free')
    visual = geometry_dir/'visual.stl'
    if hashlib.sha256(visual.read_bytes()).hexdigest() != geometry['visual_sha256']:
        raise ValueError('Visual mesh checksum mismatch')
    ET.SubElement(assets, 'mesh', name='object_visual', file=str(visual.resolve()))
    ET.SubElement(body, 'geom', name='object_visual', type='mesh', mesh='object_visual',
                  contype='0', conaffinity='0', mass='0', group='1',
                  rgba=' '.join(map(str,cfg['object']['rgba'])))
    for index, part in enumerate(geometry['collision_parts']):
        path = geometry_dir/part['file']
        if hashlib.sha256(path.read_bytes()).hexdigest() != part['sha256']:
            raise ValueError('Collision mesh checksum mismatch')
        name = f'object_collision_{index}'
        ET.SubElement(assets, 'mesh', name=name, file=str(path.resolve()))
        ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name,
                      group='3', rgba='.8 .35 .1 .2',
                      mass=str(mass*part['volume_m3']/sum(volumes)),
                      friction=' '.join(map(str,cfg['object']['friction'])))
    root.find('option').set('timestep', str(cfg['runtime']['sim_dt']))
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    path = output/'scene.xml'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    if model.nu != len(robot['joint_names']) or model.joint('object_free').qposadr[0] != model.nu:
        raise ValueError('Expected configured robot controls followed by one free object')
    (output/'scene.json').write_text(json.dumps({
        'task_id': cfg['task_id'], 'profile': cfg, 'robot': robot,
        'template_sha256': hashlib.sha256(template.read_bytes()).hexdigest(),
        'scene_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'mesh_dependencies': {mesh.get('file'): hashlib.sha256(Path(mesh.get('file')).read_bytes()).hexdigest()
                              for mesh in assets.findall('mesh')},
        'object_com_offset_m': model.body('object').ipos.tolist(),
        'object_inertia_kg_m2': model.body('object').inertia.tolist(),
        'geometry': geometry, 'task_success': None}, indent=2)+'\n')
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=['G01','G02','G03','G04','G05','all'], default='all')
    parser.add_argument('--robot', choices=ROBOTS, default='allegro')
    args = parser.parse_args()
    for task_id in TASKS:
        cfg = load_profile(task_id, args.robot)
        if args.task in ('all', cfg['task_id']):
            print(build(cfg, (ROOT/cfg['scene']).parent))
