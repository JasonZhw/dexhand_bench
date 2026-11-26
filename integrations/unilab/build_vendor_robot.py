"""Compose X5 arms and pinned native vendor hands without changing task rules."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from dexbench.assets import verify_asset


def build(kind, output):
    if kind not in ('wuji','sharpa'):
        raise ValueError('Supported vendor hands: wuji, sharpa')
    cfg=json.loads((ROOT/f'profiles/robots/dual_x5_{kind}.json').read_text())
    for key in ('source_arm_manifest','source_hand_manifest'):
        for entry in json.loads((ROOT/cfg[key]).read_text())['files']:
            if verify_asset(entry,ROOT)['status'] != 'verified':
                raise ValueError(f"Missing or changed source asset: {entry['local_path']}")
    template=ROOT/cfg['source_arm_template']
    if hashlib.sha256(template.read_bytes()).hexdigest() != json.loads((template.parent/'conversion.json').read_text())['scene_sha256']:
        raise ValueError('X5 source differs from its conversion receipt')
    root=ET.parse(template).getroot();root.set('model',cfg['id'])
    root.find('compiler').set('autolimits','true')
    root.find('option').attrib.update(cfg['simulation_options'])
    world,assets,actuator,contact=(root.find(x) for x in ('worldbody','asset','actuator','contact'))
    for body in list(world.findall('body')):
        if body.get('name') not in ('left_base_link','right_base_link'):world.remove(body)
    for side in ('left','right'):
        wrist=world.find(f".//body[@name='{side}_link6']")
        for child in list(wrist.findall('body')):wrist.remove(child)
        for act in list(actuator):
            if act.get('joint') in (f'{side}_joint7',f'{side}_joint8'):actuator.remove(act)
    used_meshes={g.get('mesh') for g in world.iter('geom') if g.get('mesh')}
    for mesh in list(assets.findall('mesh')):
        if mesh.get('name') not in used_meshes:assets.remove(mesh)
    bodies={b.get('name') for b in world.iter('body')}
    for exclusion in list(contact):
        if not {exclusion.get('body1'),exclusion.get('body2')}<=bodies:contact.remove(exclusion)
    defaults=ET.SubElement(root,'default')
    hands={};offsets={'left_joint1':-.08,'right_joint1':.08};source_options={}
    for side in ('left','right'):
        source_path=ROOT/cfg['source_models'][side]
        source=mujoco.MjModel.from_xml_path(str(source_path))
        hand=ET.parse(source_path).getroot();prefix=f'{side}_{kind}_'
        compiler=hand.find('compiler')
        if compiler.get('angle') != 'radian':raise ValueError('Vendor hand must use radians')
        source_options[side]=hand.find('option').attrib if hand.find('option') is not None else {}
        for mesh in hand.findall('./asset/mesh'):
            mesh.set('name',mesh.get('name',Path(mesh.get('file')).stem))
            mesh.set('file',str((source_path.parent/compiler.get('meshdir','')/mesh.get('file')).resolve()))
        # Freeze source dampratio-derived damping before attaching a moving arm;
        # otherwise MuJoCo would recompute it from the new assembly's inertia.
        for act in hand.find('actuator'):
            if 'dampratio' in act.attrib:
                value=-source.actuator(act.get('name')).biasprm[2]
                del act.attrib['dampratio'];act.set('kv',str(value))
        for node in hand.iter():
            for attr in ('name','class','childclass','mesh','material','joint','body1','body2','site','objname','refname'):
                if attr in node.attrib:node.set(attr,prefix+node.get(attr))
        body=hand.find('./worldbody/body')
        body.set('pos',' '.join(map(str,cfg['hand_mount']['position_m'])))
        body.set('quat',' '.join(map(str,cfg['hand_mount']['quaternion_wxyz'])))
        scoped=ET.SubElement(defaults,'default',{'class':prefix+'defaults'})
        if hand.find('default') is not None:scoped.extend(hand.find('default'))
        body.set('childclass',prefix+'defaults')
        # Both sources have duplicate visual geoms; reserve group 3 for collision display.
        for geom in body.iter('geom'):
            if geom.get('group')=='2':geom.set('group','3')
        world.find(f".//body[@name='{side}_link6']").append(body)
        for section in ('asset','actuator','contact','sensor'):
            src=hand.find(section)
            if src is not None:
                dst=root.find(section)
                if dst is None:dst=ET.SubElement(root,section)
                dst.extend(src)
        mapping=cfg['hands'][side]
        hands[side]={'palm':prefix+mapping['palm'],'fingers':{}}
        for label,name in mapping['finger_roots'].items():
            finger=body.find(f".//body[@name='{prefix+name}']")
            if finger is None:raise ValueError(f'Missing finger root: {name}')
            hands[side]['fingers'][label]=[b.get('name') for b in finger.iter('body')]
        offsets.update({prefix+n:v for n,v in mapping['probe_offsets'].items()})
    for body in world.iter('body'):body.set('gravcomp','1' if cfg['gravity_compensation'] else '0')
    joint_names=[j.get('name') for j in world.iter('joint')]
    by_joint={a.get('joint'):a for a in actuator}
    if len(by_joint)!=len(actuator) or set(by_joint)!=set(joint_names):
        raise ValueError('Each scalar joint must have one independent position drive')
    actuator[:]=[by_joint[n] for n in joint_names]
    output=Path(output);output.mkdir(parents=True,exist_ok=False);path=output/'scene.xml'
    ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(path.resolve()))
    if model.nu != 12+2*cfg['joints_per_hand'] or model.nq != model.nu:
        raise ValueError('Unexpected joint or control dimension')
    np.testing.assert_array_equal(model.actuator_trnid[:,0],np.arange(model.nu))
    metadata={'id':cfg['id'],'configuration':cfg,'template_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'root_body_name':'left_base_link','root_bodies':['left_base_link','right_base_link'],
        'joint_names':joint_names,'actuator_names':[a.get('name') for a in actuator],
        'body_names':[b.get('name') for b in world.iter('body')],
        'arm_joint_names':[f'{s}_joint{i}' for s in ('left','right') for i in range(1,7)],
        'hand_joint_names':{s:[n for n in joint_names if n.startswith(s+'_'+kind+'_')] for s in ('left','right')},
        'hands':hands,'probe_joint_offsets':offsets,'designated_hand':cfg['designated_hand'],
        'hand_source_revision':json.loads((ROOT/cfg['source_hand_manifest']).read_text())['revision'],
        'source_simulation_options':source_options,'default_joint_positions':model.qpos0.tolist(),
        'limitations':cfg['limitations']}
    (output/'robot.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return path


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',choices=['wuji','sharpa'],required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    print(build(args.robot,args.output or ROOT/f'assets/generated/dual_x5_{args.robot}'))
