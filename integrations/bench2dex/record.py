"""Incremental JSONL evidence writer; incomplete runs remain visibly incomplete."""
import hashlib
import json
import time
from pathlib import Path
from importlib.metadata import version, PackageNotFoundError


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


class Recorder:
    def __init__(self,directory,manifest):
        self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=False)
        self.manifest=manifest
        (self.directory/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
        self.stream=(self.directory/'states.jsonl').open('x',buffering=1)
        self.steps=0
        self.start=time.monotonic()
    def write(self,row):
        encoded=json.dumps(row,allow_nan=False)
        self.stream.write(encoded+'\n')
        if row.get('phase')=='replay':self.steps+=1
    def finish(self):
        self.stream.close()
        receipt={'completed':self.steps==self.manifest.get('expected_steps'),
                 'recorded_replay_steps':self.steps,
                 'wall_seconds_including_recording':time.monotonic()-self.start}
        (self.directory/'completion.json').write_text(json.dumps(receipt,indent=2)+'\n')


def start_recording(args, robot, obj, physics_dt, stride, frames, initial_z, source_script):
    packages={}
    for name in ('isaacsim','isaaclab','torch','numpy'):
        try:packages[name]=version(name)
        except PackageNotFoundError:packages[name]=None
    manifest={'schema_version':1,'mode':'dynamic','backend':'isaacsim_physx','packages':packages,
        'task':'apple_body_lift_diagnostic','target_object':args.target_object,
        'initial_z_m':float(initial_z),'physics_dt_s':physics_dt,'control_stride':stride,
        'expected_steps':frames*stride,'warmup_steps':args.warmup_steps,
        'interpolation':not args.no_interp,'gravity_compensation':True,
        'joint_names':list(robot.joint_names),'arguments':vars(args),
        'source_hdf5_sha256':sha256(args.hdf5),'generated_script_sha256':sha256(source_script),
        'scene_sha256':sha256(args.scene) if args.scene else None,
        'dynamic_target_verified':False,
        'dynamic_target_verification':'configuration-only guard; runtime rigid-body flags not audited',
        'capabilities':{'contact_force':False,'hand_support_verification':False},
        'sampling':'post-step, after update_sim_objects; initial and warmup are separate phases',
        'units':{'position':'m','time':'s','joint_angles':'rad','quaternion':'wxyz'},
        'limitations':['initial velocities follow upstream replay (zeroed)',
                       'solver warm-start state unavailable','asset content hashes not yet captured',
                       'diagnostic host recording overhead included; not a throughput benchmark']}
    return Recorder(args.record_dir,manifest)


def snapshot(writer, robot, obj, target, t_s, phase):
    def cpu(tensor):return tensor[0].detach().cpu().tolist()
    pos=cpu(obj.data.root_pos_w)
    quat=cpu(obj.data.root_quat_w)
    writer.write({'phase':phase,'t_s':float(t_s),
        'object_pose_wxyz':pos+quat,'object_velocity_world':cpu(obj.data.root_vel_w),
        'joint_positions':cpu(robot.data.joint_pos),'joint_velocities':cpu(robot.data.joint_vel),
        'command_targets':cpu(target),'hand_support_verified':None,'contacts':None})
