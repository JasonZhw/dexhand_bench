"""Run G01-G05 through UniLab, observe and score probes or a scripted grasp."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
from importlib.metadata import version

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'third_party/UniLab/src')]
import mujoco
import numpy as np
import torch
from hydra import compose, initialize_config_dir
from hydra.utils import instantiate
from omegaconf import OmegaConf
from PIL import Image
from unilab.envs import make_manager_based_rl_env
from dexbench.evaluation import evaluate
from integrations.unilab.grasp_observations import ContactGeometry, compute_signals
from integrations.unilab.build_allegro_robot import load_robot
from integrations.unilab.grasp_profiles import load_profile, ROBOTS, TASKS
from integrations.unilab.runtime_physics import ContactSource, instrument_scene, MujocoContactSource, PhysicsRecorder


def environment_config(profile, owner_config_name='x5_control_diagnostic'):
    runtime = profile['runtime']
    if profile['backend'] != 'mujoco':
        raise ValueError('This asset/control adapter currently supports only mujoco')
    if runtime['controller'] not in ('hold', 'probe', 'ik_grasp'):
        raise ValueError('Select hold, probe or ik_grasp')
    if runtime['controller'] == 'ik_grasp' and (
            profile['task_id'] not in ('G01', 'G02') or load_robot(profile)['id'] != 'dual_x5_wuji'):
        raise ValueError('ik_grasp currently supports only G01 and G02 with wuji')
    if runtime['sim_dt'] != runtime['ctrl_dt'] or not 0 < runtime['sim_dt'] <= .1:
        raise ValueError('This first binding records every physics step, dt <= 0.1 s')
    if not isinstance(runtime['steps'], int) or runtime['steps'] < 1 or runtime['num_envs'] < 2:
        raise ValueError('Positive steps and at least two environments required for reset verification')
    with initialize_config_dir(version_base=None, config_dir=str(ROOT/'conf/unilab')):
        cfg = compose(config_name=owner_config_name)
    OmegaConf.set_struct(cfg, False)
    robot = load_robot(profile)
    entity = cfg.env.scene.entities.robot
    for key in ('root_body_name', 'joint_names', 'actuator_names', 'body_names'):
        entity[key] = robot[key]
    cfg.env.scene.model_file = str((ROOT/profile['scene']).resolve())
    cfg.env.scene.entities.objects.root_body_name = 'object'
    cfg.env.scene.entities.objects.body_names = ['object']
    terms = cfg.env.observations.policy.terms
    del terms['object_pose']
    del terms['source_geometry']
    terms.object_state = {
        '_target_':'unilab.managers.ObservationTermCfg',
        'func':{'_target_':'hydra.utils.get_class', 'path':'integrations.unilab.grasp_terms.GraspObjectState'},
        'params':{'entity_name':'objects','body_names':['object']}}
    cfg.env.metrics = {'object_height_m':{
        '_target_':'unilab.managers.MetricsTermCfg',
        'func':{'_target_':'hydra.utils.get_class','path':'integrations.unilab.object_scene_terms.MeanHeight'},
        'params':{'entity_name':'objects','body_names':['object']},'reduce':'last'}}
    task = json.loads((ROOT/profile['task_file']).read_text())
    if task['id'] != profile['task_id']:
        raise ValueError('Launch profile must reference the matching task contract')
    cfg.env.sim_dt, cfg.env.ctrl_dt = runtime['sim_dt'], runtime['ctrl_dt']
    cfg.env.max_episode_seconds = task['benchmark']['episode']['timeout_s']
    if runtime['steps']*runtime['ctrl_dt'] >= cfg.env.max_episode_seconds:
        raise ValueError('Diagnostic must finish before task timeout')
    return cfg.env


def validate_scene_profile(profile, receipt):
    built = receipt['profile']
    if (receipt['task_id'] != profile['task_id']
            or any(profile[key] != built[key] for key in ('object','initial','robot_template','robot_config'))
            or profile['runtime']['sim_dt'] != built['runtime']['sim_dt']):
        raise ValueError('Physical launch configuration changed; rebuild the scene into a new directory')
    if 'robot' in receipt and receipt['robot'] != load_robot(profile):
        raise ValueError('Robot metadata changed; rebuild the scene')


def probe_action(robot, num_envs, amplitude):
    """Small named joint offsets in environment zero, never a grasp policy."""
    action = np.zeros((num_envs, len(robot['joint_names'])), dtype=np.float32)
    if 'probe_joint_offsets' in robot:
        for name, delta in robot['probe_joint_offsets'].items():
            action[0, robot['joint_names'].index(name)] = delta*amplitude
        return action
    for side, sign in (('left',-1), ('right',1)):
        action[0,robot['joint_names'].index(f'{side}_joint1')] = sign*.08*amplitude
        for finger in ('ff','mf','rf','th'):
            for index, delta in ((1,.35),(2,.25),(3,.2)):
                name = f'{side}_allegro_{finger}j{index}'
                action[0,robot['joint_names'].index(name)] = delta*amplitude
    return action


def validate_scene_assets(scene, receipt):
    if hashlib.sha256(scene.read_bytes()).hexdigest() != receipt['scene_sha256']:
        raise ValueError('Scene differs from its build receipt; rebuild before running')
    dependencies = receipt.get('mesh_dependencies')
    if not dependencies:
        raise ValueError('Missing external mesh checksums; rebuild the scene')
    for filename, digest in dependencies.items():
        path = Path(filename)
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f'External mesh changed or missing: {filename}; rebuild before running')


def render_recording(scene, records, output, profile):
    robot = load_robot(profile)
    probe = ContactGeometry(scene, robot)
    camera = mujoco.MjvCamera()
    settings = profile['runtime']['camera']
    camera.lookat[:] = settings['lookat']
    camera.distance, camera.azimuth, camera.elevation = settings['distance'], settings['azimuth'], settings['elevation']
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # Show source surface, retain collision shapes in physics.
    stride = 10
    renderer = mujoco.Renderer(probe.model, height=480, width=640)
    hand_camera = mujoco.MjvCamera()
    hand_view = profile['runtime'].get('hand_camera', {'distance': .47, 'azimuth': 115, 'elevation': -50})
    hand_camera.distance = hand_view['distance']
    hand_camera.azimuth, hand_camera.elevation = hand_view['azimuth'], hand_view['elevation']
    writers = []
    try:
        for name in ('preview.mp4', 'hand.mp4'):
            writers.append(subprocess.Popen(['ffmpeg','-loglevel','error','-f','rawvideo','-pixel_format','rgb24',
                '-video_size','640x480','-framerate',str(1/(profile['runtime']['ctrl_dt']*stride)),
                '-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(output/name)],stdin=subprocess.PIPE))
        for index, record in enumerate(records):
            if index % stride:
                continue
            probe.update(record['qpos'][0], record['pose'][0])
            renderer.update_scene(probe.data, camera=camera, scene_option=option)
            frame = renderer.render()
            if index == 0:
                Image.fromarray(frame).save(output/'preview.png')
            else:
                writers[0].stdin.write(frame.tobytes())
            hand_camera.lookat[:] = probe.data.body(robot['hands']['right']['palm']).xpos + np.array([0,.035,0])
            renderer.update_scene(probe.data, camera=hand_camera, scene_option=option)
            hand_frame = renderer.render()
            if index:
                writers[1].stdin.write(hand_frame.tobytes())
            if index == (len(records)//2)//stride*stride:
                Image.fromarray(hand_frame).save(output/'hand.png')
        # Close-up for inspecting the actual object independently of robot framing.
        camera.lookat[:] = np.asarray(records[-1]['pose'][0][:3])+np.array([0,0,.04])
        camera.distance = max(.25, float(np.max(np.array(json.loads((scene.parent/'scene.json').read_text())['geometry']['extents_m'])))*2.5)
        camera.azimuth, camera.elevation = 120, -40
        renderer.update_scene(probe.data, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(output/'object.png')
    finally:
        renderer.close()
        failed = False
        for writer in writers:
            writer.stdin.close()
            if writer.wait() != 0:
                failed = True
        if failed:
            raise RuntimeError('Video encoding failed')


def run(profile, output, video=False, live=False, viewer_port=8080,
        owner_config_name='x5_control_diagnostic'):
    cfg = environment_config(profile, owner_config_name)
    output = Path(output)
    task = json.loads((ROOT/profile['task_file']).read_text())
    spec = task['benchmark']
    scene = ROOT/profile['scene']
    scene_receipt = json.loads((scene.parent/'scene.json').read_text())
    validate_scene_profile(profile,scene_receipt)
    validate_scene_assets(scene,scene_receipt)
    recording = profile.get('physics_recording', {})
    enabled = recording.get('enabled', False)
    warmup_steps = recording.get('warmup_steps', 0)
    if type(enabled) is not bool or type(warmup_steps) is not int or warmup_steps < 0:
        raise ValueError('Recording enabled must be boolean; warmup_steps must be a nonnegative integer')
    output.mkdir(parents=True, exist_ok=False)
    robot = scene_receipt['robot']
    manifest = None
    if enabled:
        runtime_scene = output/'runtime-scene.xml'
        manifest = instrument_scene(scene, runtime_scene, robot, recording.get('contact_slots',32))
        cfg.scene.model_file = str(runtime_scene.resolve())
        (output/'sensor-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'config.json').write_text(json.dumps({'profile':profile,'env':OmegaConf.to_container(cfg,resolve=True)},indent=2)+'\n')
    env = make_manager_based_rl_env(instantiate(cfg,_convert_='all'), num_envs=profile['runtime']['num_envs'], backend_type=profile['backend'])
    probes = [ContactGeometry(scene, robot) for _ in range(env.num_envs)]
    viewer = None
    if live:
        try:
            from integrations.unilab.grasp_viewer import GraspViewer
            viewer = GraspViewer(env, port=viewer_port, camera=profile['runtime']['camera'])
        except BaseException:
            env.close()
            raise
    traces = [{'task_id':task['id'],'protocol_version':spec['protocol_version'],'kind':'recorded',
               'complete':False,'frames':[]} for _ in range(env.num_envs)]
    records, physics, history = [], [], [(False,False) for _ in range(env.num_envs)]
    step_seconds = []
    source: ContactSource | None = None
    recorder = PhysicsRecorder(num_envs=env.num_envs,designated_hand=robot['designated_hand'],dt=cfg.ctrl_dt) if enabled else None
    try:
        if viewer is not None:
            viewer.wait_for_client()
        if enabled:
            source = MujocoContactSource(env.backend,manifest)
            (output/'physics-capabilities.json').write_text(json.dumps(source.metadata(),indent=2)+'\n')
        env.reset()
        warm_action=torch.zeros((env.num_envs,len(robot['joint_names'])),device=env.device)
        for _ in range(warmup_steps):
            env.step(warm_action)
        # MuJoCo CPU steps finish synchronously. Warmup never consumes a scored step.
        initial, _ = env.reset()
        initial_obs = initial['obs'].clone()
        def read():
            body = env.scene.read_plan.body_tensor_view(env.scene['objects'],('object',))
            joint = env.scene.read_plan.joint_tensor_view(env.scene['robot'])
            return (torch.cat((body.pos_w,body.quat_w),-1)[:,0].cpu().numpy().copy(),
                    body.lin_vel_w[:,0].cpu().numpy().copy(), body.ang_vel_w[:,0].cpu().numpy().copy(),
                    joint.joint_pos.cpu().numpy().copy(),joint.joint_vel.cpu().numpy().copy())
        initial_state = read()
        home = initial_state[3].copy()
        controller = None
        stage = profile['runtime']['controller']
        if stage == 'ik_grasp':
            from integrations.unilab.grasp_ik import GraspIKController
            controller = GraspIKController(scene, robot, home, initial_state[0], profile['scripted_grasp'])
            (output/'controller-plan.json').write_text(json.dumps(controller.plan,indent=2)+'\n')
        def sample(step, action):
            if recorder is not None:
                recorder.record(step,source.snapshot(),step_seconds[-1] if step else None)
            pose, linear, angular, q, qv = read()
            if not all(np.isfinite(x).all() for x in (pose,linear,angular,q,qv)):
                raise RuntimeError('Nonfinite simulation state')
            records.append({'step':step,'t_s':step*cfg.ctrl_dt,'controller_phase':stage,'pose':pose.tolist(),
                            'linear_velocity':linear.tolist(),'angular_velocity':angular.tolist(),
                            'qpos':q.tolist(),'qvel':qv.tolist(),'actions':action.tolist()})
            for index in range(env.num_envs):
                points, palm, other_support, penetration = probes[index].update(q[index],pose[index])
                if index == 0 and viewer is not None and step % max(1, round(.05/cfg.ctrl_dt)) == 0:
                    viewer.update()
                values, history[index] = compute_signals(
                    pose=pose[index],lin_vel=linear[index],ang_vel=angular[index],
                    com_offset=scene_receipt['object_com_offset_m'],baseline_z=initial_state[0][index,2],
                    finger_points=points,other_hand_contact=palm,other_support=other_support,
                    region=profile['object']['grasp_region'],
                    ever_lifted=history[index][0],dropped=history[index][1])
                traces[index]['frames'].append({'t_s':step*cfg.ctrl_dt,'signals':{k:values[k] for k in spec['signals']}})
                physics_row = {'step':step,'env_id':index,'contact_force_n':None,'slip_m_s':None,
                    'finger_contacts':probes[index].finger_contacts,'palm_contacts':probes[index].palm_contacts,
                    'penetration_m':penetration,'penetration_method':'offline forward contact geometry from recorded poses; maximum over all object contacts including table',
                    'step_time_s':step_seconds[-1] if step else None}
                if recorder is not None:
                    runtime_row = recorder.rows[-env.num_envs+index]
                    for key in ('contact_force_n','hand_contact_force_n','penetration_m','slip_m_s',
                                'slip_missing_reason','solver_t_s','state_t_s','sample_phase'):
                        physics_row[key] = runtime_row[key]
                    physics_row['offline_geometry_penetration_m'] = penetration
                    physics_row['penetration_method'] = 'runtime object_all contact sensor at solver_t_s'
                physics.append(physics_row)
        action = np.zeros_like(home)
        if controller is not None:
            action, stage = controller.action(0.)
        sample(0,action)
        steps = profile['runtime']['steps']
        for step in range(1,steps+1):
            if controller is not None:
                action, stage = controller.action(step*cfg.ctrl_dt)
            elif profile['runtime']['controller'] == 'probe':
                alpha = np.sin(np.pi*step/steps)
                action = probe_action(robot,env.num_envs,alpha)
            action_tensor = torch.as_tensor(action,device=env.device)
            start = time.perf_counter()
            state = env.step(action_tensor)
            step_seconds.append(time.perf_counter()-start)
            if state.terminated.any() or state.truncated.any():
                raise RuntimeError('Unexpected episode end during diagnostic')
            sample(step,action)
        before_reset = read()
        after, _ = env.reset(torch.tensor([0],device=env.device))
        reset_state = read()
        torch.testing.assert_close(after['obs'][0],initial_obs[0],atol=1e-6,rtol=0)
        for initial_array, before_array, reset_array in zip(initial_state,before_reset,reset_state):
            np.testing.assert_allclose(reset_array[0],initial_array[0],atol=1e-6,rtol=0)
            np.testing.assert_array_equal(reset_array[1:],before_array[1:])
        for trace in traces:
            trace['complete'] = True
    except BaseException:
        if viewer is not None:
            viewer.close()
        raise
    finally:
        env.close()
        (output/'states.json').write_text(json.dumps(records)+'\n')
        (output/'physics.json').write_text(json.dumps(physics)+'\n')
        if recorder is not None:
            (output/'runtime-physics.json').write_text(json.dumps(recorder.rows,allow_nan=False)+'\n')
            summary=recorder.result()
            summary['timing']={'warmup_steps':warmup_steps,'measured_steps':len(step_seconds),
                'batch_size':env.num_envs,'synchronization':'synchronous MuJoCo CPU env.step',
                'scope':'batch env.step wall time, includes managers and authored sensors; excludes recording, offline geometry, rendering, reset and warmup; diagnostic timing, not a throughput benchmark',
                'mean_batch_step_s':float(np.mean(step_seconds)) if step_seconds else None,
                'p95_batch_step_s':float(np.percentile(step_seconds,95)) if step_seconds else None}
            (output/'physics-summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        for index, trace in enumerate(traces):
            (output/f'trace-{index}.json').write_text(json.dumps(trace)+'\n')
    scores = [evaluate(task['id'],spec,trace) for trace in traces]
    (output/'evaluation.json').write_text(json.dumps(scores,indent=2)+'\n')
    if video:
        render_recording(scene,records,output,profile)
    motion = np.max(np.abs(np.array([r['qpos'][0] for r in records])-home[0]), axis=0)
    receipt = {'task_id':task['id'],'scope': 'scripted IK grasp attempt' if controller else 'local arm/dexterous-hand articulation and grasp-scene diagnostic',
        'robot_id':robot['id'],'joint_names':robot['joint_names'],
        'num_arm_joints':len(robot['arm_joint_names']),
        'num_hand_joints':{s:len(names) for s,names in robot['hand_joint_names'].items()},
        'controller':profile['runtime']['controller'],'successful_grasp_policy_available':False,
        'task_success':[x['success'] for x in scores],'num_envs':env.num_envs,'steps':steps,
        'simulated_seconds':steps*cfg.ctrl_dt,'reset_pose_and_velocity_verified':True,
        'independent_reset_verified':True,'backend':'mujoco','execution':'UniLab ManagerBasedRlEnv',
        'object_dynamic':True,'object_pose_commands_during_execution':False,
        'robot_max_motion_rad':float(np.max(motion)),
        'hand_joint_motion_rad':{s:{n:float(motion[robot['joint_names'].index(n)]) for n in names}
                                 for s,names in robot['hand_joint_names'].items()},
        'physics_force_available':source is not None,
        'contact_method':'task scoring: offline pose-based geometry; physics diagnostics: runtime sensors' if enabled else 'offline pose-based geometry, not runtime force sensor',
        'physics_recording':{'enabled':enabled,'warmup_steps':warmup_steps,'runtime_scene_sha256':manifest['runtime_scene_sha256'] if manifest else None},
        'timing_scope':'CPU batch env.step wall time, includes managers and enabled authored sensors; excludes recording, geometry, warmup and rendering; not a throughput benchmark',
        'versions':{n:version(n) for n in ('unisim-core','mujoco','mjbatch-uni','torch')},
        'python':sys.version,'platform':platform.platform(),
        'task_file_sha256':hashlib.sha256((ROOT/profile['task_file']).read_bytes()).hexdigest(),
        'scene_sha256':scene_receipt['scene_sha256'],'limitations':profile['limitations']}
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(task['id'],'rendered=',video,'success=',receipt['task_success'],flush=True)
    if viewer is not None:
        print('UniLab Viser remains open at', viewer.url, '— Ctrl+C to exit.', flush=True)
        try:
            while True:
                time.sleep(.2)
        except KeyboardInterrupt:
            pass
        finally:
            viewer.close()
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',choices=['G01','G02','G03','G04','G05','all'])
    parser.add_argument('--robot',choices=ROBOTS)
    parser.add_argument('--launch-config',help='Hydra owner in conf/unilab, e.g. grasp_g02_wuji')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--video',action='store_true')
    parser.add_argument('--live',action='store_true',help='Show live UniLab env 0 in the official Viser browser renderer; do not save video')
    parser.add_argument('--viewer-port',type=int)
    parser.add_argument('--controller',choices=['hold','probe','ik_grasp'])
    parser.add_argument('--controller-config',type=Path,help='Defaults to profiles/controllers/<task>_wuji_ik.json')
    parser.add_argument('--physics',choices=['on','off'],help='Override recording without changing warmup; useful for observer parity checks')
    args = parser.parse_args()
    owner_config_name = 'x5_control_diagnostic'
    if args.launch_config:
        with initialize_config_dir(version_base=None, config_dir=str(ROOT/'conf/unilab')):
            launch = compose(config_name=args.launch_config)
        if launch.scope != 'grasp_g02_wuji':
            parser.error('Unsupported grasp launch owner')
        for field, expected in (('task',launch.task_id),('robot',launch.robot),('controller',launch.controller)):
            selected = getattr(args,field)
            if selected is not None and selected != expected:
                parser.error(f'--{field} conflicts with the launch owner ({expected})')
            setattr(args,field,str(expected))
        owner_config_name = args.launch_config
        if args.controller_config is None:
            args.controller_config = ROOT/str(launch.controller_config)
        if args.viewer_port is None:
            args.viewer_port = int(launch.viewer.port)
        if str(launch.viewer.mode) == 'viser':
            args.live = True
    args.task = args.task or 'all'
    args.robot = args.robot or 'allegro'
    args.viewer_port = args.viewer_port or 8080
    if args.controller == 'ik_grasp' and (args.task not in ('G01','G02') or args.robot != 'wuji'):
        parser.error('ik_grasp requires --task G01 or G02 and --robot wuji')
    for task_id in TASKS:
        profile=load_profile(task_id,args.robot)
        if args.task in ('all',profile['task_id']):
            if args.controller:profile['runtime']['controller']=args.controller
            if profile['runtime']['controller']=='ik_grasp':
                controller_config=args.controller_config or ROOT/f'profiles/controllers/{task_id}_wuji_ik.json'
                profile['scripted_grasp']=json.loads(controller_config.read_text())
                profile['runtime']['steps']=int(np.ceil(sum(profile['scripted_grasp']['phases_s'].values())/profile['runtime']['ctrl_dt']))
            if args.physics:profile.setdefault('physics_recording',{})['enabled']=args.physics=='on'
            run(profile,args.output/profile['task_id'],args.video,args.live,args.viewer_port,owner_config_name)
