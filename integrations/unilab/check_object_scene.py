"""Load/reset/step the object-only scene via UniLab's public backend factory.

This is a scene acceptance check, not the Manager-Based insertion environment.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import subprocess
from importlib.metadata import version

import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/UniLab/src'))
from unilab.base.backend_factory import create_backend
from unilab.base.scene import SceneCfg


def run(scene, output, steps, video=False):
    if steps <= 0:
        raise ValueError('steps must be positive')
    if video and steps < 10:
        raise ValueError('Video requires at least 10 simulation steps')
    dt = json.loads((scene.parent / 'conversion.json').read_text())['physics_dt']
    output.mkdir(parents=True, exist_ok=False)
    backend = create_backend('mujoco', SceneCfg(model_file=str(scene.resolve())),
                             num_envs=2, sim_dt=dt)
    renderer = writer = None
    try:
        backend.materialize()
        if video:
            from PIL import Image
            import mujoco
            render_model = mujoco.MjModel.from_xml_path(str(scene.resolve()))
            render_data = mujoco.MjData(render_model)
            renderer = mujoco.Renderer(render_model, height=480, width=640)
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [.15, 0, .78]
            camera.distance, camera.azimuth, camera.elevation = .8, 135, -45
            writer = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-f', 'rawvideo',
                '-pixel_format', 'rgb24', '-video_size', '640x480', '-framerate', str(1 / (dt * 10)),
                '-i', '-', '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                str(output / 'scene.mp4')], stdin=subprocess.PIPE)
        initial = {k: v.clone() for k, v in backend.get_state_views(('qpos', 'qvel')).items()}
        controls = torch.zeros((2, len(backend.get_actuator_names())), dtype=torch.float32)
        records = []
        for step in range(steps):
            backend.step_tensor(controls)
            state = backend.get_state_views(('qpos', 'qvel'))
            if not all(torch.isfinite(v).all() for v in state.values()):
                raise ValueError(f'Non-finite state at step {step}')
            records.append({'step': step + 1, 'sim_time': (step + 1) * dt,
                            'qpos': state['qpos'].tolist(), 'qvel': state['qvel'].tolist()})
            if video and (step + 1) % 10 == 0:
                render_data.qpos[:] = state['qpos'][0].cpu().numpy()
                mujoco.mj_forward(render_model, render_data)
                renderer.update_scene(render_data, camera=camera)
                frame = renderer.render()
                writer.stdin.write(frame.tobytes())
                if step == 9:
                    Image.fromarray(frame).save(output / 'scene.png')
        other = {k: v[1].clone() for k, v in state.items()}
        backend.set_state_tensor(torch.tensor([0], dtype=torch.int64),
                                 initial['qpos'][:1].float().contiguous(),
                                 initial['qvel'][:1].float().contiguous())
        reset = backend.get_state_views(('qpos', 'qvel'))
        torch.testing.assert_close(reset['qpos'][0], initial['qpos'][0], atol=1e-6, rtol=0)
        torch.testing.assert_close(reset['qvel'][0], initial['qvel'][0], atol=1e-6, rtol=0)
        for key in ('qpos', 'qvel'):
            torch.testing.assert_close(reset[key][1], other[key], atol=0, rtol=0)
        receipt = {'scope': 'UniLab backend factory object-scene acceptance only',
                   'backend': 'mujoco', 'num_envs': 2, 'steps': steps,
                   'sim_dt': dt, 'partial_reset_verified': True, 'finite_states': True,
                   'task_success': None, 'manager_environment': False,
                   'versions': {name: version(name) for name in ('unisim-core', 'mujoco', 'torch')},
                   'video': {'enabled': video, 'fps': 1 / (dt * 10), 'state_stride': 10,
                             'first_frame_sim_time': dt * 10, 'physics_steps_added': 0},
                   'scene_files_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                          for p in sorted(scene.parent.iterdir()) if p.is_file()},
                   'limitations': ['No robot or action policy', 'No insertion evaluation',
                                   'Approximate convex collision; no physical accuracy reference']}
    finally:
        video_exit = 0
        if writer is not None:
            writer.stdin.close()
            video_exit = writer.wait()
        if renderer is not None:
            renderer.close()
        backend.close()
        if video_exit != 0:
            raise RuntimeError('ffmpeg failed to encode scene video')
    (output / 'states.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in records))
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=250)
    parser.add_argument('--video', action='store_true')
    args = parser.parse_args()
    run(args.scene, args.output, args.steps, args.video)
