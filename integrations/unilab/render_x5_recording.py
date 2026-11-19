"""Render recorded joint/object poses without advancing physical simulation."""
import subprocess

import mujoco
import numpy as np
from PIL import Image


def render(scene, records, joint_names, output, dt):
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, -.25, .9]
    camera.distance, camera.azimuth, camera.elevation = 1.8, 120, -30
    stride = 10
    renderer = mujoco.Renderer(model, height=480, width=640)
    writer = None
    try:
        writer = subprocess.Popen([
            'ffmpeg', '-loglevel', 'error', '-f', 'rawvideo', '-pixel_format', 'rgb24',
            '-video_size', '640x480', '-framerate', str(1 / (dt * stride)), '-i', '-',
            '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(output / 'control.mp4')],
            stdin=subprocess.PIPE)
        for index, record in enumerate(records):
            if (index + 1) % stride:
                continue
            for name, value in zip(joint_names, record['qpos'][0]):
                data.qpos[model.joint(name).qposadr[0]] = value
            poses = np.asarray(record['object_pose'][0]).reshape(4, 7)
            for tube in range(3):
                address = model.joint(f'tube{tube}_free').qposadr[0]
                data.qpos[address:address + 7] = poses[tube]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            writer.stdin.write(frame.tobytes())
            if index + 1 == (len(records) // 2 // stride) * stride:
                Image.fromarray(frame).save(output / 'control.png')
    finally:
        renderer.close()
        if writer is not None:
            writer.stdin.close()
            if writer.wait() != 0:
                raise RuntimeError('ffmpeg encoding failed')
