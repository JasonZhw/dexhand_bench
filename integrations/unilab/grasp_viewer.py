"""Show a running grasp task with UniLab's Viser playback components."""

import time

import mujoco
import numpy as np
import viser

from unilab.visualization.playback_state import (
    PhysicsStateApplier,
    assert_physics_state_playback_supported,
)
from unilab.visualization.viser_scene import MujocoViserScene


class GraspViewer:
    """Render the UniLab environment state; never step a second simulator."""

    def __init__(self, env, port=8080, env_index=0, camera=None):
        assert_physics_state_playback_supported(env, entrypoint='grasp viewer')
        self.env = env
        self.env_index = env_index
        model = env.get_playback_model(env_index)
        if not isinstance(model, mujoco.MjModel):
            raise TypeError('The MuJoCo grasp owner must expose its playback MjModel')
        self.data = mujoco.MjData(model)
        self.applier = PhysicsStateApplier(env, model, env_index=env_index)
        self.server = viser.ViserServer(host='127.0.0.1', port=port)
        try:
            if camera is not None:
                look_at = np.asarray(camera['lookat'], dtype=np.float64)
                azimuth = np.deg2rad(camera['azimuth'])
                elevation = np.deg2rad(camera['elevation'])
                distance = float(camera['distance'])
                offset = distance * np.array([
                    np.cos(elevation) * np.cos(azimuth),
                    np.cos(elevation) * np.sin(azimuth),
                    -np.sin(elevation),
                ])
                self.server.initial_camera.position = tuple(look_at + offset)
                self.server.initial_camera.look_at = tuple(look_at)
            self.scene = MujocoViserScene(self.server, model, name_prefix='/grasp')
        except BaseException:
            self.server.stop()
            raise
        self.url = f'http://localhost:{self.server.get_port()}'

    def wait_for_client(self):
        print(f'UniLab Viser: {self.url} — open this URL to start the task', flush=True)
        while not self.server.get_clients():
            time.sleep(.1)

    def update(self):
        state = self.env.get_physics_state_snapshot()
        self.applier.apply(state[self.env_index], self.data)
        self.scene.update(self.data)

    def close(self):
        self.scene.close()
        self.server.stop()
