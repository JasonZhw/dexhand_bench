"""MuJoCo kinematic planning model, separate from UniLab physical execution.

Pose reachability only: joint interpolation is not collision-free planning.
"""
import mujoco
import numpy as np


class X5IK:
    def __init__(self, scene, side, tcp_offset):
        if side not in ('left', 'right') or not np.isfinite(tcp_offset):
            raise ValueError('Invalid X5 side or TCP offset')
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.body = self.model.body(side + '_link6').id
        joints = [self.model.joint(f'{side}_joint{i}') for i in range(1, 7)]
        self.qids = np.array([j.qposadr[0] for j in joints])
        self.vids = np.array([j.dofadr[0] for j in joints])
        self.lower = np.maximum([j.range[0] for j in joints], -np.pi)
        self.upper = np.minimum([j.range[1] for j in joints], np.pi)
        self.offset = np.array([tcp_offset, 0., 0.])

    def pose(self, q):
        self.data.qpos[self.qids] = q
        mujoco.mj_forward(self.model, self.data)
        body = self.data.body(self.body)
        return (body.xpos + body.xmat.reshape(3, 3) @ self.offset, body.xquat.copy())

    def solve(self, position, quaternion, seed, attempts=12):
        position, quaternion = np.asarray(position), np.asarray(quaternion)
        if position.shape != (3,) or quaternion.shape != (4,):
            raise ValueError('Expected xyz position and wxyz quaternion')
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
            raise ValueError('Target must be finite')
        if not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-6):
            raise ValueError('Target quaternion must be normalized')
        rng = np.random.default_rng(0)
        best = None
        for attempt in range(attempts):
            q = np.clip(np.asarray(seed, dtype=float) if attempt == 0 else
                        rng.uniform(-1.5, 1.5, 6), self.lower, self.upper)
            for _ in range(250):
                point, current_quat = self.pose(q)
                rotation_error = np.zeros(3)
                mujoco.mju_subQuat(rotation_error, quaternion, current_quat)
                # mju_subQuat returns the current local frame; mj_jac is world-frame.
                rotation_error = self.data.body(self.body).xmat.reshape(3, 3) @ rotation_error
                residual = dict(position_m=float(np.linalg.norm(position - point)),
                                rotation_rad=float(np.linalg.norm(rotation_error)))
                score = residual['position_m'] + .1 * residual['rotation_rad']
                if best is None or score < best[0]:
                    best = (score, q.copy(), residual)
                if residual['position_m'] < .0005 and residual['rotation_rad'] < .005:
                    return q, residual
                jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
                mujoco.mj_jac(self.model, self.data, jacp, jacr, point, self.body)
                jacobian = np.vstack((jacp[:, self.vids], .1 * jacr[:, self.vids]))
                error = np.concatenate((position - point, .1 * rotation_error))
                delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + .0001 * np.eye(6), error)
                q = np.clip(q + delta * min(1., .2 / max(np.linalg.norm(delta), 1e-12)),
                            self.lower, self.upper)
        raise ValueError(f'No reachable pose within tolerance; best={best[2]}')
