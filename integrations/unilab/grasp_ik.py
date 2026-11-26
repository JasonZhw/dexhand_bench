"""Fixed-scene IK arm waypoints plus authored finger closure, executed via UniLab.

IK plans joint targets only. This module neither steps physics nor commands the
object. Joint-space interpolation is not collision-free motion planning.
"""
import numpy as np
import mujoco

from integrations.unilab.x5_ik import X5IK

PHASES = ('settle', 'raise', 'approach', 'descend', 'close', 'lift', 'hold')


class GraspIKController:
    def __init__(self, scene, robot, home, object_pose, settings):
        if robot['id'] != 'dual_x5_wuji' or robot['designated_hand'] != 'right':
            raise ValueError('Grasp IK supports only the right wuji hand')
        self.home = np.asarray(home, dtype=float).copy()
        self.durations = np.array([settings['phases_s'][p] for p in PHASES])
        if not np.isfinite(self.durations).all() or np.any(self.durations <= 0):
            raise ValueError('Phase durations must be finite and positive')
        self.ends = np.cumsum(self.durations)
        self.duration_s = float(self.ends[-1])
        solver = X5IK(scene, 'right', 0.)
        model, data = solver.model, solver.data
        names = robot['joint_names']
        for name, value in zip(names, self.home[0]):
            data.qpos[model.joint(name).qposadr[0]] = value
        mujoco.mj_forward(model, data)
        arm_ids = [names.index(f'right_joint{i}') for i in range(1, 7)]
        seed = self.home[0, arm_ids].copy()
        link = data.body('right_link6')
        quat = link.xquat.copy()
        palm = data.body(robot['hands']['right']['palm'])
        palm_home = palm.xpos.copy()
        palm_rotation = palm.xmat.reshape(3, 3).copy()
        fixed_world_palm_offset = palm_home - link.xpos
        rotation_delta = np.asarray(settings.get('palm_rotation_delta_world_wxyz', [1., 0., 0., 0.]))
        if rotation_delta.shape != (4,) or not np.isclose(np.linalg.norm(rotation_delta), 1., atol=1e-6):
            raise ValueError('Palm rotation delta must be a normalized wxyz quaternion')
        delta_matrix = np.zeros(9)
        mujoco.mju_quat2Mat(delta_matrix, rotation_delta)
        delta_matrix = delta_matrix.reshape(3, 3)
        grasp_quat = np.zeros(4)
        mujoco.mju_mulQuat(grasp_quat, rotation_delta, quat)
        grasp_offset = delta_matrix @ fixed_world_palm_offset
        palm_rotation = delta_matrix @ palm_rotation
        obj_rotation = np.zeros(9)
        mujoco.mju_quat2Mat(obj_rotation, np.asarray(object_pose[0, 3:7], dtype=float))
        center = object_pose[0, :3] + obj_rotation.reshape(3, 3) @ model.body('object').ipos
        grasp = center - palm_rotation @ np.asarray(settings['object_center_in_palm_m'])
        above = grasp + [0, 0, settings['approach_clearance_m']]
        raised = palm_home.copy()
        raised[2] = max(palm_home[2], above[2])
        lifted = grasp + [0, 0, settings['lift_height_m']]
        points = (palm_home, raised, above, grasp, grasp, lifted, lifted)
        closed = self.home[0].copy()
        for name, value in settings['closed_joint_positions_rad'].items():
            if name not in robot['hand_joint_names']['right']:
                raise ValueError(f'Closure target is not a right-hand joint: {name}')
            joint = model.joint(name)
            if not np.isfinite(value) or not joint.range[0] <= value <= joint.range[1]:
                raise ValueError(f'Closure target outside joint range: {name}')
            closed[names.index(name)] = value
        self.targets = []
        waypoints = []
        for i, (phase, point) in enumerate(zip(PHASES, points)):
            q = self.home[0].copy() if i < 4 else closed.copy()
            if phase == 'settle':
                residual = {'position_m': 0., 'rotation_rad': 0.}
            else:
                if phase not in ('close', 'hold'):
                    offset = fixed_world_palm_offset if phase == 'raise' else grasp_offset
                    orientation = quat if phase == 'raise' else grasp_quat
                    seed, residual = solver.solve(point-offset, orientation, seed)
                q[arm_ids] = seed
            self.targets.append(q)
            waypoints.append({'phase': phase, 'end_s': float(self.ends[i]),
                              'palm_position_m': point.tolist(), 'qpos': q.tolist(),
                              'ik_residual': residual})
        self.plan = {'controller': 'ik_grasp', 'settings': settings,
                     'scope': 'fixed initial scene; environment 0 scripted, others hold HOME',
                     'object_center_world_m': center.tolist(), 'waypoints': waypoints,
                     'object_pose_commands': False, 'learned_policy': False}

    def action(self, time_s):
        index = min(int(np.searchsorted(self.ends, time_s, side='right')), len(PHASES)-1)
        start = 0 if index == 0 else self.ends[index-1]
        alpha = np.clip((time_s-start)/self.durations[index], 0., 1.)
        alpha = alpha*alpha*(3-2*alpha)
        previous = self.home[0] if index == 0 else self.targets[index-1]
        target = previous + alpha*(self.targets[index]-previous)
        action = np.zeros_like(self.home, dtype=np.float32)
        action[0] = target-self.home[0]
        return action, PHASES[index]
