"""Offline MuJoCo contact geometry from recorded poses; never infer forces.

This is a backend-specific diagnostic extension. It does not reproduce the
original solver's contact forces, velocities, warm starts or impulses.
"""
import mujoco
import numpy as np


def diagnose(scene, records, joint_names, side, target_body):
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    target = model.body(target_body).id
    finger_ids = [model.body(f'{side}_link{i}').id for i in (7, 8)]
    rows = []
    for record in records:
        if record['phase'] not in ('close', 'lift', 'hold'):
            continue
        for name, value in zip(joint_names, record['qpos'][0]):
            data.qpos[model.joint(name).qposadr[0]] = value
        for tube, pose in enumerate(np.asarray(record['object_pose'][0]).reshape(4, 7)[:3]):
            address = model.joint(f'tube{tube}_free').qposadr[0]
            data.qpos[address:address + 7] = pose
        mujoco.mj_forward(model, data)
        contact = [False, False]
        penetration = 0.
        for item in data.contact:
            bodies = [model.geom_bodyid[item.geom1], model.geom_bodyid[item.geom2]]
            if target not in bodies:
                continue
            for index, finger in enumerate(finger_ids):
                if finger in bodies and item.dist <= 0:
                    contact[index] = True
                    penetration = max(penetration, -float(item.dist))
        rows.append(dict(step=record['step'], phase=record['phase'],
                         finger_contacts=contact, maximum_penetration_m=penetration))
    hold = [r for r in rows if r['phase'] == 'hold']
    return dict(method='offline MuJoCo forward contact geometry from recorded poses',
                runtime_contact_force_available=False, runtime_contact_force=None,
                maximum_finger_penetration_m=max((r['maximum_penetration_m'] for r in rows), default=None),
                phase_maximum_penetration_m={phase: max((r['maximum_penetration_m'] for r in rows
                                                         if r['phase'] == phase), default=None)
                                             for phase in ('close', 'lift', 'hold')},
                hold_bilateral_contact_fraction=(sum(all(r['finger_contacts']) for r in hold) / len(hold)
                                                 if hold else None),
                samples=rows)
