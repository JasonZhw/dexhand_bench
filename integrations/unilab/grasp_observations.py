"""Convert real state and explicitly reconstructed contact geometry to task signals."""
import mujoco
import numpy as np


def compute_signals(*, pose, lin_vel, ang_vel, com_offset, baseline_z,
                    finger_points, other_hand_contact, region, ever_lifted, dropped,
                    other_support=False):
    pose = np.asarray(pose)
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, pose[3:])
    rotation = rotation.reshape(3, 3)
    height = float(pose[2]-baseline_z)
    # UniSim exposes link-origin velocity; the score requests COM velocity.
    velocity = np.asarray(lin_vel)+np.cross(ang_vel, rotation@np.asarray(com_offset))
    ever_lifted = bool(ever_lifted or height >= .05)
    dropped = bool(dropped or (ever_lifted and height < .02))
    region_valid = bool(finger_points) and not (other_hand_contact or other_support)
    if region is not None and finger_points:
        local = (np.asarray(finger_points)-pose[:3])@rotation
        region_valid = bool(region_valid and np.all(local >= region['lower']) and np.all(local <= region['upper']))
    result = {'height_gain_m': height, 'speed_m_s': float(np.linalg.norm(velocity)),
              'hand_contact': bool(finger_points or other_hand_contact), 'dropped': dropped,
              'tilt_deg': float(np.rad2deg(np.arccos(np.clip(rotation[2,2],-1,1)))),
              'grasp_region_valid': region_valid}
    return result, (ever_lifted, dropped)


class ContactGeometry:
    """Collision snapshot from observed poses, not forces from the runtime solver."""
    def __init__(self, scene, robot):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.object_id = self.model.body('object').id
        self.hand_groups = {side: {label:{self.model.body(name).id for name in names}
                            for label,names in hand['fingers'].items()}
                            for side,hand in robot['hands'].items()}
        self.palms = {side:self.model.body(hand['palm']).id for side,hand in robot['hands'].items()}
        self.designated_hand = robot['designated_hand']
        self.fingers = set().union(*self.hand_groups[self.designated_hand].values())
        self.palm = self.palms[self.designated_hand]
        self.robot_bodies = {self.model.body(name).id for name in robot['body_names']}
        self.names = robot['joint_names']

    def update(self, joints, pose):
        if len(joints) != len(self.names):
            raise ValueError('Observed joint vector does not match robot joint mapping')
        for name, value in zip(self.names, joints):
            self.data.qpos[self.model.joint(name).qposadr[0]] = value
        offset = self.model.joint('object_free').qposadr[0]
        self.data.qpos[offset:offset+7] = pose
        mujoco.mj_forward(self.model, self.data)
        points, palm, other_support, penetration = [], False, False, 0.
        self.finger_contacts = {side:{label:False for label in groups} for side,groups in self.hand_groups.items()}
        self.palm_contacts = {side:False for side in self.palms}
        for contact in self.data.contact:
            pair = [self.model.geom_bodyid[contact.geom1], self.model.geom_bodyid[contact.geom2]]
            if self.object_id not in pair or contact.dist > 0:
                continue
            penetration = max(penetration, -float(contact.dist))
            hand = set(pair) & self.robot_bodies
            if hand:
                for side, groups in self.hand_groups.items():
                    for label, bodies in groups.items():
                        self.finger_contacts[side][label] |= bool(hand & bodies)
                    self.palm_contacts[side] |= self.palms[side] in hand
                if hand & self.fingers:
                    points.append(contact.pos.copy().tolist())
                if self.palm in hand:
                    palm = True
                if hand - self.fingers - {self.palm}:
                    other_support = True
        return points, palm, other_support, penetration
