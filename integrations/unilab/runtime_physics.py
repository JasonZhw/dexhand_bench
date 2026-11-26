"""Runtime contact recording; task scoring never consumes these diagnostics."""
import copy
import hashlib
import xml.etree.ElementTree as ET
from typing import Protocol

import numpy as np

from dexbench.probes import ContactEnvelope
from dexbench.physics import contact_metrics


CONTACT_FIELDS = 'found force torque dist pos normal tangent'
CONTACT_WIDTH = 17
GROUPS = ('object_all', 'table', 'left_hand', 'right_hand')


class ContactSource(Protocol):
    """A backend owns sampling; the recorder only consumes normalized contacts."""
    def metadata(self) -> dict: ...
    def snapshot(self) -> list[dict]: ...


def instrument_scene(source, destination, robot, slots=32):
    """Create a sensor-only derivative without changing original scene assets."""
    if type(slots) is not int or slots < 1:
        raise ValueError('contact_slots must be a positive integer')
    root = ET.parse(source).getroot()
    sensors = root.find('sensor')
    if sensors is None:
        sensors = ET.SubElement(root, 'sensor')
    selectors = {'object_all': {'body1':'object'},
                 'table': {'body1':'object', 'geom2':'diagnostic_table'}}
    for side in ('left', 'right'):
        selectors[side+'_hand'] = {'body1':'object', 'subtree2':robot['hands'][side]['palm']}
    groups = {}
    for group in GROUPS:
        name = 'dexbench_contact_'+group
        if any(node.get('name') == name for node in sensors):
            raise ValueError('Scene already contains recorder sensors')
        ET.SubElement(sensors, 'contact', name=name, num=str(slots),
                      data=CONTACT_FIELDS, reduce='mindist', **selectors[group])
        groups[group] = {'name':name, 'slots':slots, 'selector':selectors[group]}
    ET.indent(root)
    ET.ElementTree(root).write(destination,encoding='unicode')
    return {'schema_version':'1.0', 'groups':groups, 'fields':CONTACT_FIELDS.split(),
            'slot_width':CONTACT_WIDTH,
            'source_scene_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'runtime_scene_sha256':hashlib.sha256(destination.read_bytes()).hexdigest()}


def decode_contacts(values, manifest):
    """Normalize native sensor slots; preserve incomplete samples explicitly."""
    result = {}
    for group,item in manifest['groups'].items():
        array = np.asarray(values[group],dtype=float).reshape(item['slots'],CONTACT_WIDTH)
        if not np.isfinite(array).all():
            raise ValueError('Nonfinite runtime contact sensor')
        found = array[:,0]
        count = int(np.max(found))
        contacts = []
        for row in array[found > 0]:
            normal,tangent = row[11:14],row[14:17]
            contacts.append({'distance_m':float(row[7]),
                'force_local_N_Nm':row[1:7].tolist(), 'position_world_m':row[8:11].tolist(),
                'frame_world_rows':np.stack([normal,tangent,np.cross(normal,tangent)]).tolist()})
        overflow = count > item['slots']
        metrics = contact_metrics(None if overflow else contacts)
        result[group] = {'matched_contacts':count,'recorded_contacts':len(contacts),
            'overflow':overflow,'contacts':contacts,
            'normal_force_sum_n':metrics['normal_force_sum_N'],
            'max_penetration_m':metrics['max_penetration_m'],
            'max_tangential_speed_m_s':None}
    return result


class MujocoContactSource:
    """Read authored sensors through UniSim's public named sensor binding."""
    def __init__(self, backend, manifest):
        if backend.backend_type != 'mujoco':
            raise ValueError('This contact source supports only mujoco')
        self.num_envs = backend.num_envs
        self.manifest = manifest
        model = backend.model
        import mujoco
        self.settings = {
            'timestep_s':float(model.opt.timestep),
            'solver':mujoco.mjtSolver(int(model.opt.solver)).name,
            'iterations':int(model.opt.iterations),'tolerance':float(model.opt.tolerance),
            'integrator':mujoco.mjtIntegrator(int(model.opt.integrator)).name,
            'cone':mujoco.mjtCone(int(model.opt.cone)).name,
            'impratio':float(model.opt.impratio),'gravity_m_s2':model.opt.gravity.tolist(),
            'object_mass_kg':float(model.body('object').mass[0])}
        self.views = {group:backend.bind_sensor_data((item['name'],))
                      for group,item in manifest['groups'].items()}
        for group,view in self.views.items():
            if tuple(view.dimensions) != (manifest['groups'][group]['slots']*CONTACT_WIDTH,):
                raise ValueError('Runtime contact sensor width differs from manifest')

    def metadata(self):
        return {'backend':'mujoco','interface':'SimBackend.bind_sensor_data',
                'effective_settings':self.settings,
                'capabilities':{'runtime_contact_force':True,'runtime_contact_distance':True,
                                'relative_contact_velocity':False},
                'sampling':'last physical solve before integration; no extra forward',
                'force_definition':'sum of absolute normal forces per group, N; not net force',
                'slip_missing_reason':'unsupported_contact_velocity_in_pinned_mujoco',
                'group_overlap':'object_all contains table and both hands; do not add groups together',
                'manifest':self.manifest}

    def snapshot(self):
        values = {group:view.read() for group,view in self.views.items()}
        return [decode_contacts({group:array[index] for group,array in values.items()},self.manifest)
                for index in range(self.num_envs)]


class PhysicsRecorder:
    """Independent per-env probes consuming one completed step's contacts."""
    def __init__(self, *, num_envs, designated_hand, dt):
        self.num_envs,self.hand,self.dt = num_envs,designated_hand+'_hand',dt
        self.rows = []
        self.probes = [{group:ContactEnvelope() for group in GROUPS} for _ in range(num_envs)]
        self.slip_max = [None]*num_envs
        self.overflow_samples = [0]*num_envs
        for probes in self.probes:
            for probe in probes.values():probe.reset({})

    def record(self, step, samples, step_time_s):
        if len(samples) != self.num_envs:
            raise ValueError('Contact sample environment count mismatch')
        for index,groups in enumerate(samples):
            all_contacts,hand = groups['object_all'],groups[self.hand]
            slip = hand['max_tangential_speed_m_s']
            if step:
                for group,probe in self.probes[index].items():
                    sample=groups[group]
                    probe.observe({'contacts':None if sample['overflow'] else sample['contacts']},{})
                self.overflow_samples[index] += int(any(g['overflow'] for g in groups.values()))
                if slip is not None:
                    self.slip_max[index]=max(self.slip_max[index] or 0.,slip)
            self.rows.append({'step':step,'env_id':index,'state_t_s':step*self.dt,
                'solver_t_s':max(0,step-1)*self.dt,'sample_phase':'last_step_solve' if step else 'reset_forward',
                'contact_force_n':all_contacts['normal_force_sum_n'],
                'hand_contact_force_n':hand['normal_force_sum_n'],
                'penetration_m':all_contacts['max_penetration_m'],'slip_m_s':slip,
                'slip_missing_reason':'unsupported_contact_velocity_in_pinned_mujoco' if slip is None else None,
                'step_time_s':step_time_s,'groups':copy.deepcopy(groups)})

    def result(self):
        interpretation = ('pre-integration solver contact diagnostics read after the step; '
                          'not physical accuracy or net force')
        return {'schema_version':'1.0','scope':'runtime contact diagnostics, not physical accuracy',
                'reset_sample_in_aggregates':False,
                'environments':[{'env_id':index,
                                 **{g:{**p.result(),'interpretation':interpretation}
                                    for g,p in probes.items()},
                                 'max_hand_tangential_speed_m_s':self.slip_max[index],
                                 'overflow_samples':self.overflow_samples[index]}
                                for index,probes in enumerate(self.probes)]}
