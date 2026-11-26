import json
import tempfile
import unittest
from pathlib import Path

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]


def contact_fixture_robot(left=False):
    bodies = ['right_link7', 'right_link8', 'right_link6']
    if left:
        bodies.append('left_link7')
    return {'joint_names': [], 'body_names': bodies, 'designated_hand': 'right',
            'hands': {'right': {'palm': 'right_link6',
                               'fingers': {'index':['right_link7'], 'thumb':['right_link8']}}}}


class GraspLaunchTests(unittest.TestCase):
    def test_five_profiles_reference_existing_task_contracts(self):
        for task_id in ('G01', 'G02', 'G03', 'G04', 'G05'):
            path = ROOT / 'profiles/local_grasp' / (task_id + '.json')
            self.assertTrue(path.is_file(), f'Missing launch profile: {task_id}')
            cfg = json.loads(path.read_text())
            self.assertEqual(cfg['task_id'], task_id)
            self.assertEqual(cfg['backend'], 'mujoco')
            self.assertTrue((ROOT / cfg['task_file']).is_file())
            self.assertGreater(cfg['object']['mass_kg'], 0)
            self.assertEqual(len(cfg['object']['source_scale']), 3)


@unittest.skipIf(mujoco is None, 'Requires scene runtime')
class GraspSignalTests(unittest.TestCase):
    def test_table_penetration_is_reported_without_hand_contact(self):
        from integrations.unilab.grasp_observations import ContactGeometry
        xml = '''<mujoco><worldbody>
        <geom type="plane" size="1 1 .1"/>
        <body name="right_link7" pos="3 0 0"><geom type="sphere" size=".05"/></body>
        <body name="right_link8" pos="4 0 0"><geom type="sphere" size=".05"/></body>
        <body name="right_link6" pos="5 0 0"><geom type="sphere" size=".05"/></body>
        <body name="object"><freejoint name="object_free"/><geom type="sphere" size=".05" mass=".1"/></body>
        </worldbody></mujoco>'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'scene.xml'
            path.write_text(xml)
            points, palm, support, penetration = ContactGeometry(path, contact_fixture_robot()).update([], [0,0,.04,1,0,0,0])
        self.assertFalse(points or palm or support)
        self.assertAlmostEqual(penetration, .01)

    def test_launch_rejects_changed_external_mesh(self):
        from integrations.unilab.run_grasp_scene import validate_scene_assets
        import hashlib
        with tempfile.TemporaryDirectory() as tmp:
            scene, mesh = Path(tmp)/'scene.xml', Path(tmp)/'mesh.stl'
            scene.write_text('<mujoco/>')
            mesh.write_bytes(b'original mesh')
            receipt = {'scene_sha256': hashlib.sha256(scene.read_bytes()).hexdigest(),
                       'mesh_dependencies': {str(mesh): hashlib.sha256(mesh.read_bytes()).hexdigest()}}
            validate_scene_assets(scene, receipt)
            mesh.write_bytes(b'changed mesh')
            with self.assertRaisesRegex(ValueError, 'mesh'):
                validate_scene_assets(scene, receipt)

    def test_restricted_grasp_rejects_other_robot_support(self):
        from integrations.unilab.grasp_observations import ContactGeometry, compute_signals
        xml = '''<mujoco><worldbody>
        <body name="right_link7" pos=".09 0 .1"><geom type="sphere" size=".05"/></body>
        <body name="right_link8" pos="3 0 0"><geom type="sphere" size=".05"/></body>
        <body name="right_link6" pos="5 0 0"><geom type="sphere" size=".05"/></body>
        <body name="left_link7" pos="-.09 0 .1"><geom type="sphere" size=".05"/></body>
        <body name="object" pos="0 0 .1"><freejoint name="object_free"/><geom type="sphere" size=".05" mass=".1"/></body>
        </worldbody></mujoco>'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scene.xml';path.write_text(xml)
            probe=ContactGeometry(path, contact_fixture_robot(left=True))
            points,palm,other_support,penetration=probe.update([], [0,0,.1,1,0,0,0])
        self.assertTrue(other_support)
        self.assertTrue(probe.finger_contacts['right']['index'])
        self.assertFalse(probe.finger_contacts['right']['thumb'])
        values,_=compute_signals(pose=[0,0,.1,1,0,0,0],lin_vel=[0,0,0],ang_vel=[0,0,0],
            com_offset=[0,0,0],baseline_z=0,finger_points=points,other_hand_contact=palm,
            other_support=other_support,region={'lower':[-.1,-.1,-.1],'upper':[.1,.1,.1]},ever_lifted=False,dropped=False)
        self.assertTrue(values['hand_contact'])
        self.assertFalse(values['grasp_region_valid'])

    def test_profile_changes_require_rebuild_but_controller_changes_do_not(self):
        from integrations.unilab.run_grasp_scene import validate_scene_profile
        import copy
        cfg = json.loads((ROOT/'profiles/local_grasp/G01.json').read_text())
        receipt = {'task_id':'G01','profile':copy.deepcopy(cfg)}
        cfg['runtime']['controller'] = 'hold'
        validate_scene_profile(cfg, receipt)
        cfg['object']['mass_kg'] *= 2
        with self.assertRaisesRegex(ValueError, 'rebuild'):
            validate_scene_profile(cfg, receipt)

    def test_unsupported_backend_rejected_before_launch(self):
        from integrations.unilab.run_grasp_scene import environment_config
        cfg = json.loads((ROOT/'profiles/local_grasp/G01.json').read_text())
        cfg['backend'] = 'isaacsim'
        with self.assertRaisesRegex(ValueError, 'only mujoco'):
            environment_config(cfg)

    def test_unilab_binding_and_probe_follow_named_hand_joints(self):
        from integrations.unilab.run_grasp_scene import environment_config, probe_action
        from integrations.unilab.build_allegro_robot import load_robot
        cfg = json.loads((ROOT/'profiles/local_grasp/G01.json').read_text())
        robot = load_robot(cfg)
        env_cfg = environment_config(cfg)
        self.assertEqual(list(env_cfg.scene.entities.robot.joint_names), robot['joint_names'])
        self.assertEqual(len(env_cfg.scene.entities.robot.actuator_names), 44)
        action = probe_action(robot, 2, 1.)
        self.assertEqual(action.shape, (2, 44))
        self.assertEqual(action.dtype, np.float32)
        self.assertFalse(np.any(action[1]))
        for side in ('left','right'):
            for finger in ('ff','mf','rf','th'):
                self.assertGreater(action[0, robot['joint_names'].index(f'{side}_allegro_{finger}j1')], 0)

    def test_geometry_changes_cannot_reuse_old_prepared_mesh(self):
        from integrations.unilab.build_grasp_scene import build
        if not (ROOT/'assets/generated/grasp_objects/G01/geometry.json').is_file():
            self.skipTest('Prepare geometry first')
        cfg = json.loads((ROOT/'profiles/local_grasp/G01.json').read_text())
        cfg['object']['source_scale'] = [2,2,2]
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, 'prepare'):
                build(cfg, Path(tmp)/'stale')

    def test_contact_region_rejects_shaft_and_palm_support(self):
        from integrations.unilab.grasp_observations import compute_signals
        kwargs = dict(pose=[0, 0, .1, 1, 0, 0, 0], lin_vel=[0, 0, 0],
                      ang_vel=[0, 0, 0], com_offset=[0, 0, 0], baseline_z=0,
                      region={'lower': [-.1, -.1, -.1], 'upper': [.1, .1, .1]},
                      ever_lifted=False, dropped=False)
        result, _ = compute_signals(**kwargs, finger_points=[[0, 0, .1]], other_hand_contact=False)
        self.assertTrue(result['grasp_region_valid'])
        result, _ = compute_signals(**kwargs, finger_points=[[.2, 0, .1]], other_hand_contact=False)
        self.assertFalse(result['grasp_region_valid'])
        result, _ = compute_signals(**kwargs, finger_points=[[0, 0, .1]], other_hand_contact=True)
        self.assertFalse(result['grasp_region_valid'])
        result, _ = compute_signals(**kwargs, finger_points=[], other_hand_contact=False)
        self.assertFalse(result['hand_contact'])
        self.assertFalse(result['grasp_region_valid'])

    def test_com_velocity_drop_latch_and_upright(self):
        from integrations.unilab.grasp_observations import compute_signals
        kwargs = dict(pose=[0, 0, .01, 1, 0, 0, 0], lin_vel=[0, 0, 0],
                      ang_vel=[0, 0, 2], com_offset=[.1, 0, 0], baseline_z=0,
                      region=None, finger_points=[], other_hand_contact=False)
        result, history = compute_signals(**kwargs, ever_lifted=True, dropped=False)
        self.assertAlmostEqual(result['speed_m_s'], .2)
        self.assertTrue(result['dropped'])
        self.assertEqual(history, (True, True))
        self.assertAlmostEqual(result['tilt_deg'], 0)
        kwargs['pose'] = [0, 0, .2, 2**-.5, 2**-.5, 0, 0]
        result, _ = compute_signals(**kwargs, ever_lifted=True, dropped=True)
        self.assertTrue(result['dropped'])
        self.assertAlmostEqual(result['tilt_deg'], 90)

    def test_five_models_compile_with_independent_dynamic_objects(self):
        from integrations.unilab.build_grasp_scene import build
        if not (ROOT / 'assets/generated/grasp_objects/G05/geometry.json').is_file():
            self.skipTest('Prepare official grasp object geometry first')
        with tempfile.TemporaryDirectory() as temp:
            for task_id in ('G01', 'G02', 'G03', 'G04', 'G05'):
                cfg = json.loads((ROOT / 'profiles/local_grasp' / (task_id + '.json')).read_text())
                path = build(cfg, Path(temp) / task_id)
                model = mujoco.MjModel.from_xml_path(str(path))
                self.assertEqual(model.nu, 44)
                self.assertEqual(model.nq, 51)
                self.assertEqual(model.joint('object_free').qposadr[0], 44)
                self.assertAlmostEqual(model.body('object').mass[0], cfg['object']['mass_kg'], places=6)
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                obj = model.body('object').id
                table = model.geom('diagnostic_table').id
                penetration = [c.dist for c in data.contact
                               if table in (c.geom1, c.geom2)
                               and obj in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2])]
                self.assertFalse(any(d < -1e-6 for d in penetration))
                self.assertFalse(any(c.dist < -1e-6 and obj in
                                     (model.geom_bodyid[c.geom1],model.geom_bodyid[c.geom2])
                                     for c in data.contact), 'Initial object must not intersect any robot link')
                data.ctrl[:] = model.qpos0[:44]
                for _ in range(50):
                    mujoco.mj_step(model, data)
                self.assertTrue(np.isfinite(data.qpos).all())
                self.assertGreater(data.body('object').xpos[2], .7)


if __name__ == '__main__':
    unittest.main()
