import tempfile
import unittest
from pathlib import Path

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, 'Requires MuJoCo scene environment')
class RuntimePhysicsTests(unittest.TestCase):
    def setUp(self):
        from integrations.unilab import runtime_physics
        self.physics = runtime_physics
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.xml = '''<mujoco><option timestep=".002" integrator="implicitfast"/>
        <worldbody><geom name="diagnostic_table" type="plane" size="1 1 .1"/>
        <body name="right_hand" pos="3 0 0"><geom type="sphere" size=".05"/></body>
        <body name="left_hand" pos="4 0 0"><geom type="sphere" size=".05"/></body>
        <body name="object" pos="0 0 .1"><freejoint name="object_free"/>
        <geom type="sphere" size=".05" mass=".2"/></body></worldbody></mujoco>'''
        self.robot = {'hands': {side: {'palm': side+'_hand'} for side in ('right','left')},
                      'designated_hand':'right'}

    def scene(self, xml=None, slots=16):
        root = Path(self.temp.name)
        (root/'source.xml').write_text(xml or self.xml)
        manifest = self.physics.instrument_scene(root/'source.xml', root/'instrumented.xml', self.robot, slots)
        return mujoco.MjModel.from_xml_path(str(root/'instrumented.xml')), manifest

    def sensor_sample(self, model, data, manifest):
        return self.physics.decode_contacts({group: data.sensor(item['name']).data.copy()
                                            for group,item in manifest['groups'].items()}, manifest)

    def test_runtime_support_force_and_no_hand_slip(self):
        model, manifest = self.scene()
        data = mujoco.MjData(model)
        for _ in range(1000):
            mujoco.mj_step(model,data)
        before = (data.qpos.copy(), data.qvel.copy(), data.qacc_warmstart.copy())
        sample = self.sensor_sample(model,data,manifest)
        self.assertAlmostEqual(sample['object_all']['normal_force_sum_n'], .2*9.81, delta=.01)
        self.assertAlmostEqual(sample['table']['normal_force_sum_n'], .2*9.81, delta=.01)
        self.assertEqual(sample['right_hand']['normal_force_sum_n'],0.)
        self.assertIsNone(sample['right_hand']['max_tangential_speed_m_s'])
        for current,original in zip((data.qpos,data.qvel,data.qacc_warmstart),before):
            np.testing.assert_array_equal(current,original)

    def test_freefall_is_zero_force_but_not_zero_slip(self):
        model,manifest = self.scene()
        data = mujoco.MjData(model)
        mujoco.mj_step(model,data)
        sample = self.sensor_sample(model,data,manifest)
        self.assertEqual(sample['object_all']['matched_contacts'],0)
        self.assertEqual(sample['object_all']['normal_force_sum_n'],0.)
        self.assertIsNone(sample['object_all']['max_tangential_speed_m_s'])

    def test_overflow_does_not_underreport_force_or_penetration(self):
        xml = self.xml.replace('<geom type="sphere" size=".05" mass=".2"/>',
                              '<geom type="box" size=".05 .05 .05" mass=".2"/>')
        model,manifest = self.scene(xml,slots=1)
        data=mujoco.MjData(model)
        for _ in range(200):mujoco.mj_step(model,data)
        sample=self.sensor_sample(model,data,manifest)['table']
        self.assertGreater(sample['matched_contacts'],1)
        self.assertTrue(sample['overflow'])
        self.assertIsNone(sample['normal_force_sum_n'])
        self.assertIsNone(sample['max_penetration_m'])
        self.assertEqual(len(sample['contacts']),1)

    def test_unsupported_contact_speed_remains_missing_even_when_moving(self):
        model,manifest=self.scene()
        data=mujoco.MjData(model)
        data.qpos[2]=.049
        data.qvel[:3]=[.3,.4,0.]
        mujoco.mj_forward(model,data)
        sample=self.sensor_sample(model,data,manifest)['table']
        self.assertGreater(sample['matched_contacts'],0)
        self.assertIsNone(sample['max_tangential_speed_m_s'])

    def test_hand_contact_is_not_table_support(self):
        model,manifest=self.scene(self.xml.replace('pos="3 0 0"','pos=".09 0 .1"'))
        data=mujoco.MjData(model)
        mujoco.mj_step(model,data)
        sample=self.sensor_sample(model,data,manifest)
        self.assertGreater(sample['right_hand']['normal_force_sum_n'],0)
        self.assertEqual(sample['table']['normal_force_sum_n'],0)
        self.assertEqual(sample['left_hand']['normal_force_sum_n'],0)

    def test_recorder_runs_existing_probe_and_marks_solver_time(self):
        model,manifest=self.scene()
        data=mujoco.MjData(model)
        for _ in range(500):mujoco.mj_step(model,data)
        sample=self.sensor_sample(model,data,manifest)
        recorder=self.physics.PhysicsRecorder(num_envs=2, designated_hand='right',dt=.002)
        recorder.record(1,[sample,sample],.001)
        summary=recorder.result()
        self.assertAlmostEqual(summary['environments'][0]['object_all']['max_normal_force_sum_N'],1.962,delta=.01)
        self.assertEqual(summary['environments'][1]['object_all']['samples_available'],1)
        self.assertIn('pre-integration solver',summary['environments'][0]['object_all']['interpretation'])
        self.assertEqual(recorder.rows[0]['solver_t_s'],0.)
        self.assertEqual(recorder.rows[0]['state_t_s'],.002)
        self.assertIsNone(recorder.rows[0]['slip_m_s'])

    def test_instrumentation_preserves_dynamics(self):
        instrumented,manifest=self.scene()
        original=mujoco.MjModel.from_xml_string(self.xml)
        a,b=mujoco.MjData(original),mujoco.MjData(instrumented)
        for _ in range(200):
            mujoco.mj_step(original,a)
            mujoco.mj_step(instrumented,b)
        np.testing.assert_array_equal(a.qpos,b.qpos)
        np.testing.assert_array_equal(a.qvel,b.qvel)

    def test_unilab_recording_matches_disabled_run_and_reads_live_sensor(self):
        import copy
        import json
        from integrations.unilab.grasp_profiles import load_profile
        from integrations.unilab.run_grasp_scene import run, ROOT
        profile=load_profile('G01','wuji')
        if not (ROOT/profile['scene']).exists():
            self.skipTest('Requires prepared Wuji apple assets')
        profile['runtime']['steps']=60
        profile['physics_recording']={'enabled':True,'contact_slots':32,'warmup_steps':10}
        enabled=Path(self.temp.name)/'enabled'
        receipt=run(profile,enabled)
        self.assertTrue(receipt['physics_force_available'])
        rows=json.loads((enabled/'runtime-physics.json').read_text())
        self.assertEqual(len(rows),122)
        self.assertTrue(any(r['groups']['table']['normal_force_sum_n'] > 1 for r in rows))
        self.assertTrue(all(r['hand_contact_force_n']==0 for r in rows))
        self.assertTrue(all(r['slip_m_s'] is None for r in rows))
        public_rows=json.loads((enabled/'physics.json').read_text())
        self.assertTrue(any(r['contact_force_n'] is not None and r['contact_force_n'] > 1 for r in public_rows))
        self.assertTrue(all('offline_geometry_penetration_m' in r for r in public_rows))
        disabled=Path(self.temp.name)/'disabled'
        off=copy.deepcopy(profile)
        off['physics_recording']['enabled']=False
        run(off,disabled)
        a=json.loads((enabled/'states.json').read_text())
        b=json.loads((disabled/'states.json').read_text())
        self.assertEqual(a,b, 'Observation must not perturb actions or state evolution')
        self.assertEqual(json.loads((enabled/'evaluation.json').read_text()),
                         json.loads((disabled/'evaluation.json').read_text()))


if __name__ == '__main__':
    unittest.main()
