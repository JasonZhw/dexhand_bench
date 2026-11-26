"""Physical integration checks: mappings, FK, control independence and task reuse."""
import json
from pathlib import Path
import tempfile
import unittest
try:
    import numpy as np
    import mujoco
except ImportError:
    mujoco = None

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipIf(mujoco is None, 'Requires MuJoCo scene runtime')
class VendorHandTests(unittest.TestCase):
    def setUp(self):
        if not (ROOT/'assets/vendor/wuji/hand2/hand2_2/body/mjcf/left.xml').exists():
            self.skipTest('Fetch pinned vendor hand assets first')

    def test_vendor_robots_preserve_source_mechanics_and_remove_grippers(self):
        from integrations.unilab.build_vendor_robot import build
        for kind, count in [('wuji',20),('sharpa',22)]:
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as tmp:
                path=build(kind,Path(tmp)/'robot')
                model=mujoco.MjModel.from_xml_path(str(path))
                metadata=json.loads((path.parent/'robot.json').read_text())
                self.assertEqual(model.nu,12+2*count)
                self.assertEqual(model.nq,model.nu)
                np.testing.assert_array_equal(model.actuator_trnid[:,0],np.arange(model.nu))
                data=mujoco.MjData(model);data.ctrl[:]=model.qpos0;mujoco.mj_forward(model,data)
                self.assertFalse(any(c.dist < -1e-5 for c in data.contact),'Unexpected initial robot collision')
                for side in ['left','right']:
                    self.assertEqual(len(metadata['hand_joint_names'][side]),count)
                    self.assertEqual(len(metadata['hands'][side]['fingers']),5)
                    self.assertNotIn(f'{side}_joint7',metadata['joint_names'])
                    cfg=metadata['configuration'];prefix=f'{side}_{kind}_'
                    source=mujoco.MjModel.from_xml_path(str(ROOT/cfg['source_models'][side]))
                    srcdata=mujoco.MjData(source)
                    for i in range(source.njnt):
                        name=source.joint(i).name;dest=model.joint(prefix+name).id
                        np.testing.assert_allclose(model.jnt_range[dest],source.jnt_range[i])
                        np.testing.assert_allclose(model.dof_armature[dest],source.dof_armature[i])
                        value=float(np.clip(.1,*source.jnt_range[i]))
                        data.qpos[dest]=srcdata.qpos[i]=value
                    for i in range(source.nu):
                        dest=model.actuator(prefix+source.actuator(i).name).id
                        np.testing.assert_allclose(model.actuator_gainprm[dest],source.actuator_gainprm[i])
                        np.testing.assert_allclose(model.actuator_biasprm[dest],source.actuator_biasprm[i],atol=1e-9)
                    mujoco.mj_forward(model,data);mujoco.mj_forward(source,srcdata)
                    source_root=cfg['hands'][side]['root'];a=srcdata.body(source_root);b=data.body(prefix+source_root)
                    for i in range(1,source.nbody):
                        name=source.body(i).name;dest=model.body(prefix+name)
                        np.testing.assert_allclose(dest.mass,source.body(i).mass)
                        np.testing.assert_allclose(dest.inertia,source.body(i).inertia)
                        expected=(srcdata.body(name).xpos-a.xpos)@a.xmat.reshape(3,3)
                        actual=(data.body(prefix+name).xpos-b.xpos)@b.xmat.reshape(3,3)
                        np.testing.assert_allclose(actual,expected,atol=1e-7)
                mujoco.mj_resetData(model,data);data.ctrl[:]=model.qpos0
                for _ in range(250):mujoco.mj_step(model,data)
                self.assertTrue(np.isfinite(data.qpos).all())
                np.testing.assert_allclose(data.qpos,model.qpos0,atol=.01)

    def test_selected_robot_keeps_task_criteria_and_changes_scene_and_action_mapping(self):
        from integrations.unilab.grasp_profiles import load_profile
        from integrations.unilab.build_vendor_robot import build
        from integrations.unilab.run_grasp_scene import probe_action
        for kind,count in [('wuji',52),('sharpa',56)]:
            for tid in ['G01','G02','G03','G04','G05']:
                p=load_profile(tid,kind);base=load_profile(tid,'allegro')
                self.assertEqual(p['task_file'],base['task_file'])
                self.assertEqual(p['object'],base['object'])
                self.assertIn(kind,p['scene'])
            with tempfile.TemporaryDirectory() as tmp:
                path=build(kind,Path(tmp)/'robot');r=json.loads((path.parent/'robot.json').read_text())
                model=mujoco.MjModel.from_xml_path(str(path))
                action=probe_action(r,2,1)
                self.assertEqual(action.shape,(2,count));self.assertEqual(action.dtype,np.float32)
                self.assertTrue(np.all(action[1]==0))
                targets=action[0]+model.qpos0
                self.assertTrue(np.all(targets>=model.jnt_range[:,0]))
                self.assertTrue(np.all(targets<=model.jnt_range[:,1]))
                for hand in r['hands'].values():
                    flat=[b for group in hand['fingers'].values() for b in group]
                    self.assertEqual(len(flat),len(set(flat)))
                    self.assertNotIn(hand['palm'],flat)
                    for bodies in hand['fingers'].values():
                        ids=[i for i in range(model.njnt) if model.body(model.jnt_bodyid[i]).name in bodies]
                        self.assertTrue(any(action[0,i] != 0 for i in ids))
