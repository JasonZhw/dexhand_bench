import tempfile
import unittest
from pathlib import Path
try:
    import mujoco
except ImportError:
    mujoco = None

@unittest.skipIf(mujoco is None, 'Requires MuJoCo geometry probe')
class ContactProbeTests(unittest.TestCase):
    def test_bilateral_contacts_and_forces_are_not_fabricated(self):
        from integrations.unilab.pick_contact_probe import diagnose
        xml = '''<mujoco><worldbody>
        <body name="right_link7" pos=".09 0 0"><geom type="sphere" size=".05"/></body>
        <body name="right_link8" pos="-.09 0 0"><geom type="sphere" size=".05"/></body>
        <body name="tube0"><freejoint name="tube0_free"/><geom type="sphere" size=".05" mass=".1"/></body>
        <body name="tube1" pos="3 0 0"><freejoint name="tube1_free"/><geom type="sphere" size=".05" mass=".1"/></body>
        <body name="tube2" pos="5 0 0"><freejoint name="tube2_free"/><geom type="sphere" size=".05" mass=".1"/></body>
        </worldbody></mujoco>'''
        records=[]
        for step,z in enumerate([0., 1.]):
            poses=[0,0,z,1,0,0,0, 3,0,0,1,0,0,0, 5,0,0,1,0,0,0, 0,0,0,1,0,0,0]
            records.append(dict(step=step,phase='hold',qpos=[[]],object_pose=[poses]))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scene.xml';path.write_text(xml)
            result=diagnose(path,records,[],'right','tube0')
        self.assertAlmostEqual(result['maximum_finger_penetration_m'], .01)
        self.assertEqual(result['hold_bilateral_contact_fraction'], .5)
        self.assertIsNone(result['runtime_contact_force'])
        self.assertFalse(result['runtime_contact_force_available'])
        self.assertEqual(result['samples'][0]['finger_contacts'],[True,True])
        self.assertEqual(result['samples'][1]['finger_contacts'],[False,False])

if __name__ == '__main__': unittest.main()
