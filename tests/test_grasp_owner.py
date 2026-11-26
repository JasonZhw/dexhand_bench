import unittest
from pathlib import Path

from hydra import compose, initialize_config_dir

from integrations.unilab.grasp_profiles import load_profile
from integrations.unilab.run_grasp_scene import ROOT, environment_config


class GraspOwnerTests(unittest.TestCase):
    def test_g02_launch_owner_resolves_to_the_existing_unilab_environment(self):
        with initialize_config_dir(version_base=None, config_dir=str(ROOT/'conf/unilab')):
            launch = compose(config_name='grasp_g02_wuji')
        self.assertEqual((launch.task_id, launch.robot, launch.controller),
                         ('G02', 'wuji', 'ik_grasp'))
        self.assertEqual(launch.viewer.mode, 'viser')
        self.assertTrue((ROOT/Path(launch.controller_config)).is_file())
        profile = load_profile('G02', 'wuji')
        profile['runtime']['controller'] = 'ik_grasp'
        cfg = environment_config(profile, 'grasp_g02_wuji')
        self.assertEqual(Path(cfg.scene.model_file), (ROOT/profile['scene']).resolve())
        self.assertEqual(cfg.sim_dt, profile['runtime']['sim_dt'])
        self.assertEqual(cfg.ctrl_dt, profile['runtime']['ctrl_dt'])


if __name__ == '__main__':
    unittest.main()
