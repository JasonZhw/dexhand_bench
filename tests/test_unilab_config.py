import unittest
from pathlib import Path

from hydra import compose, initialize_config_dir

ROOT = Path(__file__).resolve().parents[1]


class UniLabConfigTest(unittest.TestCase):
    def compose(self, owner):
        with initialize_config_dir(version_base=None, config_dir=str(ROOT / 'conf/unilab')):
            return compose(config_name='benchmark', overrides=[f'task=insert_tubes/{owner}'])

    def test_backends_share_task_protocol(self):
        mujoco = self.compose('mujoco')
        isaac = self.compose('isaacsim')
        self.assertEqual(mujoco.task.protocol, isaac.task.protocol)
        self.assertEqual(mujoco.task.backend.name, 'mujoco')
        self.assertEqual(isaac.task.backend.name, 'isaacsim')
        self.assertNotEqual(mujoco.task.backend.asset_route, isaac.task.backend.asset_route)

    def test_no_false_runtime_support_claim(self):
        for owner in ('mujoco', 'isaacsim'):
            cfg = self.compose(owner)
            self.assertFalse(cfg.task.runtime.ready)
            self.assertIsNone(cfg.task.runtime.env_factory)
            self.assertTrue(cfg.task.runtime.blockers)
            self.assertEqual(cfg.measurement.missing_capability, 'unavailable')

    def test_protocol_references_exist(self):
        cfg = self.compose('mujoco')
        for field in ('declaration', 'layout', 'assets'):
            self.assertTrue((ROOT / cfg.task.protocol[field]).is_file())


if __name__ == '__main__':
    unittest.main()
