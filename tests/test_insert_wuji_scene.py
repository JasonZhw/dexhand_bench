import json
import math
from pathlib import Path
import tempfile
import unittest

import torch
from hydra.utils import instantiate

from integrations.unilab.build_insert_wuji_scene import build
from integrations.unilab.play_task import InsertionDiagnosticPlaybackSession
from integrations.unilab.run_insert_wuji_scene import environment_config, home_predicate, run
from integrations.unilab.task_assets import fetch, source_asset_entries


class InsertWujiSceneTests(unittest.TestCase):
    def test_builder_compiles_wuji_and_all_four_i01_objects(self):
        with tempfile.TemporaryDirectory() as directory:
            scene = build(Path(directory) / "scene")
            receipt = json.loads((scene.parent / "scene.json").read_text())
            self.assertEqual(receipt["task_id"], "I01")
            self.assertEqual(receipt["nu"], 52)
            self.assertGreater(receipt["nq"], receipt["nu"])
            self.assertGreater(len(receipt["mesh_dependencies"]), 100)

    def test_home_uses_per_axis_position_and_quaternion_angle(self):
        origin_pos = torch.zeros((1, 2, 3))
        origin_quat = torch.tensor([[[1., 0., 0., 0.], [1., 0., 0., 0.]]])
        pos = origin_pos.clone()
        pos[0, 0, 0] = .14
        pos[0, 0, 1] = .14
        quat = origin_quat.clone()
        theta = math.radians(19) / 2
        quat[0, 0] = torch.tensor([math.cos(theta), math.sin(theta), 0, 0])
        self.assertTrue(bool(home_predicate(pos, quat, origin_pos, origin_quat)[0]))
        pos[0, 0, 0] = .16
        self.assertFalse(bool(home_predicate(pos, quat, origin_pos, origin_quat)[0]))
        pos[0, 0, 0] = 0
        theta = math.radians(21) / 2
        quat[0, 0] = torch.tensor([math.cos(theta), math.sin(theta), 0, 0])
        self.assertFalse(bool(home_predicate(pos, quat, origin_pos, origin_quat)[0]))

    def test_asset_fetch_budget_is_checked_before_any_download(self):
        _manifest, entries = source_asset_entries("I02")
        self.assertEqual(len(entries), 5)
        with self.assertRaisesRegex(ValueError, "budget"):
            fetch("M02", max_total_bytes=0)

    def test_both_insertion_diagnostics_produce_complete_signal_rows(self):
        for task_id, count in (("I01", 10), ("I02", 4)):
            with self.subTest(task=task_id), tempfile.TemporaryDirectory() as directory:
                profile = json.loads((Path(__file__).resolve().parents[1] /
                                      "profiles" / f"{task_id}-wuji.json").read_text())
                output = Path(directory) / "run"
                receipt = run(profile, output, steps=2)
                scores = json.loads((output / "evaluation.json").read_text())
                self.assertEqual(receipt["score_signal_count"], count)
                self.assertIsNone(receipt["task_success"])
                self.assertTrue(receipt["independent_reset_verified"])
                self.assertEqual([score["missing_signals"] for score in scores], [[], []])
                self.assertEqual([score["failure_reason"] for score in scores],
                                 ["incomplete_trace", "incomplete_trace"])

    def test_i02_native_viser_session_advances_the_unilab_environment(self):
        from unilab.envs import make_manager_based_rl_env

        profile = json.loads((Path(__file__).resolve().parents[1] /
                              "profiles/I02-wuji.json").read_text())
        env_cfg, robot = environment_config(profile)
        env = make_manager_based_rl_env(instantiate(env_cfg, _convert_="all"),
                                        num_envs=2, backend_type="mujoco")

        class Controls:
            def consume_step_permission(self):
                return True

        try:
            session = InsertionDiagnosticPlaybackSession(env, profile, robot)
            session.reset()
            self.assertTrue(session.advance(Controls()))
            self.assertEqual(session.step_count, 1)
            self.assertIsNotNone(session.physics_state())
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
