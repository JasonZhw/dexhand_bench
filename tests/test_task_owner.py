import unittest

from integrations.unilab.task_owner import TASK_IDS, audit_task, bound_profile, load_owner


class TaskOwnerTests(unittest.TestCase):
    def test_all_21_task_owners_compose_and_load_their_scoring_contracts(self):
        self.assertEqual(len(TASK_IDS), 21)
        for task_id in TASK_IDS:
            with self.subTest(task=task_id):
                cfg, task = load_owner(task_id)
                self.assertEqual(cfg.task_catalog.id, task["id"])
                self.assertEqual(cfg.viewer.mode, "viser")
                self.assertTrue(task["benchmark"]["signals"])

    def test_only_five_local_grasps_claim_a_unilab_scene(self):
        bound = [task_id for task_id in TASK_IDS
                 if audit_task(task_id)["scene_binding"] == "local_grasp"]
        self.assertEqual(bound, ["G01", "G02", "G03", "G04", "G05"])
        for task_id in bound:
            with self.subTest(task=task_id):
                row = audit_task(task_id)
                self.assertTrue(row["local_scene_present"])
                self.assertTrue(row["local_scene_receipt_present"])
                cfg, _task = load_owner(task_id)
                profile = bound_profile(cfg)
                self.assertEqual(profile["task_id"], task_id)

    def test_insertion_scenes_are_diagnostics_not_task_success(self):
        for task_id, count in (("I01", 10), ("I02", 4)):
            with self.subTest(task=task_id):
                row = audit_task(task_id)
                self.assertEqual(row["scene_binding"], "unilab_scene_diagnostic")
                self.assertEqual(row["physical_load"], "smoke_verified")
                self.assertTrue(row["local_scene_present"])
                self.assertEqual(row["score_signal_count"], count)

    def test_proxy_task_cannot_use_grasp_profile(self):
        cfg, _task = load_owner("G06")
        with self.assertRaisesRegex(ValueError, "no UniLab scene binding"):
            bound_profile(cfg)


if __name__ == "__main__":
    unittest.main()
