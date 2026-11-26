"""The remaining task owners must launch honest scene diagnostics."""

import json
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from integrations.unilab.task_owner import ROOT, audit_task, load_owner


PROXY_IDS = (
    "G06", "H01", "H02", "H03", "M01",
    "P02", "R01", "R02", "R03",
)
SOURCE_IDS = ("I03", "M02", "M03", "P01", "P03")


class ProxyTaskSceneTests(unittest.TestCase):
    def test_remaining_owners_have_explicit_proxy_bindings(self):
        for task_id in PROXY_IDS:
            with self.subTest(task=task_id):
                cfg, task = load_owner(task_id)
                self.assertEqual(cfg.task_catalog.binding.kind, "unilab_proxy_diagnostic")
                self.assertEqual(cfg.task_catalog.binding.backend, "mujoco")
                self.assertEqual(cfg.task_catalog.binding.robot, "wuji")
                row = audit_task(task_id)
                self.assertEqual(row["score_signals_available"], 0)
                self.assertEqual(row["score_signal_count"], len(task["benchmark"]["signals"]))
                scene = ROOT / "assets/generated/task_proxies" / task_id / "scene.xml"
                if not scene.is_file():
                    self.assertFalse(row["local_scene_present"])
                    self.assertEqual(row["physical_load"], "not_attempted")
                    continue
                self.assertTrue(row["local_scene_present"])
                self.assertTrue(row["local_scene_receipt_present"])
                self.assertEqual(row["physical_load"], "smoke_verified")
                root = ET.parse(scene).getroot()
                self.assertTrue(all(not Path(mesh.get("file")).is_absolute()
                                    for mesh in root.find("asset").findall("mesh")))
                receipt = json.loads((ROOT / str(cfg.task_catalog.evidence)).read_text())
                self.assertIsNone(receipt["task_success"])
                self.assertEqual(receipt["score_signals_available"], 0)

    def test_proxy_layouts_do_not_claim_source_physics(self):
        layouts = json.loads((ROOT / "profiles/diagnostic_proxy_scenes.json").read_text())
        self.assertEqual(set(layouts), set(PROXY_IDS))
        for task_id, layout in layouts.items():
            with self.subTest(task=task_id):
                self.assertEqual(layout["provenance"], "self_authored_geometry_proxy")
                self.assertTrue(layout["bodies"])
                names = [body["name"] for body in layout["bodies"]]
                self.assertEqual(len(names), len(set(names)))

    def test_source_object_tasks_have_distinct_bindings_and_receipts(self):
        from integrations.unilab.run_proxy_task_scene import validate_scene

        layouts = json.loads((ROOT / "profiles/source_asset_scenes.json").read_text())
        self.assertEqual(set(layouts), set(SOURCE_IDS))
        for task_id in SOURCE_IDS:
            with self.subTest(task=task_id):
                cfg, task = load_owner(task_id)
                self.assertEqual(cfg.task_catalog.binding.kind, "unilab_source_asset_diagnostic")
                self.assertEqual(cfg.task_catalog.binding.backend, "mujoco")
                self.assertEqual(cfg.task_catalog.binding.robot, "wuji")
                self.assertEqual(str(cfg.task_catalog.binding.profile), "profiles/source_asset_scenes.json")
                self.assertEqual(layouts[task_id]["provenance"], "robodojo_source_mesh_adapted_contact")
                row = audit_task(task_id)
                self.assertEqual(row["score_signals_available"], 0)
                self.assertEqual(row["score_signal_count"], len(task["benchmark"]["signals"]))
                if row["local_scene_present"]:
                    self.assertEqual(row["scene_integrity"], "verified")
                    self.assertEqual(row["source_assets_verified"], row["source_assets_declared"])
                    scene, receipt = validate_scene(task_id)
                    self.assertTrue(scene.is_file())
                    self.assertEqual(receipt["scope"], "source_object_scene_diagnostic")
                    self.assertEqual(set(receipt["body_names"]),
                                     {body["name"] for body in layouts[task_id]["bodies"]})
                    self.assertGreater(len(receipt["source_asset_dependencies"]), 0)
                    expected = ({"t"} if task_id == "P01" else {"key", "slot"} if task_id == "I03"
                                else {"block_0", "block_1", "block_2"} if task_id == "M03"
                                else {"cube_0", "cube_1", "cube_2", "broom_shovel", "broom"} if task_id == "P03"
                                else {"pliers", "hammer", "tape_measure", "wrench", "toolbox"})
                    self.assertEqual(set(receipt["collision_conversions"]), expected)
                    if task_id == "M03":
                        self.assertEqual({entry["method"] for entry in receipt["collision_conversions"].values()},
                                         {"source_bbox_proxy"})
                        self.assertEqual({entry["source_approximation"] for entry in receipt["collision_conversions"].values()},
                                         {"boundingCube"})
                        self.assertEqual({entry["mass_origin"] for entry in receipt["collision_conversions"].values()},
                                         {"source_metadata"})
                    if task_id == "M02":
                        self.assertEqual(receipt["collision_conversions"]["toolbox"]["method"],
                                         "source_bbox_open_tray")
                        self.assertEqual(receipt["collision_conversions"]["pliers"]["mass_origin"],
                                         "diagnostic_assumption")
                        self.assertLessEqual(max(item["render_faces"] for item in
                                                 receipt["visual_conversions"].values()), 30000)
                    if task_id == "P03":
                        self.assertEqual(receipt["collision_conversions"]["broom_shovel"]["method"],
                                         "source_bbox_open_tray")
                        self.assertEqual(receipt["collision_conversions"]["broom"]["method"],
                                         "source_bbox_proxy")
                        self.assertLessEqual(max(item["render_faces"] for item in
                                                 receipt["visual_conversions"].values()), 20000)

    def test_changed_source_scene_cannot_reuse_old_smoke_receipt(self):
        scene = ROOT / "assets/generated/source_tasks/P01/scene.xml"
        if not scene.is_file():
            self.skipTest("P01 source scene is not built in this checkout")
        with patch("integrations.unilab.run_proxy_task_scene.validate_scene",
                   side_effect=ValueError("changed source asset")):
            row = audit_task("P01")
        self.assertEqual(row["scene_integrity"], "invalid")
        self.assertEqual(row["physical_load"], "not_attempted")


if __name__ == "__main__":
    unittest.main()
