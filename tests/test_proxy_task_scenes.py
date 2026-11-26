"""The remaining task owners must launch honest scene diagnostics."""

import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from integrations.unilab.task_owner import ROOT, audit_task, load_owner


PROXY_IDS = (
    "G06", "H01", "H02", "H03", "I03", "M01", "M02", "M03",
    "P01", "P02", "P03", "R01", "R02", "R03",
)


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


if __name__ == "__main__":
    unittest.main()
