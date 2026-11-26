"""The remaining task owners must launch honest scene diagnostics."""

import json
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from integrations.unilab.task_owner import ROOT, audit_task, load_owner


PROXY_IDS = (
    "G06", "H01", "H02", "H03",
    "P02", "R01", "R02", "R03",
)
SOURCE_IDS = ("I03", "M01", "M02", "M03", "P01", "P03")


class ProxyTaskSceneTests(unittest.TestCase):
    def test_source_block_density_and_contact_dimensions_match_layout(self):
        from integrations.unilab.build_proxy_task_scenes import verify_source_mjcf

        source = b'''<mujoco><option timestep="0.002" iterations="20"/><worldbody>
          <body name="object"><joint type="free" damping="0.01"/>
            <geom name="object" type="box" size="0.025 0.025 0.025"
                  density="567" condim="4"/></body>
        </worldbody></mujoco>'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            asset = root / "assets/block.xml"
            asset.parent.mkdir()
            asset.write_bytes(source)
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"files": [{"local_path": "assets/block.xml",
                                                    "url": "https://example.invalid/block.xml",
                                                    "sha256": hashlib.sha256(source).hexdigest(),
                                                    "size_bytes": len(source)}]}))
            layout = {
                "source_mjcf": {"manifest": "manifest.json", "file": "assets/block.xml",
                                "object_body": "object", "target_body": "block",
                                "timestep": 0.002, "iterations": 20,
                                "default_friction": [1, 0.005, 0.0001]},
                "bodies": [{"name": "block", "joint_damping": 0.01,
                            "geoms": [{"type": "box", "size": [0.025, 0.025, 0.025],
                                       "mass": 0.070875, "friction": [1, 0.005, 0.0001],
                                       "condim": 4}]}],
            }
            self.assertEqual(verify_source_mjcf(layout, root)["source_file"], "assets/block.xml")
            layout["bodies"][0]["geoms"][0]["condim"] = 3
            with self.assertRaisesRegex(ValueError, "condim"):
                verify_source_mjcf(layout, root)
            layout["bodies"][0]["geoms"][0]["condim"] = 4
            layout["bodies"][0]["geoms"][0]["mass"] = 0.08
            with self.assertRaisesRegex(ValueError, "mass"):
                verify_source_mjcf(layout, root)
            layout["bodies"][0]["geoms"][0]["mass"] = 0.070875
            layout["source_mjcf"]["default_friction"][0] = 0.5
            with self.assertRaisesRegex(ValueError, "default friction"):
                verify_source_mjcf(layout, root)
            layout["source_mjcf"]["default_friction"][0] = 1
            changed = source.replace(b' condim="4"', b'')
            asset.write_bytes(changed)
            manifest.write_text(json.dumps({"files": [{"local_path": "assets/block.xml",
                                                    "url": "https://example.invalid/block.xml",
                                                    "sha256": hashlib.sha256(changed).hexdigest(),
                                                    "size_bytes": len(changed)}]}))
            with self.assertRaisesRegex(ValueError, "condim"):
                verify_source_mjcf(layout, root)

    def test_h01_orientation_goal_is_recorded_as_body_pose(self):
        layouts = json.loads((ROOT / "profiles/diagnostic_proxy_scenes.json").read_text())
        goal = next(body for body in layouts["H01"]["bodies"]
                    if body["name"] == "orientation_goal")
        self.assertIn("quat", goal)
        self.assertNotIn("quat", goal["geoms"][0])
        scene = ROOT / "assets/generated/task_proxies/H01/scene.xml"
        if scene.is_file():
            body = next(body for body in ET.parse(scene).getroot().find("worldbody").findall("body")
                        if body.get("name") == "orientation_goal")
            self.assertEqual([float(value) for value in body.get("quat", "").split()], goal["quat"])
        records = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/H01/states.json").read_text())
        index = records[0]["body_names"].index("orientation_goal")
        for actual, expected in zip(records[0]["object_quat_wxyz"][index], goal["quat"]):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_source_parameterized_proxy_rejects_puck_and_table_drift(self):
        from integrations.unilab import build_proxy_task_scenes
        self.assertTrue(hasattr(build_proxy_task_scenes, "verify_source_mjcf"))
        verify_source_mjcf = build_proxy_task_scenes.verify_source_mjcf

        source = b'''<mujoco><option timestep="0.002"/><worldbody>
          <body name="table0"><geom name="table0" type="box" size="0.625 0.45 0.2"
              friction="0.1 0.005 0.0001"/></body>
          <body name="object0"><joint type="free" damping="0.01"/><geom name="object0" type="cylinder" size="0.025 0.02"
              mass="2" friction="0.1 0.005 0.0001" condim="3"/></body>
        </worldbody></mujoco>'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            asset = root / "assets/slide.xml"
            asset.parent.mkdir()
            asset.write_bytes(source)
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"files": [{"local_path": "assets/slide.xml",
                                                    "url": "https://example.invalid/slide.xml",
                                                    "sha256": hashlib.sha256(source).hexdigest(),
                                                    "size_bytes": len(source)}]}))
            layout = {
                "source_mjcf": {"manifest": "manifest.json", "file": "assets/slide.xml",
                                "object_body": "object0", "table_body": "table0", "timestep": 0.002},
                "source_table": {"size_xy": [0.625, 0.45], "friction": [0.1, 0.005, 0.0001]},
                "bodies": [{"name": "object", "joint_damping": 0.01,
                            "geoms": [{"type": "cylinder", "size": [0.025, 0.02],
                                                           "mass": 2, "friction": [0.1, 0.005, 0.0001],
                                                           "condim": 3}]}],
            }
            self.assertEqual(verify_source_mjcf(layout, root)["source_file"], "assets/slide.xml")
            layout["bodies"][0]["geoms"][0]["mass"] = 1
            with self.assertRaisesRegex(ValueError, "object mass"):
                verify_source_mjcf(layout, root)
            layout["bodies"][0]["geoms"][0]["mass"] = 2
            layout["bodies"][0]["joint_damping"] = 0.02
            with self.assertRaisesRegex(ValueError, "joint damping"):
                verify_source_mjcf(layout, root)
            layout["bodies"][0]["joint_damping"] = 0.01
            layout["source_table"]["friction"][0] = 0.8
            with self.assertRaisesRegex(ValueError, "table friction"):
                verify_source_mjcf(layout, root)

    def test_farama_diagnostics_use_source_timestep_and_control_rate(self):
        from integrations.unilab.run_proxy_task_scene import diagnostic_profile

        for task_id in ("H01", "P02"):
            with self.subTest(task=task_id):
                profile = diagnostic_profile(task_id)
                self.assertEqual(profile["sim_dt"], 0.002)
                self.assertEqual(profile["ctrl_dt"], 0.04)

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
                                else {"car", "electric_toothbrush", "hammer", "shoe", "box"} if task_id == "M01"
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
                    if task_id == "M01":
                        self.assertEqual(receipt["collision_conversions"]["box"]["method"],
                                         "source_bbox_open_tray")
                        self.assertEqual(receipt["collision_conversions"]["hammer"]["mass_origin"],
                                         "diagnostic_assumption")
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
