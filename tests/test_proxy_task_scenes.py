"""The remaining task owners must launch honest scene diagnostics."""

import json
import hashlib
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from integrations.unilab.task_owner import ROOT, audit_task, load_owner


PROXY_IDS = (
    "G06", "H01", "H02", "H03",
    "P02", "R03",
)
SOURCE_IDS = ("I03", "M01", "M02", "M03", "P01", "P03", "R01", "R02")


class ProxyTaskSceneTests(unittest.TestCase):
    def test_adroit_door_subtree_keeps_nested_joints_and_inertias(self):
        from integrations.unilab.adapt_adroit_door import prepare_door_subtree

        source = b'''<mujoco><worldbody><body name="frame" pos="0 0 0.4">
          <inertial mass="7" diaginertia="0.9 0.8 0.2" pos="0 0 0"/>
          <geom type="cylinder" size="0.05 0.25" material="wood"/>
          <body name="door" pos="0.3 0 0">
            <inertial mass="2" diaginertia="0.09 0.05 0.04" pos="0 0 0"/>
            <joint name="door_hinge" pos="0.3 0 0" axis="0 0 1"
                   range="0 1.57" damping="1" frictionloss="2"/>
            <geom type="box" size="0.2 0.05 0.25" friction="1 1 1" material="wood"/>
            <body name="latch" pos="-0.15 0 -0.025">
              <inertial mass="3" diaginertia="0.04 0.03 0.01" pos="0 0 0"/>
              <joint name="latch" pos="0 0 0" axis="0 1 0"
                     range="0 1.8" frictionloss="5"/>
              <geom type="capsule" size="0.02 0.1" material="foil"/>
            </body>
          </body>
        </body></worldbody></mujoco>'''
        defaults = b'''<mujocoinclude><option iterations="20" noslip_iterations="20"/>
          <default class="main"><geom friction="1 0.5 0.01" margin="0.0005"/>
            <joint limited="true" margin="0.01" armature="0.001"
                   damping="0.05" frictionloss="0.001"/></default>
        </mujocoinclude>'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "assets").mkdir()
            (root / "assets/door.xml").write_bytes(source)
            (root / "assets/defaults.xml").write_bytes(defaults)
            spec = {"file": "assets/door.xml", "defaults_file": "assets/defaults.xml",
                    "subtree_body": "frame", "frame_translation": [-0.2, -0.2, 0.6],
                    "default_friction": [1, 0.5, 0.01], "default_margin": 0.0005,
                    "noslip_iterations": 20}
            layout = {"bodies": [{"name": "door"}, {"name": "latch"}]}
            frame, audit = prepare_door_subtree(spec, layout, root)
            self.assertEqual(frame.get("pos"), "-0.2 -0.2 1.0")
            door = frame.find("./body[@name='door']")
            latch = door.find("./body[@name='latch']")
            self.assertEqual(door.find("joint").get("frictionloss"), "2")
            self.assertEqual(latch.find("joint").get("frictionloss"), "5")
            self.assertEqual(latch.find("joint").get("damping"), "0.05")
            self.assertEqual(latch.find("joint").get("armature"), "0.001")
            self.assertEqual(door.find("inertial").get("mass"), "2")
            self.assertEqual(door.find("geom").get("friction"), "1 1 1")
            self.assertEqual(latch.find("geom").get("friction"), "1 0.5 0.01")
            self.assertEqual(latch.find("geom").get("margin"), "0.0005")
            self.assertIsNone(latch.find("geom").get("material"))
            self.assertEqual(audit["joint_names"], ["door_hinge", "latch"])
            (root / "assets/door.xml").write_bytes(source.replace(b'frictionloss="5"', b'frictionloss="0"'))
            with self.assertRaisesRegex(ValueError, "latch frictionloss"):
                prepare_door_subtree(spec, layout, root)

    def test_shadow_egg_source_geometry_and_goal_are_checked(self):
        from integrations.unilab.build_proxy_task_scenes import verify_source_mjcf

        source = b'''<mujoco><option timestep="0.002" iterations="20"/><worldbody>
          <body name="object"><joint type="free" damping="0.01"/>
            <geom name="object" type="ellipsoid" size="0.03 0.03 0.04" condim="4"/>
            <geom name="object_hidden" type="ellipsoid" size="0.029 0.029 0.03"
                  mass="0" contype="0" conaffinity="0"/>
          </body>
          <body name="target"><geom name="target" type="ellipsoid" size="0.03 0.03 0.04"
                  contype="0" conaffinity="0"/></body>
        </worldbody></mujoco>'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "assets").mkdir()
            asset = root / "assets/egg.xml"
            asset.write_bytes(source)
            (root / "manifest.json").write_text(json.dumps({"files": [
                {"local_path": "assets/egg.xml", "sha256": hashlib.sha256(source).hexdigest()}
            ]}))
            layout = {
                "source_mjcf": {"manifest": "manifest.json", "file": "assets/egg.xml",
                                "object_body": "object", "target_body": "egg",
                                "goal_body": "orientation_goal", "source_goal_body": "target",
                                "ignored_source_geoms": ["object_hidden"],
                                "timestep": 0.002, "iterations": 20,
                                "default_density": 1000,
                                "default_friction": [1, 0.005, 0.0001]},
                "bodies": [
                    {"name": "egg", "pos": [0, 0, 0.8], "joint_damping": 0.01,
                     "geoms": [{"type": "ellipsoid", "size": [0.03, 0.03, 0.04],
                                "mass": 4 * math.pi / 3 * 0.03 * 0.03 * 0.04 * 1000,
                                "friction": [1, 0.005, 0.0001], "condim": 4}]},
                    {"name": "orientation_goal", "pos": [0, 0, 0.8], "geoms": [
                        {"type": "ellipsoid", "size": [0.03, 0.03, 0.04]}]},
                ],
            }
            result = verify_source_mjcf(layout, root)
            self.assertEqual(result["source_object_mass_kg"], layout["bodies"][0]["geoms"][0]["mass"])
            self.assertEqual(result["ignored_visual_geoms"], ["object_hidden"])
            layout["bodies"][1]["pos"][0] = 0.1
            with self.assertRaisesRegex(ValueError, "goal position"):
                verify_source_mjcf(layout, root)
            layout["bodies"][1]["pos"][0] = 0
            asset.write_bytes(source.replace(b'conaffinity="0"/>', b'conaffinity="1"/>', 1))
            manifest = json.loads((root / "manifest.json").read_text())
            manifest["files"][0]["sha256"] = hashlib.sha256(asset.read_bytes()).hexdigest()
            (root / "manifest.json").write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "ignored"):
                verify_source_mjcf(layout, root)

    def test_adroit_pen_components_and_defaults_match_layout(self):
        from integrations.unilab.build_proxy_task_scenes import verify_source_mjcf

        source = b'''<mujoco><worldbody>
          <site name="eps_ball" pos="0 -0.2 0.25"/>
          <body name="Object" pos="0 -0.2 0.25">
            <joint name="x" type="slide" damping="0"/>
            <joint name="y" type="slide" damping="0"/>
            <joint name="z" type="slide" damping="0"/>
            <joint name="rx" type="hinge" damping="0"/>
            <joint name="ry" type="hinge" damping="0"/>
            <joint name="rz" type="hinge" damping="0"/>
            <geom name="pen" type="cylinder" size="0.015 0.065" density="1500" condim="4"/>
            <geom name="cli" type="box" size="0.004 0.006 0.03" pos="-0.015 0 -0.0255"
                  contype="0" conaffinity="0"/>
          </body>
        </worldbody></mujoco>'''
        defaults = b'''<mujocoinclude><option iterations="20"/>
          <default class="main"><geom friction="1 0.5 0.01" margin="0.0005"/></default>
        </mujocoinclude>'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "assets").mkdir()
            entries = []
            for name, data in (("pen.xml", source), ("defaults.xml", defaults)):
                path = root / "assets" / name
                path.write_bytes(data)
                entries.append({"local_path": f"assets/{name}",
                                "sha256": hashlib.sha256(data).hexdigest(), "size_bytes": len(data)})
            (root / "manifest.json").write_text(json.dumps({"files": entries}))
            layout = {
                "source_mjcf": {"manifest": "manifest.json", "file": "assets/pen.xml",
                                "defaults_file": "assets/defaults.xml", "object_body": "Object",
                                "target_body": "pen", "goal_body": "target_pen",
                                "object_geom_names": ["pen", "cli"],
                                "joint_count": 6, "timestep": 0.002, "iterations": 20,
                                "default_density": 1000, "default_friction": [1, 0.5, 0.01],
                                "default_margin": 0.0005},
                "bodies": [
                    {"name": "pen", "pos": [0, 0, 0.84], "joint_damping": 0,
                     "geoms": [
                         {"source_geom": "pen", "type": "cylinder", "size": [0.015, 0.065],
                          "mass": math.pi * 0.015**2 * 0.13 * 1500,
                          "friction": [1, 0.5, 0.01], "condim": 4, "margin": 0.0005},
                         {"source_geom": "cli", "type": "box", "size": [0.004, 0.006, 0.03],
                          "pos": [-0.015, 0, -0.0255], "mass": 0.00576,
                          "friction": [1, 0.5, 0.01], "condim": 3, "margin": 0.0005,
                          "contype": 0, "conaffinity": 0}]},
                    {"name": "target_pen", "pos": [0, 0, 0.84], "geoms": []},
                ],
            }
            result = verify_source_mjcf(layout, root)
            self.assertEqual(result["source_geom_count"], 2)
            layout["bodies"][0]["geoms"][0]["margin"] = 0
            with self.assertRaisesRegex(ValueError, "margin"):
                verify_source_mjcf(layout, root)
            layout["bodies"][0]["geoms"][0]["margin"] = 0.0005
            layout["bodies"][0]["geoms"][1]["contype"] = 1
            with self.assertRaisesRegex(ValueError, "contype"):
                verify_source_mjcf(layout, root)
            layout["bodies"][0]["geoms"][1]["contype"] = 0
            layout["bodies"][1]["pos"][0] = 0.1
            with self.assertRaisesRegex(ValueError, "goal position"):
                verify_source_mjcf(layout, root)
            layout["bodies"][1]["pos"][0] = 0
            layout["source_mjcf"]["default_density"] = 500
            with self.assertRaisesRegex(ValueError, "default density"):
                verify_source_mjcf(layout, root)
            layout["source_mjcf"]["default_density"] = 1000
            alternate = root / "assets/unpinned.xml"
            alternate.write_bytes(defaults)
            layout["source_mjcf"]["defaults_file"] = "assets/unpinned.xml"
            with self.assertRaisesRegex(ValueError, "not pinned"):
                verify_source_mjcf(layout, root)

    def test_h02_goal_pose_is_recorded_at_pen_target_point(self):
        layouts = json.loads((ROOT / "profiles/diagnostic_proxy_scenes.json").read_text())
        pen, goal = layouts["H02"]["bodies"]
        self.assertEqual(goal["pos"], pen["pos"])
        self.assertEqual(len(pen["geoms"]), 4)
        self.assertAlmostEqual(sum(geom["mass"] for geom in pen["geoms"]),
                               0.1820379053855769, places=12)
        records = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/H02/states.json").read_text())
        index = records[0]["body_names"].index("target_pen")
        for actual, expected in zip(records[0]["object_quat_wxyz"][index], goal["quat"]):
            self.assertAlmostEqual(actual, expected, places=5)
        receipt = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/H02/receipt.json").read_text())
        self.assertIsNone(receipt["task_success"])
        self.assertEqual(receipt["score_signals_available"], 0)

    def test_h03_egg_compiled_inertia_and_diagnostic_goal(self):
        import mujoco

        layouts = json.loads((ROOT / "profiles/diagnostic_proxy_scenes.json").read_text())
        egg, goal = layouts["H03"]["bodies"]
        self.assertEqual(goal["pos"], egg["pos"])
        scene = ROOT / "assets/generated/task_proxies/H03/scene.xml"
        if scene.is_file():
            model = mujoco.MjModel.from_xml_path(str(scene))
            body = model.body("rigid_egg")
            self.assertAlmostEqual(model.body_mass[body.id], 0.15079644737231007, places=12)
            for actual, expected in zip(model.body_inertia[body.id],
                                        (7.539822368615503e-05,
                                         7.539822368615503e-05,
                                         5.428672105403162e-05)):
                self.assertAlmostEqual(actual, expected, places=12)
        records = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/H03/states.json").read_text())
        index = records[0]["body_names"].index("orientation_goal")
        for actual, expected in zip(records[0]["object_quat_wxyz"][index], goal["quat"]):
            self.assertAlmostEqual(actual, expected, places=5)
        receipt = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/H03/receipt.json").read_text())
        self.assertIsNone(receipt["task_success"])
        self.assertEqual(receipt["score_signals_available"], 0)

    def test_r03_door_hinge_signal_is_recorded_without_claiming_success(self):
        records = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/R03/states.json").read_text())
        traces = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/R03/traces.json").read_text())
        receipt = json.loads((ROOT / "docs/evidence/source-mjcf-20261126/R03/receipt.json").read_text())
        self.assertEqual(receipt["score_signals_available"], 1)
        self.assertIsNone(receipt["task_success"])
        self.assertTrue(all(not trace["complete"] for trace in traces))
        self.assertTrue(all(frame["signals"]["door_hinge_rad"] is not None
                            for trace in traces for frame in trace["frames"]))
        self.assertTrue(all(record["door_hinge_rad"] is not None for record in records))

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

        for task_id, ctrl_dt in (("H01", 0.04), ("H02", 0.01), ("H03", 0.04),
                                 ("P02", 0.04), ("R03", 0.01)):
            with self.subTest(task=task_id):
                profile = diagnostic_profile(task_id)
                self.assertEqual(profile["sim_dt"], 0.002)
                self.assertEqual(profile["ctrl_dt"], ctrl_dt)

    def test_remaining_owners_have_explicit_proxy_bindings(self):
        for task_id in PROXY_IDS:
            with self.subTest(task=task_id):
                cfg, task = load_owner(task_id)
                self.assertEqual(cfg.task_catalog.binding.kind, "unilab_proxy_diagnostic")
                self.assertEqual(cfg.task_catalog.binding.backend, "mujoco")
                self.assertEqual(cfg.task_catalog.binding.robot, "wuji")
                row = audit_task(task_id)
                self.assertEqual(row["score_signals_available"], 1 if task_id == "R03" else 0)
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
                self.assertEqual(receipt["score_signals_available"], 1 if task_id == "R03" else 0)

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
                self.assertEqual(layouts[task_id]["provenance"],
                                 "bench2dex_source_faucet_adapted_contact" if task_id == "R02"
                                 else "robodojo_source_wrench_custom_fixture" if task_id == "R01"
                                 else "robodojo_source_mesh_adapted_contact")
                row = audit_task(task_id)
                self.assertEqual(row["score_signals_available"], 1 if task_id in ("R01", "R02") else 0)
                self.assertEqual(row["score_signal_count"], len(task["benchmark"]["signals"]))
                if row["local_scene_present"]:
                    self.assertEqual(row["scene_integrity"], "verified")
                    self.assertEqual(row["source_assets_verified"], row["source_assets_declared"])
                    scene, receipt = validate_scene(task_id)
                    self.assertTrue(scene.is_file())
                    self.assertEqual(receipt["scope"], "source_object_scene_diagnostic")
                    self.assertEqual(set(receipt["body_names"]),
                                     {"wrench", "nut", "bolt"} if task_id == "R01" else
                                     {body["name"] for body in layouts[task_id]["bodies"]})
                    self.assertGreater(len(receipt["source_asset_dependencies"]), 0)
                    expected = ({"faucet_base", "faucet_handle_0", "faucet_handle_1"} if task_id == "R02"
                                else {"wrench", "nut", "bolt"} if task_id == "R01"
                                else {"t"} if task_id == "P01" else {"key", "slot"} if task_id == "I03"
                                else {"car", "electric_toothbrush", "hammer", "shoe", "box"} if task_id == "M01"
                                else {"block_0", "block_1", "block_2"} if task_id == "M03"
                                else {"cube_0", "cube_1", "cube_2", "broom_shovel", "broom"} if task_id == "P03"
                                else {"pliers", "hammer", "tape_measure", "wrench", "toolbox"})
                    self.assertEqual(set(receipt["collision_conversions"]), expected)
                    if task_id == "R01":
                        mujoco = __import__("mujoco")
                        model = mujoco.MjModel.from_xml_path(str(scene))
                        self.assertEqual(model.joint("nut_hinge").type,
                                         mujoco.mjtJoint.mjJNT_HINGE)
                        self.assertLess(model.joint("nut_hinge").id,
                                        model.joint("wrench_free").id)
                        self.assertEqual(receipt["collision_conversions"]["wrench"]["method"],
                                         "coacd_from_source_collision_mesh")
                        self.assertEqual(receipt["collision_conversions"]["nut"]["origin"],
                                         "self_authored_diagnostic_fixture")
                        self.assertGreater(receipt["collision_conversions"]["wrench"]["piece_count"], 1)
                        state = mujoco.MjData(model)
                        mujoco.mj_forward(model, state)
                        self.assertFalse(any(
                            {model.geom(contact.geom1).name, model.geom(contact.geom2).name}
                            == {"diagnostic_table", "nut_contact"}
                            for contact in state.contact))
                        dof = int(model.joint("nut_hinge").dofadr[0])
                        qpos = int(model.joint("nut_hinge").qposadr[0])
                        for _ in range(round(0.5 / model.opt.timestep)):
                            state.qfrc_applied[dof] = 0.04
                            mujoco.mj_step(model, state)
                        self.assertGreater(state.qpos[qpos], 0.2)
                        continue
                    if task_id == "R02":
                        mujoco = __import__("mujoco")
                        model = mujoco.MjModel.from_xml_path(str(scene))
                        self.assertEqual(model.joint("joint_0").range.tolist(), [0.0, math.pi / 2])
                        self.assertEqual(model.joint("joint_1").range.tolist(), [0.0, math.pi / 2])
                        self.assertEqual(receipt["collision_conversions"]["faucet_handle_0"]["source_link"],
                                         "link_0")
                        data = mujoco.MjData(model)
                        mujoco.mj_forward(model, data)
                        contact_pairs = []
                        for i in range(data.ncon):
                            pair = {model.body(int(model.geom_bodyid[geom])).name
                                    for geom in (data.contact[i].geom1, data.contact[i].geom2)}
                            contact_pairs.append(pair)
                        self.assertFalse(any("faucet_base" in pair and
                                             pair & {"faucet_handle_0", "faucet_handle_1"}
                                             for pair in contact_pairs))
                        continue
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

    def test_r02_records_source_joint_without_claiming_task_success(self):
        directory = ROOT / "docs/evidence/source-task-assets-20261126/R02"
        traces = json.loads((directory / "traces.json").read_text())
        receipt = json.loads((directory / "receipt.json").read_text())
        self.assertEqual(receipt["score_signals_available"], 1)
        self.assertIsNone(receipt["task_success"])
        self.assertTrue(all(not trace["complete"] and all(
            frame["signals"]["faucet_joint_rad"] is not None for frame in trace["frames"])
            for trace in traces))

    def test_r01_records_custom_nut_hinge_without_claiming_tightening_success(self):
        directory = ROOT / "docs/evidence/source-task-assets-20261126/R01"
        traces = json.loads((directory / "traces.json").read_text())
        receipt = json.loads((directory / "receipt.json").read_text())
        self.assertEqual(receipt["score_signals_available"], 1)
        self.assertIsNone(receipt["task_success"])
        self.assertTrue(all(not trace["complete"] and all(
            frame["signals"]["nut_rotation_rad"] is not None
            and frame["signals"]["wrench_engaged"] is None
            for frame in trace["frames"]) for trace in traces))

    def test_r01_source_jaws_reject_configured_ten_mm_nut(self):
        from integrations.unilab.check_r01_wrench_fit import analyze_fit

        mesh_root = ROOT / "assets/generated/source_tasks/R01"
        if not (mesh_root / "scene.xml").is_file():
            self.skipTest("R01 generated meshes are not present in this checkout")
        report = analyze_fit(mesh_root)
        measurements = report["results"]
        configured = measurements[0]
        smaller = measurements[-1]
        self.assertFalse(configured["within_0_5mm_penetration_limit"])
        self.assertGreater(configured["best_two_jaw_pose"]["max_penetration_m"], 0.002)
        self.assertTrue(smaller["within_0_5mm_penetration_limit"])
        self.assertIsNone(report["task_success"])


if __name__ == "__main__":
    unittest.main()
