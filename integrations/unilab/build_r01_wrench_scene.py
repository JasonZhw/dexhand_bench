"""Build a mixed-provenance R01 diagnostic: source wrench, designed nut fixture.

The RoboDojo asset is an adjustable wrench from a toolbox *transport* task.
The nut, hinge, resistance and 60-degree criterion are our diagnostic design;
this scene is not a reproduction of an official tightening task.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from pxr import Usd, UsdGeom, UsdPhysics
import coacd
import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from integrations.unilab.build_insert_wuji_scene import ROBOT, digest, resolve_robot_mesh
from integrations.unilab.build_source_asset_scenes import layout_digest, mesh_from_prim, verified_source

LAYOUTS = ROOT / "profiles/source_asset_scenes.json"
OUTPUT_ROOT = ROOT / "assets/generated/source_tasks/R01"


def build(output: Path = OUTPUT_ROOT) -> Path:
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite scene: {output}")
    layout = json.loads(LAYOUTS.read_text())["R01"]
    if layout["provenance"] != "robodojo_source_wrench_custom_fixture":
        raise ValueError("R01 provenance changed")
    spec = layout["wrench"]
    source, metadata = verified_source(layout, spec["source"])
    physics = metadata.get("physics") or {}
    if (physics.get("mass") != spec["mass_kg"]
            or physics.get("friction") != spec["friction"]):
        raise ValueError("R01 wrench mass or friction differs from pinned source metadata")
    stage = Usd.Stage.Open(str(source.resolve()))
    if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError("RoboDojo wrench USD must be Z-up and meter-scale")
    collider = stage.GetPrimAtPath(spec["collision_prim"])
    if (not collider or not collider.HasAPI(UsdPhysics.CollisionAPI)
            or UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get() is not True
            or collider.GetAttribute("physics:approximation").Get() != "convexDecomposition"):
        raise ValueError("R01 source collision is no longer an enabled convex decomposition")
    robot_meta = ROBOT.parent / "robot.json"
    robot = json.loads(robot_meta.read_text())
    if digest(ROBOT) != robot["template_sha256"]:
        raise ValueError("Wuji robot template differs from its receipt")

    root = ET.parse(ROBOT).getroot()
    root.set("model", "R01_source_wrench_custom_fixture_diagnostic")
    assets, world = root.find("asset"), root.find("worldbody")
    dependencies = {}
    for mesh in assets.findall("mesh"):
        path = resolve_robot_mesh(mesh.get("file"))
        mesh.set("file", os.path.relpath(path, output))
        dependencies[str(path.relative_to(ROOT))] = digest(path)
    output.mkdir(parents=True)
    try:
        visual = mesh_from_prim(stage, spec["visual_prim"])
        if len(visual.faces) > spec["max_visual_faces"]:
            visual = visual.simplify_quadric_decimation(face_count=spec["max_visual_faces"])
        collision = mesh_from_prim(stage, spec["collision_prim"])
        if len(collision.faces) > spec["max_collision_input_faces"]:
            collision = collision.simplify_quadric_decimation(face_count=spec["max_collision_input_faces"])
        source_bounds = mesh_from_prim(stage, spec["collision_prim"]).bounds
        if np.max(np.abs(collision.bounds - source_bounds)) > 0.001:
            raise ValueError("Wrench collision simplification changed bounds by over 1 mm")
        pieces = coacd.run_coacd(
            coacd.Mesh(collision.vertices, collision.faces),
            threshold=spec["coacd_threshold"], max_convex_hull=spec["max_convex_hulls"],
            mcts_iterations=spec["coacd_mcts_iterations"], seed=0)
        hulls = [trimesh.Trimesh(vertices, faces).convex_hull for vertices, faces in pieces]
        volumes = [float(hull.volume) for hull in hulls]
        if len(hulls) < 2 or not all(np.isfinite(volume) and volume > 0 for volume in volumes):
            raise ValueError("Wrench collision decomposition is degenerate")

        wrench = ET.SubElement(world, "body", name="wrench",
                               pos=" ".join(map(str, spec["pos"])),
                               quat=" ".join(map(str, spec["quat"])))
        ET.SubElement(wrench, "freejoint", name="wrench_free")
        visual_file = output / "wrench_visual.stl"
        visual.export(visual_file)
        dependencies[str(visual_file.relative_to(ROOT))] = digest(visual_file)
        ET.SubElement(assets, "mesh", name="wrench_visual_mesh", file=visual_file.name)
        ET.SubElement(wrench, "geom", name="wrench_visual", type="mesh",
                      mesh="wrench_visual_mesh", contype="0", conaffinity="0", mass="0",
                      group="2", rgba="0.82 0.65 0.23 1")
        for index, (hull, volume) in enumerate(zip(hulls, volumes)):
            file = output / f"wrench_contact_{index}.stl"
            hull.export(file)
            dependencies[str(file.relative_to(ROOT))] = digest(file)
            mesh_name = f"wrench_contact_mesh_{index}"
            ET.SubElement(assets, "mesh", name=mesh_name, file=file.name)
            ET.SubElement(wrench, "geom", name=f"wrench_contact_{index}", type="mesh",
                          mesh=mesh_name, mass=str(spec["mass_kg"] * volume / sum(volumes)),
                          friction=f"{spec['friction']} .005 .0001", group="3",
                          rgba="0 0 0 0")

        fixture = layout["fixture"]
        nut_spec = fixture["nut"]
        hex_file = output / "nut_hex.stl"
        trimesh.creation.cylinder(radius=nut_spec["radius_m"],
                                  height=2 * nut_spec["half_height_m"], sections=6).export(hex_file)
        dependencies[str(hex_file.relative_to(ROOT))] = digest(hex_file)
        ET.SubElement(assets, "mesh", name="nut_hex_mesh", file=hex_file.name)
        nut = ET.SubElement(world, "body", name="nut", pos=" ".join(map(str, nut_spec["pos"])))
        ET.SubElement(nut, "joint", name="nut_hinge", type="hinge", axis="0 0 1",
                      damping=str(nut_spec["joint_damping"]),
                      frictionloss=str(nut_spec["joint_frictionloss"]), limited="false")
        ET.SubElement(nut, "geom", name="nut_contact", type="mesh", mesh="nut_hex_mesh",
                      mass=str(nut_spec["mass_kg"]),
                      friction=f"{nut_spec['friction']} .005 .0001", rgba="0.54 0.57 0.62 1")
        ET.SubElement(nut, "site", name="nut_angle_marker", type="sphere",
                      pos=f"{nut_spec['radius_m'] * 0.8} 0 {nut_spec['half_height_m'] + 0.001}",
                      size="0.001", rgba="0.9 0.15 0.1 1")
        bolt_spec = fixture["bolt_visual"]
        bolt = ET.SubElement(world, "body", name="bolt", pos=" ".join(map(str, bolt_spec["pos"])))
        ET.SubElement(bolt, "geom", name="bolt_visual", type="cylinder",
                      size=f"{bolt_spec['radius_m']} {bolt_spec['half_height_m']}",
                      contype="0", conaffinity="0", mass="0", rgba="0.42 0.45 0.5 1")
        # UniLab resolves hinge columns correctly when the scalar fixture joint
        # precedes the wrench's seven-qpos free joint in the compiled MJCF.
        world.remove(nut)
        world.insert(list(world).index(wrench), nut)
        scene = output / "scene.xml"
        ET.indent(root)
        ET.ElementTree(root).write(scene, encoding="unicode")
        model = mujoco.MjModel.from_xml_path(str(scene))
        if model.nu != len(robot["joint_names"]) or model.joint("nut_hinge").type != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError("R01 compiled mechanism or robot actuator count differs")
        for name in ("wrench", "nut", "bolt"):
            model.body(name)
        receipt = {
            "task_id": "R01", "scope": "source_object_scene_diagnostic",
            "backend": "mujoco", "robot": "dual_x5_wuji",
            "scene_sha256": digest(scene), "layout_sha256": layout_digest(layout),
            "source_asset_manifest_sha256": digest(ROOT / layout["manifest"]),
            "robot_template_sha256": digest(ROBOT), "robot_metadata_sha256": digest(robot_meta),
            "source_asset_dependencies": {
                str(source.relative_to(ROOT)): digest(source),
                str(source.with_name("metadata.json").relative_to(ROOT)): digest(source.with_name("metadata.json")),
            },
            "mesh_dependencies": dependencies,
            "body_names": ["wrench", "nut", "bolt"],
            "collision_conversions": {
                "wrench": {"method": "coacd_from_source_collision_mesh", "piece_count": len(hulls),
                           "source_approximation": "convexDecomposition",
                           "source_mass_kg": spec["mass_kg"], "source_friction": spec["friction"],
                           "collision_input_faces": len(collision.faces)},
                "nut": {"origin": "self_authored_diagnostic_fixture", "method": "hex_mesh_hinge",
                        "radius_m": nut_spec["radius_m"], "mass_kg": nut_spec["mass_kg"]},
                "bolt": {"origin": "self_authored_diagnostic_fixture", "method": "visual_only"},
            },
            "nq": model.nq, "nv": model.nv, "nu": model.nu,
            "limitations": layout["limitations"],
        }
        (output / "scene.json").write_text(json.dumps(receipt, indent=2) + "\n")
        return scene
    except Exception:
        shutil.rmtree(output)
        raise
