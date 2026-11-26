"""Convert the pinned Bench2Dex faucet into a UniLab MuJoCo diagnostic.

Run with .venv-assets so USD is available. This ports the two-handle mechanism,
not the original multi-stage task, PhysX contacts, or its demonstration policy.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from pxr import Gf, Usd, UsdGeom, UsdPhysics
import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from dexbench.assets import verify_asset
from integrations.unilab.build_insert_wuji_scene import ROBOT, digest, resolve_robot_mesh
from integrations.unilab.build_source_asset_scenes import layout_digest

LAYOUTS = ROOT / "profiles/source_asset_scenes.json"
OUTPUT_ROOT = ROOT / "assets/generated/source_tasks/R02"


def _source_meshes(stage, link, cache):
    root = link.GetChild("collisions")
    if not root or not root.IsInstance():
        raise ValueError(f"Missing instanced source collision subtree: {link.GetPath()}")
    meshes = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        transform = cache.GetLocalToWorldTransform(prim)
        vertices = np.asarray([tuple(transform.Transform(point)) for point in mesh.GetPointsAttr().Get()])
        indices = list(mesh.GetFaceVertexIndicesAttr().Get())
        faces, offset = [], 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            polygon = indices[offset:offset + count]
            faces.extend((polygon[0], polygon[i], polygon[i + 1]) for i in range(1, count - 1))
            offset += count
        if offset != len(indices) or not faces:
            raise ValueError(f"Invalid source mesh topology: {prim.GetPath()}")
        meshes.append((str(prim.GetPath()), trimesh.Trimesh(vertices, faces, process=True)))
    if not meshes:
        raise ValueError(f"No source collision meshes in {link.GetPath()}")
    return meshes


def _joint_axis(joint, parent, child, cache):
    physics = UsdPhysics.RevoluteJoint(joint)
    if ([str(path) for path in physics.GetBody0Rel().GetTargets()] != [str(parent.GetPath())]
            or [str(path) for path in physics.GetBody1Rel().GetTargets()] != [str(child.GetPath())]
            or physics.GetAxisAttr().Get() != "Y"
            or physics.GetCollisionEnabledAttr().Get() is not False):
        raise ValueError(f"Unexpected faucet articulation: {joint.GetPath()}")
    lower, upper = physics.GetLowerLimitAttr().Get(), physics.GetUpperLimitAttr().Get()
    if not (math.isclose(lower, 0, abs_tol=1e-7) and math.isclose(upper, 90, abs_tol=1e-6)):
        raise ValueError(f"Unexpected source joint limits: {joint.GetPath()}")
    direction = cache.GetLocalToWorldTransform(parent).TransformDir(Gf.Vec3d(0, 1, 0))
    axis = np.asarray(tuple(direction), dtype=float)
    axis /= np.linalg.norm(axis)
    return axis


def build(output: Path = OUTPUT_ROOT) -> Path:
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite scene: {output}")
    layout = json.loads(LAYOUTS.read_text())["R02"]
    if layout["provenance"] != "bench2dex_source_faucet_adapted_contact":
        raise ValueError("R02 source provenance changed")
    manifest_path = ROOT / layout["manifest"]
    manifest = json.loads(manifest_path.read_text())
    faucet = manifest["assets"]["a094_faucet"]
    actuator = faucet["spawn"]["actuators"]["handles"]
    if (faucet["scale"] != [layout["source_scale"]] * 3
            or faucet["spawn"]["mass_props"]["density"] != layout["source_density_kg_m3"]
            or actuator["joint_names_expr"] != ["joint_0", "joint_1"]
            or actuator["damping"] != layout["joint_damping_from_scene"]):
        raise ValueError("R02 conversion differs from the pinned Bench2Dex scene settings")
    if any(verify_asset(entry, ROOT)["status"] != "verified" for entry in manifest["files"]):
        raise ValueError("Pinned Bench2Dex faucet files are missing or changed")
    source = ROOT / layout["source_usd"]
    stage = Usd.Stage.Open(str(source.resolve()))
    if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError("Faucet USD must be Z-up and measured in meters")
    robot_meta = ROBOT.parent / "robot.json"
    robot = json.loads(robot_meta.read_text())
    if digest(ROBOT) != robot["template_sha256"]:
        raise ValueError("Wuji robot template differs from its receipt")

    root = ET.parse(ROBOT).getroot()
    root.set("model", "R02_bench2dex_faucet_diagnostic")
    assets, world = root.find("asset"), root.find("worldbody")
    dependencies = {}
    for mesh in assets.findall("mesh"):
        path = resolve_robot_mesh(mesh.get("file"))
        mesh.set("file", os.path.relpath(path, output))
        dependencies[str(path.relative_to(ROOT))] = digest(path)

    scale = layout["source_scale"]
    density = layout["source_density_kg_m3"]
    yaw = math.radians(layout["base_yaw_deg"])
    base = ET.SubElement(world, "body", name="faucet_base",
                         pos=" ".join(map(str, layout["base_position_m"])),
                         quat=f"{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}")
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    # Both source USD revolute joints set physics:collisionEnabled=False.
    for handle in ("faucet_handle_0", "faucet_handle_1"):
        ET.SubElement(contact, "exclude", body1="faucet_base", body2=handle)
    source_root = stage.GetDefaultPrim()
    if not source_root or not source_root.IsValid():
        raise ValueError("Source faucet USD has no default prim")
    cache = UsdGeom.XformCache()
    source_fixed = source_root.GetChild("link_2")
    conversion = {}
    output.mkdir(parents=True)
    try:
        for spec in layout["bodies"]:
            source_link = source_root.GetChild(spec["source_link"])
            if not source_link or not source_link.IsValid():
                raise ValueError(f"Source link missing: {spec['source_link']}")
            origin = np.asarray(tuple(cache.GetLocalToWorldTransform(source_link).ExtractTranslation()))
            if spec["name"] == "faucet_base":
                body = base
                if not np.allclose(origin, 0, atol=1e-7):
                    raise ValueError("Unexpected fixed faucet link origin")
            else:
                body = ET.SubElement(base, "body", name=spec["name"],
                                     pos=" ".join(map(str, origin * scale)))
                source_joint = source_root.GetChild("joints").GetChild(spec["joint_name"])
                if not source_joint or not source_joint.IsValid():
                    raise ValueError(f"Source joint missing: {spec['joint_name']}")
                axis = _joint_axis(source_joint, source_fixed, source_link, cache)
                ET.SubElement(body, "joint", name=spec["joint_name"], type="hinge",
                              axis=" ".join(map(str, axis)), range=f"0 {math.pi/2}",
                              damping=str(layout["joint_damping_from_scene"]), limited="true")
            parts = _source_meshes(stage, source_link, cache)
            volume = 0.0
            for index, (source_path, source_mesh) in enumerate(parts):
                relative_vertices = (source_mesh.vertices - origin) * scale
                local = trimesh.Trimesh(relative_vertices, source_mesh.faces, process=True)
                hull = local.convex_hull
                if not np.isfinite(hull.volume) or hull.volume <= 0:
                    raise ValueError(f"Degenerate faucet collision mesh: {source_path}")
                volume += hull.volume
                name = f"{spec['name']}_{index}"
                visual_file = output / f"{name}_visual.stl"
                contact_file = output / f"{name}_contact.stl"
                local.export(visual_file)
                hull.export(contact_file)
                for file, suffix in ((visual_file, "visual"), (contact_file, "contact")):
                    dependencies[str(file.relative_to(ROOT))] = digest(file)
                    ET.SubElement(assets, "mesh", name=f"{name}_{suffix}", file=file.name)
                ET.SubElement(body, "geom", name=f"{name}_visual", type="mesh",
                              mesh=f"{name}_visual", contype="0", conaffinity="0", mass="0",
                              rgba="0.52 0.58 0.63 1" if spec["name"] == "faucet_base"
                              else "0.84 0.56 0.16 1")
                ET.SubElement(body, "geom", name=f"{name}_contact", type="mesh",
                              mesh=f"{name}_contact", group="3", rgba="0 0 0 0",
                              mass=str(hull.volume * density), friction="0.8 0.005 0.0001")
            conversion[spec["name"]] = {
                "source_link": spec["source_link"], "source_meshes": [path for path, _ in parts],
                "method": "per_source_mesh_convex_hull", "mesh_count": len(parts),
                "scaled_hull_volume_m3": volume,
                "derived_mass_kg": volume * density,
                "mass_origin": "source_scene_density_and_adapted_hulls",
            }
        scene = output / "scene.xml"
        ET.indent(root)
        ET.ElementTree(root).write(scene, encoding="unicode")
        model = mujoco.MjModel.from_xml_path(str(scene))
        if model.nu != len(robot["joint_names"]):
            raise ValueError("Faucet scene changed robot actuator count")
        for joint_name in ("joint_0", "joint_1"):
            if not np.allclose(model.joint(joint_name).range, [0, math.pi/2], atol=1e-6):
                raise ValueError(f"Converted joint limits differ: {joint_name}")
        for spec in layout["bodies"]:
            model.body(spec["name"])
        receipt = {
            "task_id": "R02", "scope": "source_object_scene_diagnostic",
            "backend": "mujoco", "robot": "dual_x5_wuji",
            "scene_sha256": digest(scene), "layout_sha256": layout_digest(layout),
            "source_asset_manifest_sha256": digest(manifest_path),
            "source_asset_dependencies": {entry["local_path"]: digest(ROOT / entry["local_path"])
                                          for entry in manifest["files"]},
            "robot_template_sha256": digest(ROBOT),
            "robot_metadata_sha256": digest(robot_meta),
            "mesh_dependencies": dependencies,
            "body_names": [spec["name"] for spec in layout["bodies"]],
            "collision_conversions": conversion,
            "nq": model.nq, "nv": model.nv, "nu": model.nu,
            "limitations": layout["limitations"],
        }
        (output / "scene.json").write_text(json.dumps(receipt, indent=2) + "\n")
        return scene
    except Exception:
        shutil.rmtree(output)
        raise


if __name__ == "__main__":
    print(build())
