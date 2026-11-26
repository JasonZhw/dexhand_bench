"""Convert pinned RoboDojo object meshes into adapted UniLab diagnostics.

Run with .venv-assets, which contains USD and CoACD. These scenes use the
original object geometry but neither the source reset nor the source solver.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

# USD must load before CoACD in this environment.
from pxr import Usd, UsdGeom, UsdPhysics
import coacd
import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from dexbench.assets import verify_asset
from integrations.unilab.build_insert_wuji_scene import ROBOT, digest, resolve_robot_mesh

LAYOUTS = ROOT / "profiles/source_asset_scenes.json"
OUTPUT_ROOT = ROOT / "assets/generated/source_tasks"


def mesh_from_prim(stage, path: str) -> trimesh.Trimesh:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsA(UsdGeom.Mesh):
        raise ValueError(f"USD mesh missing: {path}")
    mesh = UsdGeom.Mesh(prim)
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    vertices = np.asarray([tuple(transform.Transform(point)) for point in mesh.GetPointsAttr().Get()])
    indices = list(mesh.GetFaceVertexIndicesAttr().Get())
    faces, offset = [], 0
    for count in mesh.GetFaceVertexCountsAttr().Get():
        polygon = indices[offset:offset + count]
        for index in range(1, count - 1):
            faces.append((polygon[0], polygon[index], polygon[index + 1]))
        offset += count
    if offset != len(indices) or not faces:
        raise ValueError(f"Invalid USD mesh topology: {path}")
    return trimesh.Trimesh(vertices, faces, process=True)


def layout_digest(layout: dict) -> str:
    canonical = json.dumps(layout, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(canonical).hexdigest()


def verified_source(layout: dict, source: str) -> tuple[Path, dict]:
    manifest = json.loads((ROOT / layout["manifest"]).read_text())
    expected = f"assets/robodojo/Assets/Object/RoboDojo/{source}/"
    entries = [entry for entry in manifest["files"] if entry["local_path"].startswith(expected)]
    if not entries or any(verify_asset(entry, ROOT)["status"] != "verified" for entry in entries):
        raise ValueError(f"Source asset is missing or changed: {source}")
    path = ROOT / expected / "object.usdz"
    metadata = json.loads((ROOT / expected / "metadata.json").read_text())
    return path, metadata


def build(task_id: str, output: Path | None = None) -> Path:
    layouts = json.loads(LAYOUTS.read_text())
    if task_id not in layouts:
        raise ValueError(f"No source-object layout for {task_id}")
    layout = layouts[task_id]
    if layout["provenance"] != "robodojo_source_mesh_adapted_contact":
        raise ValueError("Source-object provenance missing")
    output = (output or OUTPUT_ROOT / task_id).resolve()
    if output.exists():
        raise FileExistsError(output)
    robot_meta = ROOT / "assets/generated/dual_x5_wuji/robot.json"
    robot = json.loads(robot_meta.read_text())
    if digest(ROBOT) != robot["template_sha256"]:
        raise ValueError("Wuji robot template differs from its receipt")
    root = ET.parse(ROBOT).getroot()
    root.set("model", f"{task_id}_source_object_diagnostic")
    assets, world = root.find("asset"), root.find("worldbody")
    dependencies = {}
    for mesh in assets.findall("mesh"):
        path = resolve_robot_mesh(mesh.get("file"))
        mesh.set("file", os.path.relpath(path, output))
        dependencies[str(path.relative_to(ROOT))] = digest(path)
    source_dependencies = {}
    body_names = []
    collision_conversions = {}
    output.mkdir(parents=True)
    try:
        for spec in layout["bodies"]:
            name = spec["name"]
            body_names.append(name)
            source, metadata = verified_source(layout, spec["source"])
            source_dependencies[str(source.relative_to(ROOT))] = digest(source)
            metadata_path = source.parent / "metadata.json"
            source_dependencies[str(metadata_path.relative_to(ROOT))] = digest(metadata_path)
            stage = Usd.Stage.Open(str(source.resolve()))
            if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1:
                raise ValueError(f"Expected Z-up meter-scale USD: {source}")
            visual = mesh_from_prim(stage, spec["visual_prim"])
            visual_file = output / f"{name}_visual.stl"
            visual.export(visual_file)
            dependencies[str(visual_file.relative_to(ROOT))] = digest(visual_file)
            ET.SubElement(assets, "mesh", name=f"{name}_visual_mesh", file=visual_file.name)
            body = ET.SubElement(world, "body", name=name,
                                 pos=" ".join(map(str, spec["pos"])))
            if spec["motion"] == "free":
                ET.SubElement(body, "freejoint", name=f"{name}_free")
            elif spec["motion"] != "fixed":
                raise ValueError(f"Unsupported source object motion: {spec['motion']}")
            ET.SubElement(body, "geom", name=f"{name}_source_visual", type="mesh",
                          mesh=f"{name}_visual_mesh", contype="0", conaffinity="0",
                          group="2", mass="0", rgba="0.85 0.65 0.22 1")
            if spec.get("visual_only"):
                continue
            if abs(float(metadata["physics"]["mass"]) - spec.get("mass_kg", 0.0)) > 1e-6:
                raise ValueError(f"Mass differs from pinned metadata: {name}")
            method = spec["collision"]["method"]
            collision_conversions[name] = {
                "method": method,
                "source_friction": metadata["physics"].get("friction"),
                "configured_friction": spec["friction"],
            }
            if method == "two_boxes_from_source_dimensions":
                authored = stage.GetPrimAtPath("/World/collision/model")
                if UsdPhysics.CollisionAPI(authored).GetCollisionEnabledAttr().Get() is not False:
                    raise ValueError("P01 collision source flag changed")
                boxes = spec["collision"]["boxes"]
                volume = sum(8 * np.prod(box["size"]) for box in boxes)
                if abs(volume - abs(visual.volume)) > 1e-7:
                    raise ValueError("P01 contact boxes no longer match the T mesh volume")
                for index, box in enumerate(boxes):
                    mass = spec["mass_kg"] * 8 * np.prod(box["size"]) / volume
                    ET.SubElement(body, "geom", name=f"{name}_contact_{index}", type="box",
                                  pos=" ".join(map(str, box["pos"])),
                                  size=" ".join(map(str, box["size"])),
                                  mass=str(mass), friction=f"{spec['friction']} .005 .0001",
                                  rgba="0.85 0.65 0.22 0")
            elif method == "coacd":
                collision_prim = stage.GetPrimAtPath(spec["collision_prim"])
                if not collision_prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(collision_prim).GetCollisionEnabledAttr().Get() is False:
                    raise ValueError(f"Source collision mesh disabled: {name}")
                mesh = mesh_from_prim(stage, spec["collision_prim"])
                settings = spec["collision"]
                pieces = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces),
                                         threshold=settings["threshold"],
                                         max_convex_hull=settings["max_convex_hull"],
                                         mcts_iterations=settings["mcts_iterations"], seed=0)
                volumes = [trimesh.Trimesh(v, f).convex_hull.volume for v, f in pieces]
                if not pieces or min(volumes) <= 0:
                    raise ValueError(f"No valid convex collision pieces: {name}")
                for index, ((vertices, faces), volume) in enumerate(zip(pieces, volumes)):
                    file = output / f"{name}_contact_{index}.stl"
                    trimesh.Trimesh(vertices, faces).export(file)
                    dependencies[str(file.relative_to(ROOT))] = digest(file)
                    mesh_name = f"{name}_contact_mesh_{index}"
                    ET.SubElement(assets, "mesh", name=mesh_name, file=file.name)
                    mass = spec.get("mass_kg", 1.0) * volume / sum(volumes)
                    ET.SubElement(body, "geom", name=f"{name}_contact_{index}", type="mesh",
                                  mesh=mesh_name, mass=str(mass),
                                  friction=f"{spec['friction']} .005 .0001",
                                  rgba="0.85 0.65 0.22 0")
            else:
                raise ValueError(f"Unsupported collision conversion: {method}")
        scene = output / "scene.xml"
        ET.indent(root)
        ET.ElementTree(root).write(scene, encoding="unicode")
        model = mujoco.MjModel.from_xml_path(str(scene))
        if model.nu != len(robot["joint_names"]):
            raise ValueError("Source objects changed robot actuator count")
        for name in body_names:
            model.body(name)
        receipt = {
            "task_id": task_id, "scope": "source_object_scene_diagnostic",
            "backend": "mujoco", "robot": "dual_x5_wuji",
            "scene_sha256": digest(scene), "layout_sha256": layout_digest(layout),
            "source_asset_manifest_sha256": digest(ROOT / layout["manifest"]),
            "robot_template_sha256": digest(ROBOT),
            "robot_metadata_sha256": digest(robot_meta),
            "mesh_dependencies": dependencies,
            "source_asset_dependencies": source_dependencies,
            "collision_conversions": collision_conversions,
            "body_names": body_names, "nq": model.nq, "nv": model.nv, "nu": model.nu,
            "limitations": [
                "Original RoboDojo object meshes, adapted fixed layout and MuJoCo contact conversion",
                "P01 T contact reconstructed as two boxes because its USD collider is disabled" if task_id == "P01"
                else "I03 SDF key and slot contact approximated with convex parts; aperture equivalence unverified",
                "I03 contact friction adapted to 0.45 from metadata 0.0" if task_id == "I03"
                else "Source target cushion is visual-only",
                "No source reset, policy, score lifecycle, or contact-force equivalence",
                "Wuji-to-X5 mount is not hardware calibrated",
            ],
        }
        (output / "scene.json").write_text(json.dumps(receipt, indent=2) + "\n")
        return scene
    except Exception:
        # This directory was created by this call; preserve pre-existing outputs.
        shutil.rmtree(output)
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True, choices=tuple(sorted(json.loads(LAYOUTS.read_text()))))
    args = parser.parse_args()
    print(build(args.task))
