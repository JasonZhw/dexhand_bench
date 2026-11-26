"""Convert pinned RoboDojo charger/socket collision meshes into a Wuji diagnostic."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

# Load USD before other native libraries in this conversion process.
from pxr import Usd, UsdGeom, UsdPhysics
import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from dexbench.assets import verify_asset
from integrations.unilab.build_insert_wuji_scene import ROBOT, digest, resolve_robot_mesh

MANIFEST = ROOT / "asset_manifests/I02-robodojo.json"
LAYOUT = ROOT / "profiles/layouts/I02-fixed-diagnostic.json"


def collision_parts(path: Path):
    stage = Usd.Stage.Open(str(path.resolve()))
    if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError(f"Expected Z-up meter-scale USD: {path}")
    cache = UsdGeom.XformCache()
    parts, omitted = [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        mesh = UsdGeom.Mesh(prim)
        transform = cache.GetLocalToWorldTransform(prim)
        vertices = np.array([tuple(transform.Transform(point))
                             for point in mesh.GetPointsAttr().Get()])
        indices = list(mesh.GetFaceVertexIndicesAttr().Get())
        faces, offset = [], 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            polygon = indices[offset:offset + count]
            for index in range(1, count - 1):
                faces.append((polygon[0], polygon[index], polygon[index + 1]))
            offset += count
        if offset != len(indices):
            raise ValueError(f"Invalid USD mesh topology: {prim.GetPath()}")
        part = trimesh.Trimesh(vertices, faces, process=True)
        if len(part.faces) == 0 or part.convex_hull.volume <= 0:
            # One socket mesh is an authored zero-thickness plane. MuJoCo
            # cannot represent its contact surface as a convex mesh.
            omitted.append(str(prim.GetPath()))
            continue
        parts.append(part)
    if not parts:
        raise ValueError(f"No enabled collision meshes in {path}")
    return parts, omitted


def build(output: Path):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    manifest = json.loads(MANIFEST.read_text())
    for entry in manifest["files"]:
        if verify_asset(entry, ROOT)["status"] != "verified":
            raise ValueError(f"Missing or changed source asset: {entry['local_path']}")
    source_dependencies = {
        str((ROOT / entry["local_path"]).resolve()): entry["sha256"]
        for entry in manifest["files"]
    }
    metadata = {}
    sources = {}
    for name in ("charger", "socket"):
        directory = ROOT / f"assets/robodojo/Assets/Object/RoboDojo/Rigid/{name}/00000"
        sources[name] = directory / "object.usdz"
        metadata[name] = json.loads((directory / "metadata.json").read_text())
    robot_receipt = json.loads((ROBOT.parent / "robot.json").read_text())
    if digest(ROBOT) != robot_receipt["template_sha256"]:
        raise ValueError("Wuji robot template differs from its receipt")
    layout = json.loads(LAYOUT.read_text())
    if layout["scope"] != "adapted_fixed_layout_not_source_reset":
        raise ValueError("I02 layout must retain explicit adaptation provenance")
    root = ET.parse(ROBOT).getroot()
    root.set("model", "I02_dual_x5_wuji_diagnostic")
    assets, world = root.find("asset"), root.find("worldbody")
    dependencies = {}
    for mesh in assets.findall("mesh"):
        path = resolve_robot_mesh(mesh.get("file"))
        mesh.set("file", str(path))
        dependencies[str(path)] = digest(path)
    for geom in list(world.findall("geom")):
        if geom.get("name") == "diagnostic_table":
            world.remove(geom)
    table = layout["table"]
    ET.SubElement(world, "geom", name="diagnostic_table", type="box",
                  pos=" ".join(map(str, table["position_m"])),
                  size=" ".join(map(str, table["size_half_m"])),
                  friction=".8 .005 .0001", rgba=".5 .5 .5 1")
    top = table["position_m"][2] + table["size_half_m"][2]
    output.mkdir(parents=True)
    omitted_colliders = {}
    for name in ("socket", "charger"):
        parts, omitted_colliders[name] = collision_parts(sources[name])
        bounds = np.vstack([part.bounds for part in parts])
        minimum_z = float(bounds[:, 2].min())
        xy = layout["objects_xy_m"][name]
        z = top + layout["clearance_m"] - minimum_z
        body = ET.SubElement(world, "body", name=name, pos=f"{xy[0]} {xy[1]} {z}")
        if name == "charger":
            ET.SubElement(body, "freejoint", name="charger_free")
        # MuJoCo uses the hull of each source collision part; an authored USD
        # mesh can be open, so raw signed mesh volume is not a valid weight.
        volumes = [part.convex_hull.volume for part in parts]
        for index, (part, volume) in enumerate(zip(parts, volumes)):
            mesh_name = f"{name}_collision_{index}"
            file = output / f"{mesh_name}.stl"
            part.export(file)
            dependencies[str(file)] = digest(file)
            ET.SubElement(assets, "mesh", name=mesh_name, file=str(file))
            options = {"name": mesh_name, "type": "mesh", "mesh": mesh_name,
                       "friction": f"{metadata[name]['physics']['friction']} .005 .0001",
                       "rgba": ".2 .5 .9 1" if name == "charger" else ".8 .6 .2 1"}
            if name == "charger":
                options["mass"] = str(metadata[name]["physics"]["mass"] * volume / sum(volumes))
            else:
                # Static socket is welded to the world. A positive inertial
                # bookkeeping mass is required by MuJoCo but cannot move it.
                options["mass"] = str(volume / sum(volumes))
            ET.SubElement(body, "geom", **options)
    scene = output / "scene.xml"
    ET.indent(root)
    ET.ElementTree(root).write(scene, encoding="unicode")
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nu != len(robot_receipt["joint_names"]) or model.joint("charger_free").qposadr[0] != model.nu:
        raise ValueError("Unexpected I02 robot/object joint ordering")
    for name in ("charger", "socket", "left_wuji_l_wrist", "right_wuji_r_wrist"):
        model.body(name)
    receipt = {
        "task_id": "I02", "scope": "adapted_fixed_layout_scene_diagnostic",
        "robot": "dual_x5_wuji", "scene_sha256": digest(scene),
        "robot_template_sha256": digest(ROBOT),
        "robot_metadata_sha256": digest(ROBOT.parent / "robot.json"),
        "source_asset_manifest_sha256": digest(MANIFEST),
        "layout_sha256": digest(LAYOUT),
        "mesh_dependencies": dependencies,
        "source_asset_dependencies": source_dependencies,
        "omitted_degenerate_colliders": omitted_colliders,
        "nq": model.nq, "nv": model.nv, "nu": model.nu,
        "limitations": [
            "Self-chosen fixed XY layout, not a seeded RoboDojo reset",
            "Source USD collision meshes compiled as MuJoCo convex hulls",
            "One zero-thickness socket collider omitted; contact equivalence is not established",
            "Static source socket mass 0 replaced by bookkeeping mass 1 kg for MuJoCo compilation",
            "No insertion policy, source reward lifecycle, or hardware-calibrated hand mount",
        ],
    }
    (output / "scene.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return scene


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "assets/generated/I02-wuji")
    print(build(parser.parse_args().output))
