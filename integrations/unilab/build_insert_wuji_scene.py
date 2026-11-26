"""Bind RoboDojo I01 objects to the existing dual-X5 Wuji robot template.

This is a dexterous-hand scene diagnostic. It does not port the source
controller, USD contact model, or complete RoboDojo reward lifecycle.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / "assets/generated/dual_x5_wuji/scene.xml"
OBJECTS = ROOT / "assets/generated/I01-convex/scene.xml"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def resolve_robot_mesh(raw: str) -> Path:
    """Rebase generated absolute paths so a copied checkout remains usable."""
    path = Path(raw)
    if path.is_absolute():
        parts = path.parts
        if "assets" not in parts:
            raise ValueError(f"Robot mesh outside assets tree: {raw}")
        path = ROOT.joinpath(*parts[parts.index("assets"):])
    else:
        path = ROBOT.parent / path
    if not path.is_file():
        raise FileNotFoundError(path)
    return path.resolve()


def build(output: Path) -> Path:
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing scene: {output}")
    robot_receipt = json.loads((ROBOT.parent / "robot.json").read_text())
    if digest(ROBOT) != robot_receipt["template_sha256"]:
        raise ValueError("Wuji template differs from its build receipt")
    robot = ET.parse(ROBOT).getroot()
    objects = ET.parse(OBJECTS).getroot()
    robot.set("model", "I01_dual_x5_wuji_diagnostic")
    robot_assets = robot.find("asset")
    object_assets = objects.find("asset")
    names = {node.get("name") for node in robot_assets if node.get("name")}
    dependencies = {}
    for mesh in robot_assets.findall("mesh"):
        path = resolve_robot_mesh(mesh.get("file"))
        mesh.set("file", str(path))
        dependencies[str(path)] = digest(path)
    for mesh in object_assets.findall("mesh"):
        if mesh.get("name") in names:
            raise ValueError(f"Duplicate robot/object mesh: {mesh.get('name')}")
        new_mesh = copy.deepcopy(mesh)
        path = (OBJECTS.parent / mesh.get("file")).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        new_mesh.set("file", str(path))
        dependencies[str(path)] = digest(path)
        robot_assets.append(new_mesh)
    world = robot.find("worldbody")
    source_world = objects.find("worldbody")
    for geom in list(world.findall("geom")):
        if geom.get("name") == "diagnostic_table":
            world.remove(geom)
    for node in source_world:
        if node.tag == "light":
            continue
        if node.tag == "body" and node.get("name") in robot_receipt["body_names"]:
            raise ValueError(f"Duplicate robot/object body: {node.get('name')}")
        world.append(copy.deepcopy(node))
    output.mkdir(parents=True)
    scene = output / "scene.xml"
    ET.indent(robot)
    ET.ElementTree(robot).write(scene, encoding="unicode")
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nu != len(robot_receipt["joint_names"]):
        raise ValueError("Robot actuator count changed during scene assembly")
    for name in ("tube0", "tube1", "tube2", "slot",
                 "left_wuji_l_wrist", "right_wuji_r_wrist"):
        model.body(name)
    if any(model.joint(f"{name}_free").qposadr[0] < model.nu
           for name in ("tube0", "tube1", "tube2")):
        raise ValueError("Free objects must follow controlled joints")
    receipt = {
        "scope": "I01 dexterous-hand scene diagnostic",
        "task_id": "I01",
        "robot": "dual_x5_wuji",
        "robot_template_sha256": digest(ROBOT),
        "robot_metadata_sha256": digest(ROBOT.parent / "robot.json"),
        "object_scene_sha256": digest(OBJECTS),
        "scene_sha256": digest(scene),
        "mesh_dependencies": dependencies,
        "nq": model.nq, "nv": model.nv, "nu": model.nu,
        "limitations": [
            "Converted RoboDojo USD objects use convex collision approximations",
            "Virtual Wuji-to-X5 mount is not hardware calibrated",
            "No insertion controller or source-equivalent reward lifecycle",
        ],
    }
    (output / "scene.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return scene


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "assets/generated/I01-wuji")
    print(build(parser.parse_args().output))
