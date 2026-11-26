"""Build clearly labeled primitive-only UniLab diagnostics for unported tasks.

These scenes exercise robot loading, reset, stepping, and state recording. They
are not conversions of the source benchmark's assets, contacts, or reset law.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from integrations.unilab.build_insert_wuji_scene import ROBOT, resolve_robot_mesh

ROOT = Path(__file__).resolve().parents[2]
LAYOUTS = ROOT / "profiles/diagnostic_proxy_scenes.json"
OUTPUT_ROOT = ROOT / "assets/generated/task_proxies"
GEOMETRIES = frozenset({"box", "capsule", "cylinder", "ellipsoid", "sphere"})


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def layout_digest(layout: dict) -> str:
    canonical = json.dumps(layout, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(canonical).hexdigest()


def vector(values, count: int) -> str:
    if len(values) != count or any(type(value) not in (int, float) for value in values):
        raise ValueError(f"Expected {count} numeric vector components: {values!r}")
    return " ".join(str(value) for value in values)


def build(task_id: str, output: Path | None = None) -> Path:
    layouts = json.loads(LAYOUTS.read_text(encoding="utf-8"))
    if task_id not in layouts:
        raise ValueError(f"No primitive diagnostic layout for {task_id}")
    layout = layouts[task_id]
    if layout.get("provenance") != "self_authored_geometry_proxy":
        raise ValueError("Primitive layouts must identify their provenance")
    output = (output or OUTPUT_ROOT / task_id).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite scene: {output}")
    robot_meta = ROOT / "assets/generated/dual_x5_wuji/robot.json"
    robot = json.loads(robot_meta.read_text())
    if digest(ROBOT) != robot["template_sha256"]:
        raise ValueError("Wuji robot template differs from its receipt")
    root = ET.parse(ROBOT).getroot()
    root.set("model", f"{task_id}_primitive_diagnostic")
    world = root.find("worldbody")
    ET.SubElement(world, "light", name="proxy_key_light", pos="-0.6 -0.3 2.0",
                  dir="0.3 0.1 -1", directional="true", diffuse="0.9 0.9 0.9")
    ET.SubElement(world, "light", name="proxy_fill_light", pos="0.5 0.2 1.8",
                  dir="-0.2 -0.2 -1", directional="true", diffuse="0.5 0.5 0.5")
    dependencies = {}
    for mesh in root.find("asset").findall("mesh"):
        path = resolve_robot_mesh(mesh.get("file"))
        mesh.set("file", os.path.relpath(path, output))
        dependencies[str(path.relative_to(ROOT))] = digest(path)
    names = set(robot["body_names"])
    for body_spec in layout["bodies"]:
        name = body_spec["name"]
        if not name or name in names:
            raise ValueError(f"Duplicate or empty body name: {name!r}")
        names.add(name)
        body = ET.SubElement(world, "body", name=name, pos=vector(body_spec["pos"], 3))
        motion = body_spec["motion"]
        if motion == "free":
            ET.SubElement(body, "freejoint", name=f"{name}_free")
        elif motion == "hinge":
            ET.SubElement(body, "joint", name=f"{name}_hinge", type="hinge",
                          axis=vector(body_spec["axis"], 3), damping="0.05")
        elif motion != "fixed":
            raise ValueError(f"Unsupported primitive motion: {motion}")
        for index, geom_spec in enumerate(body_spec["geoms"]):
            kind = geom_spec["type"]
            if kind not in GEOMETRIES:
                raise ValueError(f"Unsupported primitive geometry: {kind}")
            attributes = {
                "name": f"{name}_proxy_{index}", "type": kind,
                "size": vector(geom_spec["size"], 3 if kind in ("box", "ellipsoid") else 1 if kind == "sphere" else 2),
                "rgba": vector(geom_spec.get("rgba", [0.25, 0.55, 0.85, 1]), 4),
            }
            if "pos" in geom_spec:
                attributes["pos"] = vector(geom_spec["pos"], 3)
            if "quat" in geom_spec:
                attributes["quat"] = vector(geom_spec["quat"], 4)
            if motion in ("free", "hinge"):
                mass = geom_spec.get("mass")
                if type(mass) not in (int, float) or mass <= 0:
                    raise ValueError(f"Dynamic proxy {name} needs explicit positive mass")
                attributes["mass"] = str(mass)
            if body_spec.get("visual_only"):
                attributes.update(contype="0", conaffinity="0")
            ET.SubElement(body, "geom", **attributes)
    output.mkdir(parents=True)
    scene = output / "scene.xml"
    ET.indent(root)
    ET.ElementTree(root).write(scene, encoding="unicode")
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nu != len(robot["joint_names"]):
        raise ValueError("Primitive scene changed robot actuator count")
    for name in names:
        model.body(name)
    receipt = {
        "task_id": task_id, "scope": "self_authored_geometry_proxy",
        "robot": "dual_x5_wuji", "backend": "mujoco",
        "scene_sha256": digest(scene), "layout_sha256": layout_digest(layout),
        "robot_template_sha256": digest(ROBOT),
        "robot_metadata_sha256": digest(robot_meta),
        "mesh_dependencies": dependencies, "body_names": [body["name"] for body in layout["bodies"]],
        "nq": model.nq, "nv": model.nv, "nu": model.nu,
        "limitations": [
            "Self-authored primitive geometry; source assets and source physics are not ported",
            "Fixed diagnostic initial state, not source seeded reset",
            "No task policy or task-signal adapter; task success is unknown",
            "Wuji-to-X5 mount is a virtual alignment, not hardware calibrated",
        ],
    }
    (output / "scene.json").write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")
    return scene


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True, help="Task ID or 'all'")
    args = parser.parse_args()
    layouts = json.loads(LAYOUTS.read_text())
    ids = sorted(layouts) if args.task == "all" else [args.task]
    for identifier in ids:
        print(build(identifier))
