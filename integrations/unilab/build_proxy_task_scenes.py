"""Build clearly labeled primitive-only UniLab diagnostics for unported tasks.

These scenes exercise robot loading, reset, stepping, and state recording. They
are not conversions of the source benchmark's assets, contacts, or reset law.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from dexbench.assets import verify_asset
from integrations.unilab.build_insert_wuji_scene import ROBOT, resolve_robot_mesh

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


def verify_source_mjcf(layout: dict, workspace: Path = ROOT) -> dict:
    """Require adapted primitive parameters to match a pinned source MJCF."""
    spec = layout["source_mjcf"]
    manifest_path = workspace / spec["manifest"]
    manifest = json.loads(manifest_path.read_text())
    entries = manifest["files"]
    if any(verify_asset(entry, workspace)["status"] != "verified" for entry in entries):
        raise ValueError("Source MJCF files are missing or changed")
    selected = [entry for entry in entries if entry["local_path"] == spec["file"]]
    if len(selected) != 1:
        raise ValueError("Source MJCF file not pinned exactly once")
    if "defaults_file" in spec and sum(entry["local_path"] == spec["defaults_file"] for entry in entries) != 1:
        raise ValueError("Source MJCF defaults file not pinned exactly once")
    source = ET.parse(workspace / spec["file"]).getroot()
    defaults = ET.parse(workspace / spec["defaults_file"]).getroot() if "defaults_file" in spec else None
    option = source.find("option")
    if option is None and defaults is not None:
        option = defaults.find("option")
    if option is None or float(option.get("timestep", "0.002")) != spec["timestep"]:
        raise ValueError("Source timestep differs from layout")
    if "iterations" in spec and int(option.get("iterations", "-1")) != spec["iterations"]:
        raise ValueError("Source solver iterations differ from layout")

    def source_body(body_name):
        body = next((body for body in source.findall("./worldbody/body")
                     if body.get("name") == body_name), None)
        if body is None:
            raise ValueError(f"Source body missing: {body_name}")
        return body

    def source_geom(body_name):
        body = source_body(body_name)
        geom = next((geom for geom in body.findall("geom")
                     if geom.get("name") == body_name), None)
        if geom is None:
            raise ValueError(f"Source geom missing: {body_name}")
        return geom

    def numbers(geom, field):
        return [float(value) for value in geom.get(field, "").split()]

    object_layout = next((body for body in layout["bodies"]
                          if body["name"] == spec.get("target_body", "object")), None)
    if object_layout is None:
        raise ValueError("Source object body missing from layout")
    source_object_body = source_body(spec["object_body"])

    def mass_from_geom(geom):
        if geom.get("mass") is not None:
            return float(geom.get("mass"))
        density = float(geom.get("density", spec.get("default_density", "nan")))
        size = numbers(geom, "size")
        if geom.get("type") == "box":
            volume = math.prod(2 * component for component in size)
        elif geom.get("type") == "cylinder":
            volume = math.pi * size[0] ** 2 * 2 * size[1]
        else:
            raise ValueError("Source object mass cannot be determined from its MJCF")
        return density * volume

    if "object_geom_names" in spec:
        if spec.get("default_density") != 1000:
            raise ValueError("Source object default density must use the MuJoCo default")
        if (defaults is None or defaults.find("./default/geom") is None
                or numbers(defaults.find("./default/geom"), "friction") != spec["default_friction"]):
            raise ValueError("Source object default friction differs from layout")
        if float(defaults.find("./default/geom").get("margin", "nan")) != spec["default_margin"]:
            raise ValueError("Source object default margin differs from layout")
        source_names = spec["object_geom_names"]
        if (len(source_names) != len(object_layout["geoms"])
                or len(source_names) != len(set(source_names))
                or len(source_names) != len(source_object_body.findall("geom"))):
            raise ValueError("Source object component count differs from layout")
        source_mass = 0.0
        for source_name, object_geom in zip(source_names, object_layout["geoms"]):
            source_object = next((geom for geom in source_object_body.findall("geom")
                                  if geom.get("name") == source_name), None)
            if source_object is None or object_geom.get("source_geom") != source_name:
                raise ValueError(f"Source object component missing or reordered: {source_name}")
            for field in ("type", "size", "pos"):
                actual = (source_object.get(field) if field == "type" else
                          numbers(source_object, field) if source_object.get(field) is not None else [0, 0, 0])
                expected = object_geom.get(field, [0, 0, 0])
                if actual != expected:
                    raise ValueError(f"Source object {source_name} {field} differs from layout")
            for field, default in (("condim", 3), ("contype", 1), ("conaffinity", 1)):
                if int(source_object.get(field, default)) != object_geom.get(field, default):
                    raise ValueError(f"Source object {source_name} {field} differs from layout")
            friction = (numbers(source_object, "friction") if source_object.get("friction") is not None
                        else spec["default_friction"])
            if friction != object_geom.get("friction"):
                raise ValueError(f"Source object {source_name} friction differs from layout")
            margin = float(source_object.get("margin", spec["default_margin"]))
            if margin != object_geom.get("margin"):
                raise ValueError(f"Source object {source_name} margin differs from layout")
            component_mass = mass_from_geom(source_object)
            if not math.isclose(component_mass, object_geom["mass"], rel_tol=0, abs_tol=1e-10):
                raise ValueError(f"Source object {source_name} mass differs from layout")
            source_mass += component_mass
        joints = source_object_body.findall("joint")
        if (len(joints) != spec["joint_count"] or
                any(float(joint.get("damping", "nan")) != object_layout.get("joint_damping")
                    for joint in joints)):
            raise ValueError("Source object joint damping or count differs from layout")
        goal_layout = next((body for body in layout["bodies"]
                            if body["name"] == spec["goal_body"]), None)
        goal_site = next((site for site in source.findall("./worldbody/site")
                          if site.get("name") == "eps_ball"), None)
        if (goal_layout is None or goal_site is None
                or numbers(goal_site, "pos") != numbers(source_object_body, "pos")
                or goal_layout["pos"] != object_layout["pos"]):
            raise ValueError("Source pen goal position differs from layout")
        source_geom_count = len(source_names)
    else:
        if len(object_layout["geoms"]) != 1:
            raise ValueError("Source object must map to one diagnostic geom")
        source_object = source_geom(spec["object_body"])
        object_geom = object_layout["geoms"][0]
        for field in ("type", "size"):
            actual = source_object.get(field) if field == "type" else numbers(source_object, field)
            if actual != object_geom[field]:
                raise ValueError(f"Source object {field} differs from layout")
        source_mass = mass_from_geom(source_object)
        if not math.isclose(source_mass, object_geom["mass"], rel_tol=0, abs_tol=1e-12):
            raise ValueError("Source object mass differs from layout")
        if source_object.get("friction") is None and spec.get("default_friction") != [1, 0.005, 0.0001]:
            raise ValueError("Source object default friction must use the MuJoCo default")
        source_friction = (numbers(source_object, "friction")
                           if source_object.get("friction") is not None else spec["default_friction"])
        if source_friction is None or source_friction != object_geom.get("friction"):
            raise ValueError("Source object friction differs from layout")
        if int(source_object.get("condim", "3")) != object_geom.get("condim", 3):
            raise ValueError("Source object condim differs from layout")
        source_joint = source_object_body.find("joint")
        if (source_joint is None or source_joint.get("type") != "free"
                or float(source_joint.get("damping", "nan")) != object_layout.get("joint_damping")):
            raise ValueError("Source object joint damping differs from layout")
        source_geom_count = 1
    result = {"source_file": spec["file"], "manifest_sha256": digest(manifest_path),
              "source_asset_dependencies": {entry["local_path"]: digest(workspace / entry["local_path"])
                                            for entry in entries},
              "source_object_mass_kg": source_mass, "source_geom_count": source_geom_count}
    if "table_body" in spec:
        source_table = source_geom(spec["table_body"])
        if source_table.get("type") != "box":
            raise ValueError("Source table must be a box")
        if numbers(source_table, "size")[:2] != layout["source_table"]["size_xy"]:
            raise ValueError("Source table size differs from layout")
        if numbers(source_table, "friction") != layout["source_table"]["friction"]:
            raise ValueError("Source table friction differs from layout")
        result["source_table_thickness_half_m"] = numbers(source_table, "size")[2]
    return result


def build(task_id: str, output: Path | None = None) -> Path:
    layouts = json.loads(LAYOUTS.read_text(encoding="utf-8"))
    if task_id not in layouts:
        raise ValueError(f"No primitive diagnostic layout for {task_id}")
    layout = layouts[task_id]
    if layout.get("provenance") != "self_authored_geometry_proxy":
        raise ValueError("Primitive layouts must identify their provenance")
    source_mjcf = verify_source_mjcf(layout) if "source_mjcf" in layout else None
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
    if source_mjcf is not None:
        root.find("option").set("timestep", str(layout["source_mjcf"]["timestep"]))
        if "iterations" in layout["source_mjcf"]:
            root.find("option").set("iterations", str(layout["source_mjcf"]["iterations"]))
        if "source_table" in layout:
            table = next(geom for geom in world.findall("geom") if geom.get("name") == "diagnostic_table")
            half_z = float(table.get("size").split()[2])
            table.set("size", vector([*layout["source_table"]["size_xy"], half_z], 3))
            table.set("friction", vector(layout["source_table"]["friction"], 3))
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
        body_attrs = {"name": name, "pos": vector(body_spec["pos"], 3)}
        if "quat" in body_spec:
            body_attrs["quat"] = vector(body_spec["quat"], 4)
        body = ET.SubElement(world, "body", **body_attrs)
        motion = body_spec["motion"]
        if motion == "free":
            attributes = {"name": f"{name}_free"}
            if "joint_damping" in body_spec:
                attributes["damping"] = str(body_spec["joint_damping"])
                ET.SubElement(body, "joint", type="free", **attributes)
            else:
                ET.SubElement(body, "freejoint", **attributes)
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
            if "friction" in geom_spec:
                attributes["friction"] = vector(geom_spec["friction"], 3)
            if "margin" in geom_spec:
                attributes["margin"] = str(geom_spec["margin"])
            if "condim" in geom_spec:
                attributes["condim"] = str(geom_spec["condim"])
            for field in ("contype", "conaffinity"):
                if field in geom_spec:
                    attributes[field] = str(geom_spec[field])
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
            ("Selected primitive parameters checked against source MJCF; adapted robot, layout, reset and scoring"
             if source_mjcf else "Self-authored primitive geometry; source assets and source physics are not ported"),
            "Fixed diagnostic initial state, not source seeded reset",
            "No task policy or task-signal adapter; task success is unknown",
            "Wuji-to-X5 mount is a virtual alignment, not hardware calibrated",
        ],
    }
    if source_mjcf is not None:
        receipt["source_mjcf"] = source_mjcf
        receipt["source_asset_dependencies"] = source_mjcf["source_asset_dependencies"]
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
