"""Copy the pinned Adroit door mechanism into a relocated UniLab diagnostic.

The robot, table, reset and policy remain adaptations. The door mechanism's
inertials, geometry and joint parameters come from the source XML.
"""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _numbers(element: ET.Element, field: str) -> list[float]:
    return [float(value) for value in element.get(field, "").split()]


def prepare_door_subtree(spec: dict, layout: dict, workspace: Path) -> tuple[ET.Element, dict]:
    source = ET.parse(workspace / spec["file"]).getroot()
    defaults = ET.parse(workspace / spec["defaults_file"]).getroot()
    frame = source.find("./worldbody/body[@name='frame']")
    door = frame.find("./body[@name='door']") if frame is not None else None
    latch = door.find("./body[@name='latch']") if door is not None else None
    if (frame is None or door is None or latch is None
            or [body.get("name") for body in frame.findall(".//body")] != ["door", "latch"]
            or [body["name"] for body in layout["bodies"]] != ["door", "latch"]):
        raise ValueError("Source door/frame/latch hierarchy differs from diagnostic layout")
    default_geom = defaults.find("./default/geom")
    default_joint = defaults.find("./default/joint")
    option = defaults.find("option")
    if (default_geom is None or default_joint is None or option is None
            or _numbers(default_geom, "friction") != spec["default_friction"]
            or float(default_geom.get("margin", "nan")) != spec["default_margin"]
            or int(option.get("noslip_iterations", "-1")) != spec["noslip_iterations"]):
        raise ValueError("Source Adroit door defaults differ from diagnostic specification")

    joints = [(door, "door_hinge", [0, 0, 1], [0, 1.57], 2),
              (latch, "latch", [0, 1, 0], [0, 1.8], 5)]
    for body, name, axis, bounds, frictionloss in joints:
        joint = body.find("joint")
        if (joint is None or joint.get("name") != name
                or joint.get("type", "hinge") != "hinge"
                or _numbers(joint, "axis") != axis
                or _numbers(joint, "range") != bounds
                or float(joint.get("frictionloss", "nan")) != frictionloss):
            raise ValueError(f"Source {name} frictionloss, axis or range differs")
    for body in (frame, door, latch):
        inertial = body.find("inertial")
        if inertial is None or float(inertial.get("mass", "nan")) <= 0:
            raise ValueError(f"Source {body.get('name')} inertial is missing")
        if any(geom.get("type") not in ("box", "cylinder", "capsule") for geom in body.findall("geom")):
            raise ValueError("Source door mechanism has unsupported geometry")

    copy = deepcopy(frame)
    source_pos = _numbers(frame, "pos")
    translation = spec["frame_translation"]
    if len(source_pos) != 3 or len(translation) != 3:
        raise ValueError("Door frame translation must have three coordinates")
    copy.set("pos", " ".join(str(a + b) for a, b in zip(source_pos, translation)))
    copy.attrib.pop("user", None)  # This source annotation does not affect mechanics.
    for geom in copy.findall(".//geom"):
        material = geom.attrib.pop("material", None)
        geom.set("friction", geom.get("friction", default_geom.get("friction")))
        geom.set("margin", geom.get("margin", default_geom.get("margin")))
        if "rgba" not in geom.attrib:
            geom.set("rgba", "0.58 0.36 0.22 1" if material == "MatWood" else "0.68 0.73 0.78 1")
    for joint in copy.findall(".//joint"):
        for field in ("limited", "margin", "armature", "damping", "frictionloss"):
            joint.set(field, joint.get(field, default_joint.get(field)))

    audit = {
        "subtree_body": "frame", "body_names": ["frame", "door", "latch"],
        "joint_names": ["door_hinge", "latch"],
        "body_masses_kg": {body.get("name"): float(body.find("inertial").get("mass"))
                           for body in (frame, door, latch)},
        "body_inertias_kg_m2": {body.get("name"): _numbers(body.find("inertial"), "diaginertia")
                                for body in (frame, door, latch)},
        "source_geom_count": sum(len(body.findall("geom")) for body in (frame, door, latch)),
        "frame_translation_m": translation,
        "default_friction": spec["default_friction"],
        "default_margin": spec["default_margin"],
        "noslip_iterations": spec["noslip_iterations"],
        "joints": {joint.get("name"): {
            "range": _numbers(joint, "range"),
            "damping": float(joint.get("damping")),
            "armature": float(joint.get("armature")),
            "frictionloss": float(joint.get("frictionloss")),
        } for joint in copy.findall(".//joint")},
    }
    return copy, audit


def verify_compiled_door(model, audit: dict, robot_body_names: set[str]) -> None:
    for name, mass in audit["body_masses_kg"].items():
        body = model.body(name)
        if not math.isclose(model.body_mass[body.id], mass, rel_tol=0, abs_tol=1e-9):
            raise ValueError(f"Compiled {name} mass differs from source door")
        for actual, expected in zip(model.body_inertia[body.id], audit["body_inertias_kg_m2"][name]):
            if not math.isclose(actual, expected, rel_tol=0, abs_tol=1e-9):
                raise ValueError(f"Compiled {name} inertia differs from source door")
    for name, expected in audit["joints"].items():
        joint = model.joint(name)
        dof = int(model.jnt_dofadr[joint.id])
        for actual, value in zip(model.jnt_range[joint.id], expected["range"]):
            if not math.isclose(actual, value, rel_tol=0, abs_tol=1e-9):
                raise ValueError(f"Compiled {name} range differs from source door")
        for field in ("damping", "armature", "frictionloss"):
            if not math.isclose(getattr(model, f"dof_{field}")[dof], expected[field],
                                rel_tol=0, abs_tol=1e-9):
                raise ValueError(f"Compiled {name} {field} differs from source door")
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mechanism_bodies = set(audit["body_names"])
    for contact in data.contact:
        first = model.body(model.geom_bodyid[contact.geom1]).name
        second = model.body(model.geom_bodyid[contact.geom2]).name
        if ((first in mechanism_bodies and second in robot_body_names)
                or (second in mechanism_bodies and first in robot_body_names)) and contact.dist < -0.001:
            raise ValueError("Door mechanism intersects robot at diagnostic reset")
