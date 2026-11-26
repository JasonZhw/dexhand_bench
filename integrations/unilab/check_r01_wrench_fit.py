"""Measure whether the R01 source wrench jaws can accept the proposed nut.

This is a static collision-geometry diagnostic, not a manipulation rollout.
The two jaw pieces are the CoACD pieces identified in the pinned R01 conversion.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull
import trimesh

from integrations.unilab.run_proxy_task_scene import validate_scene


JAW_PIECES = (5, 7)
RADII_M = (0.005, 0.004, 0.003, 0.0025, 0.002)
MAX_DIAGNOSTIC_PENETRATION_M = 0.0005
ROOT = Path(__file__).resolve().parents[2]


def _contact_model(mesh_root: Path, nut_mesh: Path) -> mujoco.MjModel:
    assets = "".join(
        f'<mesh name="jaw_{index}" file="{mesh_root / f"wrench_contact_{index}.stl"}"/>'
        for index in JAW_PIECES
    )
    jaw_geoms = "".join(
        f'<geom name="jaw_{index}" type="mesh" mesh="jaw_{index}" mass="0.04"/>'
        for index in JAW_PIECES
    )
    xml = (
        f'<mujoco><asset>{assets}<mesh name="nut" file="{nut_mesh}"/></asset>'
        '<worldbody><body name="wrench" quat=".70710678 .70710678 0 0">'
        f'<freejoint/>{jaw_geoms}</body><body name="nut">'
        '<joint type="hinge" axis="0 0 1"/>'
        '<geom name="nut" type="mesh" mesh="nut" mass="0.01"/>'
        '</body></worldbody></mujoco>'
    )
    return mujoco.MjModel.from_xml_string(xml)


def _best_two_jaw_pose(model: mujoco.MjModel) -> dict | None:
    data = mujoco.MjData(model)
    best = None
    for x_offset in np.arange(0.075, 0.0831, 0.0005):
        for y_offset in np.arange(-0.005, 0.0011, 0.0005):
            data.qpos[:] = model.qpos0
            data.qpos[:3] = (x_offset, y_offset, 0.007)
            mujoco.mj_forward(model, data)
            contacts = [
                contact for contact in data.contact
                if model.geom(contact.geom2).name == "nut"
                and model.geom(contact.geom1).name in ("jaw_5", "jaw_7")
            ]
            if {model.geom(contact.geom1).name for contact in contacts} != {"jaw_5", "jaw_7"}:
                continue
            penetration = max(0.0, -min(contact.dist for contact in contacts))
            if best is None or penetration < best["max_penetration_m"]:
                best = {
                    "max_penetration_m": float(penetration),
                    "wrench_root_offset_from_nut_m": [float(x_offset), float(y_offset), 0.007],
                    "jaw_nut_contact_count": len(contacts),
                }
    return best


def analyze_fit(mesh_root: Path) -> dict:
    mesh_root = Path(mesh_root).resolve()
    layout = json.loads((ROOT / "profiles/source_asset_scenes.json").read_text())["R01"]
    nut = layout["fixture"]["nut"]
    if nut["radius_m"] != RADII_M[0] or nut["half_height_m"] != 0.006:
        raise ValueError("R01 fixture size changed; update the fit diagnostic")
    if len(list(mesh_root.glob("wrench_contact_*.stl"))) != 8:
        raise ValueError("R01 CoACD pieces changed; reidentify the two jaw pieces")
    with tempfile.TemporaryDirectory() as directory:
        measurements = []
        for radius in RADII_M:
            nut_mesh = Path(directory) / f"nut_{radius}.stl"
            trimesh.creation.cylinder(radius=radius, height=0.012, sections=6).export(nut_mesh)
            pose = _best_two_jaw_pose(_contact_model(mesh_root, nut_mesh))
            measurements.append({
                "nut_circumradius_m": radius,
                "best_two_jaw_pose": pose,
                "within_0_5mm_penetration_limit": (
                    pose is not None
                    and pose["max_penetration_m"] <= MAX_DIAGNOSTIC_PENETRATION_M
                ),
            })
    return {
        "scope": "static_source_wrench_jaw_fit_diagnostic",
        "jaw_collision_piece_names": [f"wrench_contact_{index}" for index in JAW_PIECES],
        "configured_nut_circumradius_m": RADII_M[0],
        "diagnostic_penetration_limit_m": MAX_DIAGNOSTIC_PENETRATION_M,
        "search_root_offset_x_m": [0.075, 0.083],
        "search_root_offset_y_m": [-0.005, 0.001],
        "search_root_height_above_nut_m": 0.007,
        "results": measurements,
        "task_success": None,
        "limitations": [
            "Only two identified jaw collision pieces and an isolated hex nut are tested",
            "Static contact with small penetration is necessary but not sufficient for stable torque transfer",
            "The 0.5 mm diagnostic threshold is an engineering screening choice, not a source benchmark criterion",
            "No robot, grasp, policy, threads, or complete task rollout is evaluated",
        ],
    }


def render_jaw_projection(mesh_root: Path, analysis: dict, path: Path) -> None:
    width, height = 1100, 420
    image = Image.new("RGB", (width, height), "white")
    drawer = ImageDraw.Draw(image)
    visual = trimesh.load(mesh_root / "wrench_visual.stl", force="mesh")
    jaw_hulls = [trimesh.load(mesh_root / f"wrench_contact_{index}.stl", force="mesh")
                 for index in JAW_PIECES]
    for panel, row in enumerate((analysis["results"][0], analysis["results"][-1])):
        origin_x = 55 + panel * 550
        origin_y = 210
        scale = 10500

        def point(x: float, z: float) -> tuple[int, int]:
            return round(origin_x + (x + 0.09) * scale), round(origin_y - z * scale)

        for triangle in visual.triangles:
            if triangle[:, 0].min() > -0.05:
                continue
            drawer.polygon([point(vertex[0], vertex[2]) for vertex in triangle], fill="#d9e1eb")
        for hull, color in zip(jaw_hulls, ("#0c75a8", "#da752b")):
            vertices = hull.vertices[:, (0, 2)]
            outline = vertices[ConvexHull(vertices).vertices]
            points = [point(x, z) for x, z in outline]
            drawer.line(points + points[:1], fill=color, width=3)
        pose = row["best_two_jaw_pose"]
        if pose:
            x_offset, y_offset, _ = pose["wrench_root_offset_from_nut_m"]
            # The wrench's 90-degree X rotation maps local Z to negative world Y.
            center_x, center_z = -x_offset, y_offset
            radius = row["nut_circumradius_m"]
            vertices = [point(center_x + radius * np.cos(k * np.pi / 3),
                              center_z + radius * np.sin(k * np.pi / 3)) for k in range(6)]
            drawer.line(vertices + vertices[:1], fill="#cf3030", width=4)
        diameter_mm = row["nut_circumradius_m"] * 2000
        depth_mm = pose["max_penetration_m"] * 1000 if pose else float("nan")
        drawer.text((origin_x, 20), f"Nut diameter {diameter_mm:g} mm", fill="#13233a")
        drawer.text((origin_x, 42), f"Best two-jaw penetration {depth_mm:.2f} mm", fill="#13233a")
        drawer.text((origin_x, height - 35), "Blue/orange: converted collision jaws; red: hex nut", fill="#435269")
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def run(output: Path) -> dict:
    scene, receipt = validate_scene("R01")
    result = analyze_fit(scene.parent)
    result["scene_sha256"] = receipt["scene_sha256"]
    output.mkdir(parents=True, exist_ok=False)
    render_jaw_projection(scene.parent, result, output / "jaw-fit.png")
    (output / "jaw-fit.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = run(args.output)
    print(json.dumps({"configured_nut": report["results"][0],
                      "smallest_nut": report["results"][-1],
                      "task_success": report["task_success"]}))
