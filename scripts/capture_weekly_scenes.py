"""Capture the initial state of every currently bound UniLab task scene.

Images are MuJoCo offscreen renders of the exact MJCF bound by the UniLab
task owner. They show scene loading, not task execution or success.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from integrations.unilab.task_owner import TASK_IDS, bound_profile, load_owner


GROUPS = {
    "grasp": ("G01", "G02", "G03", "G04", "G05", "G06"),
    "in_hand": ("H01", "H02", "H03"),
    "insertion": ("I01", "I02", "I03"),
    "transport": ("M01", "M02", "M03"),
    "push": ("P01", "P02", "P03"),
    "rotation": ("R01", "R02", "R03"),
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bound_scene(task_id: str) -> tuple[Path, str, str]:
    cfg, task = load_owner(task_id)
    binding = cfg.task_catalog.binding
    kind = str(binding.kind)
    if kind == "local_grasp":
        profile = bound_profile(cfg)
    elif kind == "unilab_scene_diagnostic":
        profile = json.loads((ROOT / str(binding.profile)).read_text())
    elif kind in ("unilab_proxy_diagnostic", "unilab_source_asset_diagnostic"):
        from integrations.unilab.run_proxy_task_scene import diagnostic_profile, validate_scene

        validate_scene(task_id)
        profile = diagnostic_profile(task_id)
    else:
        raise ValueError(f"{task_id}: no current scene binding")
    scene = ROOT / profile["scene"]
    if not scene.is_file():
        raise FileNotFoundError(scene)
    receipt = scene.parent / "scene.json"
    if receipt.is_file():
        recorded = json.loads(receipt.read_text()).get("scene_sha256")
        if recorded and sha256(scene) != recorded:
            raise ValueError(f"{task_id}: scene differs from build receipt")
    return scene, kind, task["name"]


def capture(scene: Path, output: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.0, -0.12, 0.9]
    camera.distance = 1.4
    camera.azimuth = 115
    camera.elevation = -35
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # Hide collision-only overlays, as existing task renders do.
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(output)


def contact_sheet(task_ids: tuple[str, ...], output: Path) -> None:
    margin, label = 12, 34
    columns = 3
    rows = (len(task_ids) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * (640 + margin) + margin,
                              rows * (480 + label + margin) + margin), "#f2f4f7")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    for index, task_id in enumerate(task_ids):
        image = Image.open(output.parent / f"{task_id}.png").convert("RGB")
        x = margin + (index % columns) * (640 + margin)
        y = margin + (index // columns) * (480 + label + margin)
        draw.text((x + 3, y + 3), task_id, fill="#172a40", font=font)
        sheet.paste(image, (x, y + label))
    sheet.save(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--task", choices=TASK_IDS)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    task_ids = (args.task,) if args.task else TASK_IDS
    records = []
    for task_id in task_ids:
        scene, kind, name = bound_scene(task_id)
        image_path = output / f"{task_id}.png"
        capture(scene, image_path)
        records.append({
            "task_id": task_id,
            "name": name,
            "binding": kind,
            "scene": str(scene.relative_to(ROOT)),
            "scene_sha256": sha256(scene),
            "screenshot": image_path.name,
            "screenshot_sha256": sha256(image_path),
            "method": "MuJoCo EGL offscreen render of UniLab-bound MJCF at model default initial state",
            "task_success": None,
        })
        print(task_id, name, image_path, flush=True)
    if not args.task:
        for category, ids in GROUPS.items():
            contact_sheet(ids, output / f"sheet-{category}.png")
    (output / "manifest.json").write_text(
        json.dumps({"scope": "scene_loading_screenshot_only", "tasks": records},
                   ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
