"""Check the self-authored R01 hinge with direct torque, never task control."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

import mujoco
from PIL import Image, ImageDraw

from integrations.unilab.run_proxy_task_scene import validate_scene


def run(output: Path, *, seconds: float = 2.0, torque_nm: float = 0.04) -> dict:
    if seconds <= 0 or torque_nm <= 0:
        raise ValueError("Duration and torque must be positive")
    scene, receipt = validate_scene("R01")
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    joint = model.joint("nut_hinge")
    qpos_index, dof_index = int(joint.qposadr[0]), int(joint.dofadr[0])
    output.mkdir(parents=True, exist_ok=False)
    renderer = mujoco.Renderer(model, height=480, width=640)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.18, -0.12, 0.775]
    camera.distance, camera.azimuth, camera.elevation = 0.10, 45, -75
    encoder = subprocess.Popen([
        "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
        "-pixel_format", "rgb24", "-video_size", "640x480", "-framerate", "24",
        "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
        str(output / "direct-nut-torque.mp4"),
    ], stdin=subprocess.PIPE)
    samples = []
    try:
        for frame in range(round(seconds * 24)):
            target = (frame + 1) / 24
            while data.time < target:
                data.qfrc_applied[:] = 0
                if data.time < seconds * 0.75:
                    data.qfrc_applied[dof_index] = torque_nm
                mujoco.mj_step(model, data)
            angle = float(data.qpos[qpos_index])
            samples.append({"time_s": float(data.time), "nut_rotation_rad": angle,
                            "torque_applied_nm": torque_nm if data.time < seconds * 0.75 else 0.0})
            renderer.update_scene(data, camera=camera)
            image = Image.fromarray(renderer.render())
            drawer = ImageDraw.Draw(image)
            drawer.text((12, 12), "Direct torque on custom nut hinge - no hand policy", fill="white")
            drawer.text((12, 30), f"t={data.time:.2f}s  angle={angle:.3f}rad", fill="white")
            encoder.stdin.write(image.tobytes())
        if not all(abs(row["nut_rotation_rad"]) < 100 for row in samples):
            raise ValueError("R01 nut hinge diverged")
    finally:
        renderer.close()
        encoder.stdin.close()
        result = encoder.wait()
        if result != 0:
            raise RuntimeError("ffmpeg could not encode R01 mechanism video")
    summary = {
        "task_id": "R01", "scope": "direct_joint_torque_diagnostic",
        "scene_sha256": receipt["scene_sha256"],
        "input_torque_nm": torque_nm, "duration_s": seconds,
        "final_nut_rotation_rad": samples[-1]["nut_rotation_rad"],
        "samples": samples,
        "task_success": None,
        "limitations": [
            "Torque is applied directly to the custom hinge; the wrench and hand do not turn the nut",
            "The nut does not model threads, axial advance, or preload",
            "This video checks mechanism motion and visualization, not R01 benchmark completion",
        ],
    }
    (output / "direct-nut-torque.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--torque-nm", type=float, default=0.04)
    args = parser.parse_args()
    result = run(args.output, seconds=args.seconds, torque_nm=args.torque_nm)
    print(json.dumps({key: result[key] for key in ("task_id", "scope", "final_nut_rotation_rad", "task_success")}))
