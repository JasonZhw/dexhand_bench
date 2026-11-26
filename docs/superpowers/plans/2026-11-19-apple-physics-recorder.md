# Apple physics recorder implementation plan

> Execute inline using the already approved one-task recording flow

**Goal:** G01 on UniLab/MuJoCo produces independent task scores and runtime physical diagnostics, with a reproducible video and report

**Architecture:** Add read-only MJCF contact sensors to a derived scene inside the run directory, bind their values through the public UniSim sensor interface, normalize the samples and feed the existing DiagnosticProbe protocol. Preserve the original task scoring and offline contact geometry. Enable through the G01 launch configuration; other task profiles remain unchanged

**Tech stack:** Existing Python scene environment, UniLab, UniSim 2.7.1, MuJoCo 3.11.0, mjbatch

**Approved scope:** User approved installing the recorder on one apple task first. A successful grasp policy and a second backend are out of scope. Native contact sensors are preferable to reconstructing forces in a second simulation, which would not be the executed solver result

## Implementation

- Add `integrations/unilab/runtime_physics.py`: sensor authoring, public sensor reader, per-environment recorder consuming `ContactEnvelope`
- Add `tests/test_runtime_physics.py` first: physical support force equals weight, freefall has no contact, hand/table isolation, contact overflow is not silently treated as complete, reading does not change simulation state, force sensor timing remains explicit
- Extend `run_grasp_scene.py` and G01 profile: opt-in recording, warmup followed by fresh reset, matched timestamps, separate runtime records and summary, hashes and capabilities, disabling recording for a same-action comparison
- Run real Wuji G01 twice with recording on/off, compare actions and trajectories, verify reset and scoring, save videos and a small HTML evidence page
- Run the repository unittest suite, inspect videos and review the new diff

## Semantics and review focus

- Force means sum of absolute normal contact forces, not net force or physical accuracy
- Separate all-object, table, left-hand and right-hand groups; these overlap and must not be summed together
- Tangential relative contact speed is a slip diagnostic, not an automatic failure label; no touching hand means null
- Fixed-size sensors report matched count; overflow invalidates aggregate metrics, preserves available contacts and is counted
- Current synchronous CPU step timing includes UniLab managers and authored sensors, excludes recorder, rendering and reset; warmup is reported. No throughput ranking
- Native contact sensors describe the last physics solve before integration; position records describe the post-step state. Save both times
- Do not change original scoring thresholds, old assets or existing runs. Preserve all pre-existing uncommitted work; no publication in this step

## Verification commands

`.venv-scene/bin/python -m unittest tests.test_runtime_physics -v`

`.venv-scene/bin/python -m unittest discover -s tests -v`

`MUJOCO_GL=egl .venv-scene/bin/python integrations/unilab/run_grasp_scene.py --task G01 --robot wuji --output runs/apple-physics-20261119 --video`
