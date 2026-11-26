# Apple IK script implementation plan

> Execute inline against the existing local UniLab scene; user requested an IK script before a learned policy

**Goal:** Run a scripted right-Wuji apple grasp attempt, with unchanged task scoring and runtime physics recording, and save the actual result and video

**Design:** Reuse `X5IK` for the arm and interpolate named finger targets. Approach above the apple, descend with the hand open, close, lift, and hold. Send joint-position actions through the existing UniLab runner. The apple remains a dynamic free body; never attach it or set its pose during execution. A fixed script is appropriate for this fixed initial scene; learned policies and multi-object generalization remain later work

**Files:** `integrations/unilab/apple_ik.py`, `profiles/controllers/G01_wuji_ik.json`, `integrations/unilab/run_grasp_scene.py`, `tests/test_apple_ik.py`, `docs/apple-ik-grasp.md`

**Constraints:** Preserve existing uncommitted work and the G01 criteria (5 cm lift, low speed, designated-hand contact held for 1 s). Use current MuJoCo backend and current assets. Keep environment 1 holding at HOME as a control. Configuration must record hand poses, phase durations and target geometry. Report failed grasp attempts as failures

## Implementation and verification

- [x] Add tests for the IK runner mode and FK-verified waypoints; verify missing behavior fails
- [x] Implement an apple-only, Wuji-only phase controller and configuration; keep unsupported embodiments explicit
- [x] Integrate with the shared runner, stage/plan records, task scorer and physics recorder
- [x] Run fixed-scene experiments, inspect contact and trajectory records, render actual-state videos; adjust only controller parameters when justified by those observations
- [x] Run existing affected tests, document a runnable command, measured result and limitations

## Review focus

Check named joint ordering and HOME-relative actions, right/left isolation, IK failures before execution, bounded interpolation, and task score/video agreement. IK reachability does not prove collision-free motion or grasp stability. No physical-accuracy claim follows from a successful lift
