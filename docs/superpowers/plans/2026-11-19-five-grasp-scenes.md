# Five local grasp scenes implementation plan

**Goal:** Make G01–G05 inspectable and executable through the existing UniLab/MuJoCo runtime, with real observations connected to the draft evaluator

**Architecture:** Keep the 21 task JSONs as scoring contracts. Add five launch profiles for official Bench2Dex object geometry, a shared builder using the existing X5 robot assets, and a shared UniLab runner. Use the existing parallel gripper as a diagnostic baseline; this is not a new dexterous-hand policy or a reproduction of the official complete tasks

**Scope decision:** The user delegated selection of five basic tasks. Select apple, bottle, battery, screwdriver handle and frying-pan handle; the latter two add grasp-region/load constraints. Keep proposed materials, mass conversions, coordinate changes and region definitions explicit. Do not replace official geometry with unnamed primitives or report a no-action run as task completion

## Implementation

- [x] Fetch the minimal object files at fixed Bench2Dex Assets revision bf65215f844d1fca30750e4cc70d7266650abf0a; record checksums and extraction transformations
- [x] Add scene-builder tests for five separate models, scalar robot joints before free objects, finite mass, metric scale and no initial table penetration; build against official meshes and existing X5
- [x] Add five unified launch profiles and a shared UniLab runner; record initial state, real poses/velocities, geometric contact, missing force/slip and measured step time; use existing evaluation rules
- [x] Verify reset independence with two environments; verify action/control response separately from task success; reject unsupported backends
- [x] Run five small local episodes, save video/PNG/config/trace/evaluation/receipt and a browsable overview; run the full test suite

## Boundaries

No policy training or server use. A smoke run can legitimately fail the grasp task. Source USD collision declarations are not automatically equivalent to MuJoCo convex geometry. Any handle-region annotation is our explicit proposed region, not an official segmentation. Live state and task results must be distinguished from offline geometric contact and from solver force measurements. All current launch profiles use MuJoCo; other engines remain pending

## Progress

- Repository clean at 6378caa; existing I01 objects and X5 models verified loadable
- Official objects located; no full 19.7 GB asset download needed

- Completed five UniLab/MuJoCo diagnostics, each with two environments and 500 steps; independent physical/observation resets verified
- Final evidence: `runs/grasp-five-final-20261119`; published local gallery: `docs/evidence/grasp-five/index.html`
- All five videos verified with ffprobe: 640×480, 25 fps, 50 frames, 2 seconds
- Full suite: 128 tests, 121 passed, 7 skipped due to absent source-environment dependencies; 21 task contracts and six source asset checksums verified
- Review fixes: changed profile/geometry requires regeneration; external mesh hashes checked at launch; restricted grasps reject other robot-link support; object penetration includes the table; G05 source roll corrected to -90°
- Independent review reran the ten new grasp-scene tests and checked final receipts; no remaining blocking issue within the diagnostic scope
- Runtime result: all grasp scores false under the probe controller. Battery max reconstructed object-contact penetration was 5.90 mm; physical contact calibration and successful policies remain future work
