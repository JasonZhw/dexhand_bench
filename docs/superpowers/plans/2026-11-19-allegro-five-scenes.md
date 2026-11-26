# Replace five grasp scenes with dual Allegro hands

The user explicitly requested replacing every gripper in the five local scenes with dexterous hands, after agreeing the next stage is robot/control/contact adaptation. Work continues in the same local checkout and preserves the existing uncommitted five-scene work and historical results.

Choice: keep the six-axis X5 arms; remove both old finger joints and bodies; mount left/right Allegro V3 MJCF models from Google DeepMind Menagerie at a pinned revision. Four fingers, sixteen position-controlled joints per hand. This is our virtual assembly, not the Bench2Dex Panda/Allegro physical assembly. Alternatives considered: original Bench2Dex USD would add another USD robot conversion; a floating hand would remove the reaching arm. Existing CPU MuJoCo route and the requested arm-plus-hand scope favor native MJCF hands.

Implementation:
- [x] Pin and fetch both Allegro MJCFs, required meshes and BSD-2 license; extend the existing bounded checksum downloader only for pinned Menagerie GitHub paths, with tests
- [x] Build a dual-X5/Allegro robot template, preserve source hand controls/limits and contact exclusions, use explicit virtual mounts and valid thumb rest positions; verify 44 controls with no old gripper joints
- [x] Replace five launch profiles with the shared robot spec; remove hardcoded 16-joint indexes from scene builder, UniLab entity binding, probe actions and recording
- [x] Read contact labels from the robot spec; record per-finger contact, palm support and other-body support; leave unsupported forces null
- [x] Validate real hand/finger actuation and two-env independent resets; run five scenes, record videos and hand close-ups; update gallery/docs and preserve old gripper evidence as historical
- [x] Run full suite and one independent code review; fix blockers, accurately report no grasp-success policy

Acceptance: all five main profiles launch arm-plus-dexterous-hand scenes; each side has four independently movable finger chains, no parallel gripper joints remain; reset restores hand and arm states; scoring remains tied to actual observations; videos visibly show finger articulation. No object teleportation or fake grasp success, no other backend claims.

Verified outcome (2026-11-19):
- Seventeen pinned Allegro source files fetched and checksum verified; both hands preserve source limits, actuator parameters and mass/inertia
- All five profiles use the shared 44-control robot; original gripper joints are absent
- Source-versus-assembled hand FK agrees, including the valid thumb rest coordinate transformation
- Five real UniLab/MuJoCo runs completed: two environments, 500 steps each, independent resets pass; both hands articulate
- Ten videos verified with ffprobe: 640×480, 25 fps, 50 frames, 2 seconds; all five scene and hand stills visually checked
- Current gallery updated; previous gripper evidence archived separately
- Full suite: 134 tests, 127 passed, 7 skipped for unavailable original-environment dependencies
- Independent read-only review: 22 focused tests passed; direct UniLab test separately moved a left finger and a right thumb in different environments and verified selective reset; no blocking or important findings
- No grasp policy or task success claimed. Battery object contact penetration remains about 6.10 mm and requires further diagnosis. Runtime force/slip remain unavailable

Evidence: `runs/allegro-five-final-20261119/`, `docs/evidence/grasp-five/`
