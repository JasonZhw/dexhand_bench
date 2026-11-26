# Wuji and Sharpa hand variants implementation plan

> Execute inline with superpowers:executing-plans, then one independent review

Goal: add official Wuji Hand 2.2 and Sharpa Wave models as selectable embodiments for the existing five UniLab grasp tasks, retaining Allegro and all historical evidence

Architecture: use pinned native MJCFs, preserve source mechanics and hand-specific contact groups, attach each pair to the existing X5 arms, and select the robot through a shared launch-profile resolver. Task scoring JSON stays unchanged

User authorization: the user requested Wuji/Sharpa models in this continuing local benchmark project and already delegated asset/model choices and implementation. Continue in the existing checkout with its uncommitted work; no reset, new approval gate, hardware connection or publication

Approaches: selected native vendor MJCF integration because both vendors supply it. URDF/USD conversion would add avoidable mapping errors; importing only decorative meshes would not meet the control/evaluation need

Versions: Wuji Hand 2.2 bare model (20 joints/hand), Sharpa Wave with flange (22 joints/hand), native position drives. Choose explicit virtual X5 mount transforms, not claims of hardware fit. Retain source license/notice and checksum every file. Use our shared implicitfast runtime and record differences from source simulator settings

- [x] Fetch only both hand MJCF dependency closures and licenses; extend pinned-download allowlist with tests for official repositories and rejected unpinned URLs
- [x] Build vendor robots with scoped names/defaults, preserved joint ranges/mass/inertia/drives, disjoint five-finger contact groups; test source FK equivalence, channel order (52/56), finite hold/motion, removed grippers, no unintended cross-robot/table collisions
- [x] Add robot-specific probe offsets and profile resolver; `--robot allegro|wuji|sharpa` selects model/scene without changing scoring criteria. Validate limits and per-finger activation. Keep old Allegro build and evidence usable
- [x] Build all ten vendor task scenes; run real UniLab checks, independent reset and recording, render at least one scene/hand video per vendor. Extend to all tasks if affordable
- [x] Update gallery and plain-language guide, distinguish model integration from grasp-policy success, expose physics-interface limits; full tests and independent code review

Review focus: default classes must not leak into arm dynamics; source mesh/material/sensor references must survive namespacing; joint/actuator order and limits must match; palm and finger contacts must not overlap; selecting another robot must select a matching scene receipt instead of stale Allegro data

Evidence and decisions:
- Added 115 vendor source files, about 24 MB total; all checksum verified and source licenses/NOTICE retained
- New named-action/profile and physics-preservation tests failed before implementation and passed after; final full suite137 tests,130 passed7 skipped for absent original-environment dependencies
- Source drive preservation: Sharpa dampratio converted to source-compiled kv before moving-base attachment; independent reviewer confirmed all compiled joint/actuator/contact parameters unchanged
- Initial inherited0.20m mounting offset left a visible gap. Measured X5 mesh ends at localX0.084m; selected WujiX0.086m and SharpaX0.088m with2mm geometry clearance, Wuji palm rotated toward table. Current source-FK and no-initial-contact tests pass. These virtual transforms do not certify hardware fit
- All ten vendor-task combinations ran in real UniLab/MuJoCo,2 environments ×500 steps, independent reset verified. Every commanded finger flexion moved over0.025rad. All task scores false because probe does not grasp
-20 videos verified640×480,25fps,50frames,2s; source task hashes match. Gallery under docs/evidence/vendor-hands; raw data runs/vendor-hands-final-20261119
- Independent scoped review found no actionable issues; whole-suite and runtime verification completed locally
- Existing Allegro/default profiles and historical evidence retained; no server or hardware touched, no push performed
