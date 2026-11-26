> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# Runnable task starter implementation plan

> Execution: superpowers:executing-plans, inline implementation and one independent final review

Goal: turn the selected task catalog into a portable initial benchmark with real source-environment runs, explicit asset provenance and physics diagnostic extension points

Architecture: retain task JSON as the catalog; add independent execution profiles for original environments and later dexterous adaptations. A backend adapter supplies reset/step/snapshot/capabilities/synchronize/close; the runner records actions, task outcomes and optional physical diagnostics. Original task semantics stay in the upstream environment. No successful-policy claim follows from a smoke test

Tech stack: Python >=3.10; standard library catalog/records/tests; optional isolated Gymnasium Robotics and MuJoCo installation for source runs; existing Isaac Sim environment remains untouched

Spec: docs/task-contract.md, docs/result-contract.md, docs/roadmap.md and user-approved six-category framework

Constraints: no remote repo creation; no shared-environment package changes; no silent downloads of unlicensed models; null for unavailable data; distinguish original task from adapted task; preserve prior experiments

Review focus: truncated runs mistaken for failures; source-success bool missing or transient; unsupported contacts reported as zero; timing accidentally measures logging; simulation state leaking across episodes; source assets without license/hash; package install falsely implying working env

## 1. Sources and assets
- [x] Inspect upstream RoboDojo, Farama Robotics, Bench2Dex and UniLab source revisions
- [x] Save concise task/source matrix, exact scoring locations and asset availability; proposed assets listed separately
- [x] Resolve a feasible subset of actual original environments without changing robot/task semantics

## 2. Runner and diagnostics
- [x] Write failing runner tests: missing outcome remains unknown, bounded smoke is not scored failure, exception saves incomplete report, unique output paths, backend rejection, physics null handling
- [x] Implement protocol and run recorder, source environment adapter, action modes zero/random/recorded, reproducibility metadata and optional contact probes
- [x] Implement runtime validation and test success/failure/truncation/reset behavior
- [x] Add CLI run/inspect and independent profiles; leave unimplemented backends explicitly unsupported

## 3. Execution evidence
- [x] Isolated local environment; fixed package versions; inspect actual source scorer and assets
- [x] Run selected original tasks with several seeds, save initial state/actions/metrics/config/version; verify repeatability and headless rendering when available
- [x] Test parameter override recording and a bounded multi-environment CPU execution path; label timing scope honestly
- [ ] Deferred: existing Isaac task bridge; this batch records prior experiments only, with no claim of a unified Isaac runner

## 4. Handoff and review
- [x] List remaining assets and 3D generation requirements; distinguish inaccessible vs absent assets
- [x] Document commands, architecture, extension guide and UniLab mapping against inspected source
- [x] Independent final review, targeted fixes and full test run
- [x] Update README/current status without promoting unrun tasks or claiming all backends integrated

No Git repository exists: preserve files and a work ledger rather than create commits. User has explicitly delegated implementation choices; no intermediate approval required
