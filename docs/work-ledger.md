> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# Work ledger — 2026-11-11

User granted a three-hour work window for incremental implementation, asset research and physics extension interfaces. Existing six-category design is approved. No remote publishing

Decision: use optional isolated MuJoCo/Farama environments for source-task verification where available; preserve existing Isaac Sim environment. This gives actual task execution without attempting six new controllers. It does not establish success baselines or dexterous adaptation of gripper tasks

Decision: task catalog completion and execution profiles are separate. A profile can execute a source smoke while the full task adaptation remains incomplete

Source batch: reviewed 8 RoboDojo task implementations and key geometric helper semantics; all referenced object categories resolved in pinned HF file index. Reviewed 6 Bench2Dex source scenes; server audit found 8/25 entry USDs. Missing on server is not proof unavailable upstream

Implementation: runner, profiles, contact diagnostics, paired trajectory comparison, CPU-process batches and asset checksum tools added with failing import tests followed by passing stdlib tests. Optional real-environment tests were initially skipped pending isolated dependency download; they are not yet counted as verified simulation

Decision: FetchSlide version must be v4 for pinned Gymnasium Robotics 1.4.1 (verified registration source), replacing initial v3 assumption before execution

Decision: Git clone of Farama source failed during transfer; package source and individually fetched pinned files are sufficient for source verification, so do not spend time repeatedly cloning large assets

Asset inspection: opened downloaded RoboDojo USDZ objects with USD core. Tube rack, key slot and socket declare SDF collision approximation. Record this as a migration requirement; do not silently convexify holes or attribute changed geometry failures to solver accuracy

Integration RED: isolated Gymnasium Robotics 1.4.1 + MuJoCo3.3.7 could not parse Shadow XML attribute apirate; failed log preserved in docs/evidence/initial-integration-failure.log. Official MuJoCo3.2.5 XML docs still support apirate. Decision: pin MuJoCo3.2.5 instead of modifying official assets or downloading another full task package. Physics-version choice will be recorded in manifests

Host PYTHONPATH contains external ROS packages; pip initially warned about launch-ros requiring PyYAML even though it is not a benchmark dependency. Running with env -u PYTHONPATH gives pip check: No broken requirements found. Use a clean PYTHONPATH for integration/lock generation; do not install unrelated ROS dependencies

Integration: MuJoCo3.2.5 loaded the unchanged original Shadow assets. Actual parameter test then exposed enum casing (Newton vs mjSOL_NEWTON); corrected solver/integrator mapping to documented uppercase enum names. Actual simulator suite: 65 tests passed, no skipped tests. Logs in docs/evidence/test-suite.log. Upstream emits a reward-history warning for Adroit v1; preserve pinned package/source hashes and do not claim original-paper score reproduction

Final execution: five unchanged source environments ran with zero seed0 and random seed0/1 (15 episodes). All execution completed, all final source predicates false at horizon. Five headless videos generated locally. No successful policy or engine-accuracy claim

Replay: same seed/actions for H01 gives zero recorded pose difference. Iterations 20 -> 1 changes reset state as well; comparator flags reset confound. CPU process batches of 1/2 workers each complete four episodes/400 steps; timing not used for a scaling claim

Assets: 29 files, 110249662 bytes, verified against recorded SHA256; 13 USDZ files open in USD core. These are object bundles, not complete scenes. SDF compatibility remains an explicit migration constraint

Independent review: three P2 findings (nonfinite/malformed pose comparison; missing initial state falsely equal; initialization and encoder-finalize failures bypassing durable evidence). Added failing regressions, fixed validation and lifecycle, then reran full real-environment suite and a headless video lifecycle smoke. One review/fix pass, no rereview. See review-20261111.md

Final verification: 72 tests passed with the isolated simulation dependencies (no skips); pip check reports no broken requirements; catalog audit passes; both recorded trajectory comparisons rerun with strict validation; a fresh two-step EGL video run finishes with task outcome unknown as expected

2026-11-12 source bridge: external replay tests first failed on absent module, then five passed. Added portable Bench2Dex instrumented source and durable supervisor. Existing output directories are rejected; task_outcome always unknown, never inferred from process exit

Server evidence: source commit 82e04a2a4e5b728a96ccf1ae7a546762d5a260c7, clean git status, no package changes. Initial launch during incomplete file transfer failed preflight without creating run output; after completed transfer actual replay returned 0 and recorded 213 steps. Retrieved logs/states; selected apple origin did not visibly rise. G01 catalog state unchanged

Fixed layouts: downloaded I01/M01 arx_x5 seed-directory0 layout0 at pinned dataset revision, saved checksum receipts and object/scene directory dependency inventories. Inventory is deliberately not called a resolved transitive closure or runnable scene

Final review: independent reviewer identified nonfinite timeout and malformed completion receipt failures. Both reproduced RED, fixed in one pass, full suite 79 tests GREEN. See test-suite-20261112.log. No remaining review findings

2026-11-12 meeting video: optional overhead RGB capture added to external replay, preserving control loop. Timeline tests observed RED on missing module then GREEN; full suite 82 passed. Both rendered runs saved 213 states and71frames

Evidence correction: inspecting actual camera frames and source HDF5 showed apple_3 is stationary and apple_4 is the manipulated object. Preserved apple_body_video_01, reran apple4_video_01 with correct observation target. Apple4 final origin rise 0.2679998279m vs source0.2775524259m. Task outcome still unknown; endpoint rise is not task success or physical accuracy

Independent video review: no blocking bugs, one time-axis wording correction applied. Frame0 playback starts0s but state time0.05s; report explicitly explains offset. RGB capture verified against upstream sync-render-capture order; no added physics steps

Meeting artifacts: offline HTML, seven decoded H264 videos (five source smoke, original-speed dynamic replay, quarter-speed dynamic replay), state diagnostic plot, 3-minute speaking notes, media SHA256 and frame mapping verification. No PDF/slides or Feishu publication claimed

2026-11-12 RoboDojo → UniLab first contract: inspected pinned source task, scene, seed and reward owners. Chose insert_tubes scoring migration before geometry/asset/control migration. Added explicit boolean predicate state, one-transition-per-evaluation stage logic, source latched-success behavior and final-score gate. Seven focused tests observed RED→GREEN; full existing simulation suite89passed

Parity evidence: executed actual upstream RewardManager class with boolean predicate fixtures,100episodes/3000steps, matching scores and success. Excludes invalid environments, geometry and physics; no I01 status promotion

Independent review found no important defects in bounded scoring/terms/fixture. Actual UniLab manager execution pending isolated dependency setup; do not confuse passing source/contract math tests with simulator migration

UniLab dependency attempt deferred: official torch CPU175.8MiB wheel stream measured ~40KB/s; initial bundled Python download avoided by reusing existing runtime interpreter. Stopped only this turn's installer processes, no background job claimed. Real MetricsManager fixture NOT executed; receipt unilab-metrics-attempt.json explicitly records not_executed. Source contracts and89tests remain valid, no task status promotion

## 2026-11-12 UniLab manager 与场景声明续进

真实 MetricsManager CPU 验证通过，两个环境 100/20 分，部分重置互不干扰，receipt: unilab-metrics-contract.json
Ruling: 使用只读缓存 torch2.11 ROCm wheel 做 CPU 接口验证，避免重新下载；不是完整锁定环境，缓存清理后需重建。未改 Isaac 环境
新增 check_scene_declaration.py 是一次可重复的审计脚本，不是运行时 converter：四资产 SHA、四实体位姿与 SDK 静态能力检查。九个后端均拒绝 USD 多实体入口，保留原始错误
Ruling: collision=True 明确作为候选能力请求，未宣称忠实恢复源碰撞配置；质量/材质和完整场景缺项单列，防止配置丢失被当作成功迁移
下一步：核对支持的资产装配入口、源配置合并，再做真实加载/步进/视频

验证：真实 MetricsManager 重跑通过；uv pip check 无冲突；89 项回归测试通过。补充查询：九个后端在本次 MJCF/无 variant/无 kinematic 配置下声明 exact，详细限制保存到 scene receipt；尚未转换或加载任何 MJCF 资产
独立复核：review_scene_audit 核对真实 UniSim 2.7.1 API，独立重算 receipt 一致；未发现重要正确性问题。当前仍非可运行物理任务，无新增仿真视频
