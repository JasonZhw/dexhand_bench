> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# RoboDojo explanation and first UniLab contract

Explain the existing pinned source structure, then migrate one bounded part before importing full assets

1. Read original scene/task/seed/score code and UniLab manager contracts
2. Preserve insert_tubes scoring transitions and latched success over verified boolean predicates; test against actual upstream scoring class
3. Provide downstream UniLab metric terms (last reduction) and exercise the real manager with two synthetic environments and partial reset, in an isolated dependency environment
4. Document unresolved geometry, runtime failure, robot/action, scene and physics migration requirements; never mark I01 as simulated
5. Run regression tests and independent review

No source checkout changes; user will choose their own repository later. Existing Isaac environment stays untouched. A contract fixture is not a runnable task or a throughput benchmark
