> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# 运行结果协议 v0.2

当前实现面向单环境原任务试跑；原生GPU批量协议仍是设计稿。运行目录必须新建，不覆盖旧证据

manifest.json保存任务、seed、预算、预热、控制方式、记录模式、profile与源码哈希，以及后端版本、最终参数、原参数、物理/控制周期、原环境时限、资产哈希、关节/刚体/执行器映射和能力声明

initial_state.json与initial_observation.json保存真实reset后状态和原观察（包含原任务目标）。actions.json保存实际请求的动作序列。它们可支持固定种子重放和检查，不是完整求解器checkpoint，不声称可以精确恢复任意中间帧

诊断模式steps.jsonl按控制步保存requested_action、限幅后applied_action、实际actuator_control、原观察/奖励/success/终止信息，以及qpos/qvel、刚体世界位置与wxyz姿态、执行器力和可选接触。MuJoCo的广义关节坐标含不同单位，不能直接混成一个统一物理误差

接触为控制步末采样：几何对、世界接触点、接触坐标基、带符号距离、局部力/力矩。关闭接触采样时为null；未来不支持的后端也必须为null并写原因。当前没有任务专用滑移、阶段成功和真实测量误差计算器

summary.json分开记录：

- execution_completed：运行程序是否正常结束
- task_outcome：success / failure / unknown
- final_source_success：最后一个采样时刻的原成功布尔值
- any_source_success：过程中是否曾满足原判据
- termination_reason：原终止、原时限、步数预算、动作序列用完或异常
- diagnostic_probes：只读观察器的汇总，不能自动覆盖任务分数
- timing：environment.step与同步耗时，以及包括记录的运行总耗时

当前成功协议是source-final-predicate-v1：终帧原判据true记success；原终止/原时限时false记failure；提前预算结束而false记unknown。它是明确的本工程采样协议，不自动等同于原论文的多episode统计或连续成功步数标准

吞吐模式不存逐步状态，不允许接触观察器与视频。单步计时仍包含原环境控制、观察和评分计算，不是纯求解器内核耗时。batch为独立CPU进程，另报含进程创建与加载的端到端时间

比较器检查任务、seed、原环境、控制周期、资产、动作和采样时刻；同时标记实际reset状态是否一致。初态不一致时，后续差异不能当作固定初态的单变量求解器实验。即使完全一致，差异也只是仿真一致性，需要实测或解析参照才能讨论准确性
