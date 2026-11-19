> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# 给物理测评接入者的说明

任务负责“做什么、初态是什么、怎样判断结果”，后端负责“动作怎样推进物理”。当前接通的是原版 Gymnasium Robotics + MuJoCo CPU 环境；其它后端必须新增适配器，没有静默替代

`dexbench/runner.py` 的 Backend 协议暴露 metadata、reset、action、step、snapshot、synchronize、close。动作策略目前支持零动作、固定种子的随机动作、JSON动作序列。零动作只是归一化控制输入为0，不保证关节保持当前位置，也不是训练好的策略

`step` 返回原评分、终止/超时、请求经限幅后的动作和实际执行器控制值。`snapshot` 返回关节位置/速度、物体与机器人各刚体位姿、可选接触数据。关节坐标含转动和平动，不把混合单位的qpos直接汇总成一个误差分数。当前快照不是完整求解器checkpoint，不能宣称从中完全恢复任意中间状态

MuJoCo profile允许显式设置 timestep、iterations、tolerance、solver、integrator；不认识的参数会报错。manifest同时记录原参数和修改后的参数。控制周期随原环境frame_skip/substeps与timestep共同变化，改变步长时需重新核对动作采样频率，不能直接比较相同步号

接触记录包含接触几何ID、世界坐标点、接触局部坐标基、带符号距离、局部六维力/矩。它是每个控制步结束时的求解器采样，不是控制周期内所有接触的极值。normal_force_sum_N只是法向力幅值的总和，不是物体净合力。负距离的绝对值作为接触几何穿透代理，也不能直接当真实误差

不请求接触时数据为null并注明not_requested；未来后端不支持时应填null并注明unsupported，不能填零。能力通过metadata.capabilities声明；针对某个后端的能力缺口，应保留缺口，不能让评测层去猜私有API

诊断模式记录逐步状态，可用compare命令比较同任务、同种子、相同动作和相同控制时刻的轨迹。结果命名为simulation_consistency_not_physical_accuracy：两个引擎接近，也可能一起偏离真实测量。真实精度仍需解析解或实测参照

吞吐模式省略逐步状态/接触/视频。单环境计时只包围environment.step和synchronize，仍包含环境的控制、观察与评分计算，不是纯求解器耗时。batch采用独立CPU进程，报告包括进程启动、加载和写盘的端到端时间；不是GPU原生并行环境，不能据此评价UniLab GPU扩展效率

后续适配器应保持以下责任边界：

| 接入位置 | 物理同事可扩展什么 | 必须保留的证据 |
|---|---|---|
| metadata / capability | 后端参数、设备、接触与力读取能力 | 实际版本、最终生效参数、坐标与单位 |
| reset | 同种子初态、扰动设置 | 重置后的真实状态与目标 |
| step | 后端推进、控制模式 | requested/applied动作、实际控制周期 |
| snapshot | 接触、力、穿透、求解器诊断 | null原因、采样频率，不能混用力与冲量 |
| synchronize | GPU完成同步 | 计时边界与同步方式 |
| offline comparison | 滑移、跌落、阶段失败时间 | 使用的物体/手部参考坐标系、阈值与原始轨迹 |

第一版没有通过比较器自动判断“哪个引擎错了”，也没有实现真实测量数据接口或跨后端资产转换

自定义诊断可实现`dexbench/probes.py`中的DiagnosticProbe协议：reset(initial_state)、observe(state, transition)、result()，以及name和requires_contacts属性；传给run_episode(probes=[...])。观察器只读取快照，不参与动作和成功判定。当前ContactEnvelope保存已采样接触的最大穿透代理和最大法向力幅值总和；未采集数据保持null
