# 本地写任务配置与评价指标

当前交付是 21 项任务的初步配置和可在 CPU 上检查的评分逻辑。每项继续使用 `tasks/<类别>/<ID>.json`，新增 `benchmark` 区域；原来的来源、原任务成功条件和资产清单仍保留。不要复制一套新的任务身份或覆盖原始评分说明

配置参考固定版本 RoboDojo 的终态几何判据、Adroit/Farama 的目标距离与姿态判据，以及 Bench2Dex 的任务分解。抓取子任务、扳手功能代理和水龙头独立周期属于我们的改编。阈值来源与未完成的资产绑定均在各配置中标明

## 在本地开始

在仓库根目录执行。下面的评分工具只用 Python 标准库，不需要安装 Isaac Sim、UniLab、MuJoCo、Torch 或 CUDA

```bash
python3 -m dexbench.evaluation audit
python3 -m unittest discover -s tests -p test_evaluation.py

# 合成的“抬起并保持”观测，通过初拟判据
python3 -m dexbench.evaluation score --task G01 --trace examples/evaluation/G01-hold-pass.json
# 抬起后掉落：失败
python3 -m dexbench.evaluation score --task G01 --trace examples/evaluation/G01-drop-fail.json
# 插管缺少一个必要几何观测：无法判定，success 为 null
python3 -m dexbench.evaluation score --task I01 --trace examples/evaluation/I01-missing-unknown.json
```

`examples/evaluation/` 的八份输入都明确标记为 `synthetic`。它们是测试裁判的样例，不是策略轨迹、仿真记录或任务成功视频。`score` 正常处理一个失败任务也返回进程退出码 0；配置或输入格式有错才返回 2。应读取结果 JSON 的 `success`，不能把命令退出成功当成任务成功

## 每个任务改哪里

| 区域 | 写什么 | 当前状态 |
|---|---|---|
| `scene` | 对象、资产清单、原机器人、初态描述和 layout 引用 | 所有任务有说明；未核对的数值位姿、目标手型保留 null |
| `provenance` | 原仓库、固定版本、评分代码、哪些规则是自拟 | 原规则与改编协议分开，不把自拟阈值写成官方值 |
| `signals` | 评分需要的每个数值/布尔观测、单位、计算含义 | 带单位与定义；尚未新增通用 UniLab 观测适配器 |
| `initial` | 首帧必须满足的条件，可为空 | 抓取高度增量为零；水龙头初始关闭等 |
| `success` | 终帧、连续保持或有序事件规则 | 标准库评价器可执行 |
| `failures` | 一旦发生就判失败的事件 | 仅独立抓取初稿增加掉落条件；未擅改原环境失败规则 |
| `stages` | 终帧各部分条件是否满足 | 诊断进度，不是 RoboDojo 原 transition 分数 |
| `metrics` | 最后一帧、全程最大/最小值 | 必要观测与仅用于诊断的观测分开 |
| `episode` | 时间预算、允许的采样间隔、原步数参考 | 60 s 和 0.1 s 是初拟测试预算；原 step_lim 不是秒 |
| `physics_observation_requests` | 接触力、穿透、滑移、步进耗时的采集需求 | 接口预留，未采集是 null，不是零 |
| `readiness` | 是否已绑定几何、是否可启动场景、适配器是否核验 | 本轮新增通用配置全部仍是待绑定状态 |

已有特定 I01 诊断程序的可运行状态不等于这套 21 项通用协议已经绑定后端。配置里的 `scene_launch_ready=false` 指这一层的完整任务启动能力

例如抓取的规则可写为：

```json
{
  "mode": "hold",
  "hold_s": 1.0,
  "rule": {"all": [
    {"signal": "height_gain_m", "op": "ge", "value": 0.05},
    {"signal": "speed_m_s", "op": "le", "value": 0.05},
    {"signal": "hand_contact", "op": "eq", "value": true}
  ]}
}
```

它要求物体相对初态升高至少 5 cm、速度不超过 5 cm/s，并保持手与物接触，持续 1 s。这三个阈值来自我们的初稿，需要实际试跑校准。工具柄和苹果梗还要求指定接触区域，区域 mesh 标签没有绑定前不能把它直接写成 true

## 评分输入、时间与缺失值

输入包含 `task_id`、`protocol_version`、`kind`、`complete` 与逐帧 `t_s/signals`。时间从本 episode 的 0 开始，严格增加；间隔不能超过配置的 `max_sample_gap_s`，防止只用两个相隔很远的状态声称“持续保持”。保持时长依赖采样验证，仍不能证明采样之间没有瞬时异常

- `final`：判断最后一帧；之前曾满足但最终失去目标，不能判成功
- `hold`：终帧所在连续满足区间达到保持时长；中途不满足会重新计时
- `sequence`：按顺序检测事件，每帧最多推进一个阶段，且终帧仍满足最终条件。例如先开启水龙头，再关闭

初始条件不符合协议时返回 `success=null` 与 `invalid_initial_state`。必要成功/失败/顺序观测要求全程完整，缺失时保守地返回 unknown，除非已观察到明确的硬失败。仅用于指标的可选信号缺失不影响任务判断；对应指标保留 null。`complete=false` 不能凭最后一个成功画面声称完成任务

输出区分 `success=true/false/null`，同时包含完成时间、失败原因、首次硬失败时间、缺失观测、终帧阶段状态及各指标。阶段比例只是便于排查的终态条件比例，`official_source_score` 始终为 null，本轮没有实现源框架累计得分的完整生命周期

多 episode 的成功率、置信区间和引擎排名留待真实运行后再汇总。unknown 和启动失败必须单独计数，不能丢弃后只统计成功样本

## 接入 UniLab 时补什么

运行配置负责后端、设备、并行数和是否渲染；任务 JSON 负责任务目标与评分。选后端不应暗中改变任务阈值或动作语义。policy 服务与这套轻量评价器分开部署

后端适配器需要把真实状态转换成 `signals`。例如读取管子和管架的位姿、原 bbox，按源 helper 算 `tube0_depth_m`、`tube0_inside`、`tube0_upright_deg`。仅仅传入一个手工写的 true 不代表几何适配完成。接触力和穿透继续独立记录，不因为末端条件通过就认定物理准确

优先补 H02、P02 的数值观测绑定，再用已有 I01 几何接口做与源判据的逐项对照。复杂功能点任务需要资产标签、源投影几何和机器人 reset 参照；这些暂时不装成“能直接启动”的假接口。初稿任务总览见 [21 项配置与评分表](task-evaluation-catalog.md)
