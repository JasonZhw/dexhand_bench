# 抓苹果任务：任务评分与物理记录

G01 已接入运行时接触记录。2026-11-19 的验证用 UniLab + UniSim MuJoCo 后端运行双 X5 / Wuji Hand 2.2 场景：手指试动，苹果在重力下落到桌面。这个动作没有执行抓取，因此任务评分失败，验证的是采集与评分的共同运行流程。2026-11-20 已增加 [IK 抓取脚本](apple-ik-grasp.md)，实际抓起并稳住苹果，同时沿用本文的记录接口

同一条运行命令同时输出任务结果和物理数据：

```bash
MUJOCO_GL=egl .venv-scene/bin/python integrations/unilab/run_grasp_scene.py \
  --task G01 --robot wuji --output runs/my-apple-physics --video
```

输出目录必须是新的目录。场景资产需先按 [本地场景说明](five-local-grasp-scenes.md) 和 [Wuji/Sharpa 接入说明](wuji-sharpa-hands.md) 准备

## 配置怎么控制

`profiles/local_grasp/G01.json` 中，`backend: mujoco` 选择当前唯一接通的任务后端；`runtime` 设置步长、步数、并行环境数和动作方式；`physics_recording` 控制采集：

```json
{
  "enabled": true,
  "contact_slots": 32,
  "warmup_steps": 25
}
```

默认每个接触组最多保存 32 个接触，预热 25 步后重新 reset，才开始记录任务。`--physics off` 可以关闭传感器和记录，保持同样的预热，做同动作的对照。G01 和 [G02 瓶子抓取](bottle-ik-grasp.md) 均已默认开启；配置中的后端名称还不能任意切换，因为资产和控制适配仍只验证了 MuJoCo

## 数据从哪里来

运行器在本次输出目录写一个 `runtime-scene.xml`，给原场景增加 MuJoCo contact 传感器，保持原来的模型、控制和任务标准。原场景不会被覆盖。之后通过公开的 `env.backend.bind_sensor_data()` 接口读取传感器，读取时不额外推进仿真，也不重新求解接触力

传感器分别观察苹果的全部接触、苹果与桌面、苹果与左手、苹果与右手。手部范围是该手掌及其子树。全部接触组已经包含其他组，不能将四组再相加。原来的姿态几何接触依然用于任务评分，物理记录不参与成功判断

| 输出 | 内容 |
|---|---|
| `evaluation.json` | 原任务标准的评分、失败原因和任务指标，每个环境一份结果 |
| `states.json` | 动作、关节状态、物体位姿和速度 |
| `physics.json` | 每步的接触法向力幅值总和、指定手接触力、穿透代理、耗时；同时保留离线几何穿透值 |
| `runtime-physics.json` | 接触组及每个接触的局部力/力矩、世界坐标接触点、接触坐标基、带符号距离和采样时刻 |
| `physics-summary.json` | 每个环境各接触组的采样峰值、缺失与溢出计数、步进耗时汇总 |
| `physics-capabilities.json` | 支持/不支持的信号、真实生效的求解器参数、质量和采样定义 |
| `sensor-manifest.json` / `receipt.json` | 传感器定义、原始与运行时场景哈希、软件版本、任务结果及重置验证 |
| `preview.mp4` / `hand.mp4` | 从这次真实状态记录渲染的全景和手部视频 |

`contact_force_n` 是苹果所有接触的法向力幅值之和，`hand_contact_force_n` 只取指定的右手。两者都不是物体净合力。如果手没有碰到苹果，手部接触力为 0；如果数据不支持或采集不完整，应记录 null，而不是 0

当前安装的 MuJoCo 3.11.0 contact 传感器不接受 `linvel` 数据字段，已用最小编译测试确认。因此 `slip_m_s` 为 null，原因是 `unsupported_contact_velocity_in_pinned_mujoco`。没有用物体速度冒充手指接触点滑移速度。固定槽位溢出时，该组力总和和最大穿透也记 null，同时保留已采集的接触和真实匹配数量

## 时间和物理含义

运行步长和控制周期均为 4 ms。当前 MuJoCo/mjbatch 的接触传感器保留刚完成的那一步求解数据，而关节和物体状态已推进到步末。记录明确区分 `solver_t_s` 与 `state_t_s`：例如第 10 步的求解时刻为 0.036 s，步末状态为 0.040 s。step 0 是 reset 后的 forward 样本，不计入运行峰值统计

穿透来自求解器的接触距离，不能直接等同于真实世界误差。力和穿透只是该场景、质量、碰撞体和参数下的模拟输出，物理准确性仍需解析解或真实测量参照

耗时按整个双环境的 `env.step` 计量，包含控制、UniLab 管理器和场景内传感器，排除记录器、离线几何探针、渲染、预热与 reset。这是诊断计时，不是纯物理求解器耗时，也没有完成多并发吞吐对比

## 给物理同事的扩展位置

`integrations/unilab/runtime_physics.py` 中，`ContactSource` 约定 `metadata()` 和 `snapshot()`；当前实现是 `MujocoContactSource`。后续后端需要将自己的实际接触输出转换成同样的接触组格式，并在 metadata 中声明能力、单位、坐标系和时间定义。当前 `snapshot()` 输出在本次调用内归调用方持有，不应在下次读取时悄悄改变

`PhysicsRecorder` 按环境调用已有的 `DiagnosticProbe` 协议实现 `ContactEnvelope`，保存结果；任务仍调用 `dexbench.evaluation.evaluate()`。因此扩展物理指标不必改任务成功条件。这里先接通了采集接口，没有把整个 UniLab 环境包装成旧版 Gym runner 的 `Backend`，也没有完成第二个引擎的资产/控制适配

2026-11-20 的 IK 抓取实验补上了手与苹果持续接触的片段，右手接触法向力幅值总和峰值为 17.40 N，详情和视频见 [抓取实验记录](apple-ik-grasp.md)。这个数值是仿真输出，仍需要真实测量参照才能讨论力的准确性

测试依据包括：0.2 kg 球体静止支撑力接近 1.962 N、自由落体无接触、手/桌面分组隔离、槽位溢出、增加传感器不改变动力学，以及真实 UniLab 苹果场景记录开关前后轨迹与评分一致

传感器定义参考 [MuJoCo contact sensor 文档](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#sensor-contact)，实际字段以本地 3.11.0 的编译结果为准。最新版文档新增的字段不代表安装版本已经支持
