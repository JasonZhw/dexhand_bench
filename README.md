# dexhand_bench

灵巧操作 benchmark 的初步任务集合与 UniLab 接入工程

当前 21 个候选任务，覆盖抓取、搬运、推动、旋转工具操作、手内调整、插入。来源与改编关系见 `tasks/`，抓取容差、负载重心、抗转矩、接触切换、双手协同用于解释难度，目前不是实测难度排名

当前工作重点是完善每项任务的配置文件与测评标准。21 项条目已有来源、难度假设与可执行的初步判定规则；这些规则尚未全部接入真实仿真观测，不能等同于完成 UniLab 迁移。policy 复现与完整任务运行留待后续接入

已有五项 Gymnasium Robotics / Adroit 原环境试跑和 Bench2Dex 片段回放桥接。插管已接通 UniLab 物体场景、双臂控制、重置与抓取诊断；完整插入任务尚未完成

插管的物体场景现已通过 UniLab 后端工厂接入 MuJoCo：原始 USD 转为近似多凸块 MJCF，10 个孔口探针通过，两个环境各推进 500 步并验证独立重置，保存 2 秒视频。这是物体场景检查；另有下文的双臂控制与抓取诊断，完整任务仍未就绪。复现命令与验收边界见 [场景接入检查](docs/insert-tubes-scene.md)

## 仓库与 UniLab 的关系

这是独立下游仓库，不是 GitHub fork。`third_party/UniLab` 以 Git submodule 固定到 `794c0bbd67771ac48407e084c0164c0e841bfd97`，上游版本 1.5.9、依赖 unisim-core 2.7.1。当前不修改上游源码，后续需要修改 UniLab 核心时再向上游提交补丁

```bash
git clone --recurse-submodules https://github.com/JasonZhw/dexhand_bench.git
cd dexhand_bench
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python -r requirements/config.txt
uv run --no-project --python .venv/bin/python python -m dexbench list
uv run --no-project --python .venv/bin/python python -m dexbench audit
```

## 本地任务配置与评分

21 项任务的 JSON 现均包含 `benchmark` 初稿：场景与初态说明、观测单位、成功/失败条件、评价指标、来源及待绑定项。参见 [任务总览](docs/task-evaluation-catalog.md) 与 [本地使用说明](docs/local-task-evaluation.md)

```bash
# 无需仿真或 GPU 依赖，使用系统 Python 即可
python3 -m dexbench.evaluation audit
python3 -m unittest discover -s tests -p test_evaluation.py
python3 -m dexbench.evaluation score --task H02 --trace examples/evaluation/H02-pass.json
```

示例全部为合成观测，仅检查判定逻辑。真实 UniLab 观测绑定、源评分生命周期和策略运行仍需单独验证

## 初步任务配置

下面两条命令只解析配置，不安装或启动物理引擎

```bash
uv run --no-project --python .venv/bin/python python integrations/unilab/configure.py task=insert_tubes/mujoco --cfg job --resolve
uv run --no-project --python .venv/bin/python python integrations/unilab/configure.py task=insert_tubes/isaacsim --cfg job --resolve
```

`conf/unilab/task/insert_tubes/base.yaml` 定义共享任务协议，两个 owner YAML 选择后端及资产路线，`benchmark.yaml` 定义运行与物理观测需求。正常调用而不带 `--cfg` 会明确报错退出：这还不是可执行的 UniLab 环境，不能用于训练或汇报任务成绩

这些 YAML 是下游启动配置原型，不是可以直接传给上游 `train` 的配置。物理步长、动作映射、模型路径保留空值，必须从原任务核对，不能用猜测值掩盖缺口

## 物理测评接口

计划由统一执行流程采集动作、关节状态、物体位姿、仿真时间和任务阶段，再由独立评价器计算结果。接触力、穿透按后端能力提供，缺失不填零。保持同一任务协议，用不同后端 owner 比较结果；耗时测量要预热、设备同步，并区分渲染开销。不同引擎的差异不是物理误差，准确性需要实测或解析参照

当前配置只是这些需求的声明，尚未驱动物理执行。已有原环境运行器及接口在 `dexbench/`；完整迁移顺序是资产与碰撞验证 → 机器人及控制映射 → reset/观测 → 几何判据 → manager 环境注册 → 回放记录 → 第二后端对照

## 测试与原环境运行

```bash
uv run --no-project --python .venv/bin/python python -m unittest discover -s tests
# 原环境另建环境，避免与 UniLab/Isaac 的依赖混装
uv venv .venv-source --python 3.11
uv pip install --python .venv-source/bin/python -r requirements/simulation.txt
uv run --no-project --python .venv-source/bin/python python -m dexbench run --profile profiles/H01-source.json --output runs/h01-first --seed 0 --steps 100 --action zero
```

若未安装原仿真依赖，相应测试会跳过。原始资产、HDF5、虚拟环境和完整运行产物不入 Git；少量精选演示视频和截图保存在 docs/evidence，资产按 `asset_manifests/` 恢复。第三方代码与资产遵循各自许可；Bench2Dex 回放代码的 MIT 许可在对应目录。本项目新增代码的开源许可尚未指定

UniLab 的物体场景现在也支持 [Manager-Based 统一运行配置](conf/unilab/scene_diagnostic.yaml)：真实位姿观察、源插管几何判据、超时、指标汇总与选定环境物理重置已验收。参见 [运行步骤与范围说明](docs/insert-tubes-scene.md#manager-based-运行层已接通)。完整机器人插管任务尚未就绪

双臂 ARX X5 的 [控制接入检查](docs/x5-control.md) 已运行：统一配置下发双臂和夹爪动作、记录真实关节状态、验证并行环境独立重置并保存视频。它仍是控制诊断，不是抓取或插管成功结果

新增 [抓取与接触诊断](docs/x5-pick.md)：通过 UniLab 执行接近、闭合、抬升和保持，并保存两组夹爪目标对照。较紧目标可抬升约 85 mm，但指尖穿透峰值约 10 mm；较宽目标未抬起。完整插管任务仍未完成

## 已有材料索引

- [21 项任务配置](tasks/)：抓取 6 项，其余五类各 3 项
- [任务与评分约定](docs/task-contract.md)、[结果字段](docs/result-contract.md)、[物理测评接口设计](docs/physics-extension.md)：历史设计，待按当前任务逐项完善
- [来源调研](docs/research/)与[资产缺口](docs/asset-gaps.md)：保留原核对记录及日期
- [历史进度与试跑视频](docs/progress.md)：2026-11-11 至 12 日，零动作试跑不代表任务成功
- [UniLab 抓取诊断](docs/x5-pick.md)：含较紧/较宽夹持对照与接触穿透限制
- [历史文件补入清单](docs/history-import.json)：保留来源哈希，未覆盖仓库中的较新实现

早期报告脚本保存在 scripts/；build_meeting_report.py、plot_replay_diagnostics.py 依赖未入库的历史 runs，check_robodojo_score_parity.py 依赖旧工作区源码路径。这次仅归档，不承诺克隆后可直接重建历史报告
