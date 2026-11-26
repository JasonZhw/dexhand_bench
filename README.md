# dexhand_bench

灵巧操作 benchmark 的初步任务集合与 UniLab 接入工程

当前 21 个候选任务，覆盖抓取、搬运、推动、旋转工具操作、手内调整、插入。来源与改编关系见 `tasks/`，抓取容差、负载重心、抗转矩、接触切换、双手协同用于解释难度，目前不是实测难度排名

当前工作重点是完善每项任务的配置文件与测评标准。21 项条目已有来源、难度假设与可执行的初步判定规则；这些规则尚未全部接入真实仿真观测，不能等同于完成 UniLab 迁移。policy 复现与完整任务运行留待后续接入

首批 [五个基础抓取场景](docs/five-local-grasp-scenes.md) 已在本地接入 UniLab/MuJoCo：苹果、瓶子、电池、螺丝刀柄、锅柄，各有启动配置、真实状态评分和 [预览视频](docs/evidence/grasp-five/index.html)。五项默认机器人已替换为 X5 双臂＋左右 Allegro V3 灵巧手（每手四指、16 个可控关节），支持逐指接触记录和手部近景录像；默认试动仅检查关节和接触，未执行抓取

G01 苹果已有 [Wuji IK 抓取脚本](docs/apple-ik-grasp.md)：通过 UniLab/MuJoCo 抬起约 10.4 cm 并保持稳定，保存了 [视频、评分和接触记录](docs/evidence/apple-ik/index.html)。G02 瓶子也已接通 [侧面 IK 抓取](docs/bottle-ik-grasp.md)：抬高约 6.74 cm、末尾倾斜约 7.86°，通过现有评分，支持同一套物理记录并保存了 [视频](docs/evidence/bottle-ik/index.html)。这两项是固定初态的脚本基线，其他任务的抓取控制与学习策略仍待接入

已有五项 Gymnasium Robotics / Adroit 原环境试跑和 Bench2Dex 片段回放桥接。插管已接通 UniLab 物体场景、双臂控制、重置与抓取诊断；完整插入任务尚未完成

插管的物体场景现已通过 UniLab 后端工厂接入 MuJoCo：原始 USD 转为近似多凸块 MJCF，10 个孔口探针通过，两个环境各推进 500 步并验证独立重置，保存 2 秒视频。这是物体场景检查；另有下文的双臂控制与抓取诊断，完整任务仍未就绪。复现命令与验收边界见 [场景接入检查](docs/insert-tubes-scene.md)

> 本地五个抓取场景支持 Allegro / Wuji Hand 2.2 / Sharpa Wave，通过 `--robot` 切换。[模型来源、切换命令与视频](docs/wuji-sharpa-hands.md)

## 仓库与 UniLab 的关系

这是独立下游仓库，不是 GitHub fork。`third_party/UniLab` 以 Git submodule 固定到 `794c0bbd67771ac48407e084c0164c0e841bfd97`，上游版本 1.5.9、依赖 unisim-core 2.7.1。当前不修改上游源码，后续需要修改 UniLab 核心时再向上游提交补丁

本地 G01/G02 场景环境现由仓库根目录的 `pyproject.toml`、`uv.lock` 管理。`uv sync --locked` 建立 `.venv`，使用 CPU 版 PyTorch 执行 MuJoCo 仿真。G02 可通过 [UniLab Viser 实时查看](docs/bottle-ik-grasp.md#在本机启动)；旧 `.venv-scene` 暂留作结果对照

21 个任务现在有各自的 Hydra 启动配置，并使用 [统一任务入口与 UniLab 原生 Viser 播放](docs/unilab-task-owners.md)。其中 G01–G05 能在 UniLab 加载和短程测试，I01/I02 已接入 Wuji 插入场景诊断与真实评分输入读取。其余 14 项都能[试跑加载、动作和 reset](docs/unilab-proxy-task-status.md)：P01/I03/M02/M03 已[接入 RoboDojo 原物体网格](docs/source-object-asset-integration.md)，另 10 项仍为自建基础形状。策略、reset 与评分信号仍需逐项接通，不能把场景诊断当成原 benchmark 成绩

```bash
git clone --recurse-submodules https://github.com/JasonZhw/dexhand_bench.git
cd dexhand_bench
uv sync --locked
uv run --locked python -m dexbench list
uv run --locked python -m dexbench audit
```

## 本地任务配置与评分

21 项任务的 JSON 现均包含 `benchmark` 初稿：场景与初态说明、观测单位、成功/失败条件、评价指标、来源及待绑定项。参见 [任务总览](docs/task-evaluation-catalog.md) 与 [本地使用说明](docs/local-task-evaluation.md)

```bash
# 无需仿真或 GPU 依赖，使用系统 Python 即可
python3 -m dexbench.evaluation audit
python3 -m unittest discover -s tests -p test_evaluation.py
python3 -m dexbench.evaluation score --task H02 --trace examples/evaluation/H02-pass.json
```

examples/evaluation 中的示例全部为合成观测，仅检查判定逻辑。G01–G05 已有独立的 UniLab 实际状态绑定检查；源评分生命周期和策略运行仍需单独验证

## 初步任务配置

下面两条命令只解析配置，不安装或启动物理引擎

```bash
uv run --locked python integrations/unilab/configure.py task=insert_tubes/mujoco --cfg job --resolve
uv run --locked python integrations/unilab/configure.py task=insert_tubes/isaacsim --cfg job --resolve
```

`conf/unilab/task/insert_tubes/base.yaml` 定义共享任务协议，两个 owner YAML 选择后端及资产路线，`benchmark.yaml` 定义运行与物理观测需求。正常调用而不带 `--cfg` 会明确报错退出：这还不是可执行的 UniLab 环境，不能用于训练或汇报任务成绩

这些 YAML 是下游启动配置原型，不是可以直接传给上游 `train` 的配置。物理步长、动作映射、模型路径保留空值，必须从原任务核对，不能用猜测值掩盖缺口

## 物理测评接口

计划由统一执行流程采集动作、关节状态、物体位姿、仿真时间和任务阶段，再由独立评价器计算结果。接触力、穿透按后端能力提供，缺失不填零。保持同一任务协议，用不同后端 owner 比较结果；耗时测量要预热、设备同步，并区分渲染开销。不同引擎的差异不是物理误差，准确性需要实测或解析参照

21 项通用配置仍以需求声明为主；G01–G05 的本地启动配置已驱动 MuJoCo 执行，记录几何穿透和每步耗时，接触力、滑移仍为缺失。已有原环境运行器及接口在 `dexbench/`；完整迁移顺序是资产与碰撞验证 → 机器人及控制映射 → reset/观测 → 几何判据 → manager 环境注册 → 回放记录 → 第二后端对照

## 测试与原环境运行

```bash
uv run --locked python -m unittest discover -s tests
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
