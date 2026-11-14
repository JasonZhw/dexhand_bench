# dexhand_bench

灵巧操作 benchmark 的初步任务集合与 UniLab 接入工程

当前 21 个候选任务，覆盖抓取、搬运、推动、旋转工具操作、手内调整、插入。来源与改编关系见 `tasks/`，抓取容差、负载重心、抗转矩、接触切换、双手协同用于解释难度，目前不是实测难度排名

已有五项 Gymnasium Robotics / Adroit 原环境试跑和 Bench2Dex 片段回放桥接。原环境试跑不等于任务成功，也不等于迁入 UniLab。插管评分已有真实 UniLab MetricsManager 的接口验证；完整插管场景、控制和重置仍未接通

插管的物体场景现已通过 UniLab 后端工厂接入 MuJoCo：原始 USD 转为近似多凸块 MJCF，10 个孔口探针通过，两个环境各推进 500 步并验证独立重置，保存 2 秒视频。当前不含机器人，完整任务仍未就绪。复现命令与验收边界见 [场景接入检查](docs/insert-tubes-scene.md)

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

若未安装原仿真依赖，相应测试会跳过。资产、视频、HDF5、虚拟环境和运行产物不入 Git，资产按 `asset_manifests/` 恢复。第三方代码与资产遵循各自许可；Bench2Dex 回放代码的 MIT 许可在对应目录。本项目新增代码的开源许可尚未指定
