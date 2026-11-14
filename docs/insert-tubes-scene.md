# 插管物体场景的接入检查

这是完整任务前的资产与物理场景检查，不包含机器人、策略或插管成功率。任务配置的 ready 仍为 false

先用资产清单恢复 I01 的四个原始文件；转换脚本会校验 SHA256。USD 使用米和 Z 向上，碰撞网格在资产坐标中导出，管架的 SDF 碰撞由 CoACD 多凸块近似。三根管和固定管架沿用固定布局位姿，桌面使用布局中的盒体尺寸与位姿。质量来自布局，惯性按转换后的体积估算；接触材质、凸块重叠与摩擦组合尚不等价于源 PhysX 场景

```bash
# 从仓库根目录运行；环境独立于 Bench2Dex / Isaac
uv venv .venv-assets --python 3.10
uv pip install --python .venv-assets/bin/python -r requirements/scene-assets.txt
uv run --no-project --python .venv-assets/bin/python python -m dexbench assets --manifest asset_manifests/I01-robodojo.json --fetch
uv run --no-project --python .venv-assets/bin/python python integrations/unilab/convert_insert_tubes.py --output assets/generated/I01-convex

uv venv .venv-scene --python 3.10
uv pip install --python .venv-scene/bin/python -r requirements/scene-runtime.txt
uv run --no-project --python .venv-scene/bin/python python integrations/unilab/check_rack_apertures.py --scene assets/generated/I01-convex/scene.xml
MUJOCO_GL=egl uv run --no-project --python .venv-scene/bin/python python integrations/unilab/check_object_scene.py --scene assets/generated/I01-convex/scene.xml --output runs/I01-object-scene --steps 500 --video
```

视频需要系统 ffmpeg。无渲染机器可以去掉 --video，运行状态与重置检查不依赖渲染。输出目录必须不存在，以免覆盖既有证据

验收分三层：

1. conversion.json：原资产哈希、网格范围、凸分解设置及局限
2. apertures.json：在源元数据的孔口中心放置半径 3 mm 的球，检查是否穿入实体；孔间板面应产生碰撞。这只检查孔口，不能证明整根管沿插入路径都不受阻
3. receipt.json / states.jsonl：通过 UniLab 后端工厂加载两个 MuJoCo 环境，记录有限状态，并检查只重置环境 0 不影响环境 1。不是 Manager-Based 任务环境，也没有任务成功判定

视频由记录的环境 0 状态驱动独立渲染，不增加物理步。每 10 步一帧，仿真步长 0.004 s，25 fps；首帧对应 0.04 s。渲染期间耗时不能用于无渲染吞吐测评

完整任务还需要原机器人或明确的目标灵巧手、关节及驱动映射、初始化与回位、实际几何判据、动作和观察 manager、终止条件，以及失败状态处理。不能因为物体场景能运行就把 I01 标记为已完成迁移

## 2026-11-14 实际验证

本地 Linux / Python 3.10.12，UniLab 固定子模块，UniSim 2.7.1、MuJoCo 3.11.0、Torch 2.11.0+rocm7.2。MuJoCo 在 CPU 上执行；本机 Torch 复用了已有只读安装缓存，其他机器按上面的依赖文件安装自己的 Torch

- 转换生成管架 141 个凸块、试管 3 个凸块；使用原碰撞形状显示，不保留原材质外观
- 10 个孔口中心的 3 mm 球探针没有穿入实体，孔间板面探针有碰撞；不代表整根试管已通过插入路径验证
- 经 `create_backend → materialize → step_tensor → set_state_tensor → close`，2 个环境各运行 500 步、模拟 2 秒，状态有限；重置环境 0 后环境 1 的 qpos 和 qvel 均未变化
- 视频 `runs/I01-object-scene-materialized/scene.mp4`：640×480、25 fps、50 帧、2 秒。只展示物体落稳，不展示机器人插管
- 回归检查 92 项，85 项通过，7 项因未安装原环境仿真依赖跳过

验收记录保存在 [evidence/I01-object-scene](evidence/I01-object-scene)。视频、生成网格和逐步状态留在本地 runs/assets 目录，不提交 Git

接入中修复了两处实际问题：USD 25.11 与 CoACD 1.0.7 在本环境存在导入顺序冲突，先导入 USD 后正常；后端工厂构造完成后仍需要调用公开的 `materialize()`，之后才能推进仿真。初次失败目录 `runs/I01-object-scene` 没有成功记录，不作为验收结果

下一步接机器人与控制时保持这条场景验证独立。先确定关节和驱动映射、恢复固定初态，再把真实物体位姿接到插管几何判据与 MetricsManager；之后才注册完整 Manager-Based 环境并开放任务启动配置。物理接触数据仍按后端实际能力读取，不能用这次无机器人运行计算任务成功率或物理准确性

## Manager-Based 运行层已接通

`conf/unilab/scene_diagnostic.yaml` 现在可以启动真正的 UniLab `ManagerBasedRlEnv`，通过 MuJoCo 后端运行同一套物体资产。配置统一声明场景、时间步、观察、重置事件、超时条件与指标。它是无机器人的独立诊断配置，完整 I01 任务仍保持 `ready: false`

```bash
# 完成上面的资产转换和 scene-runtime 依赖安装后，从仓库根目录执行
uv run --no-project --python .venv-scene/bin/python python integrations/unilab/run_scene_diagnostic.py output=runs/I01-manager-check steps=60

# 几何公式的独立参考算法对照及全套回归
uv pip install --python .venv-scene/bin/python -r requirements/scene-test.txt
uv run --no-project --python .venv-scene/bin/python python -m unittest discover -s tests
```

每个环境观察为 37 维：三根试管和管架各 7 维位置/四元数，加上每根管的深度、架内、竖直三个布尔条件。这三项条件来自固定版本 RoboDojo 的 `insert_tubes`、`func_parser.py` 和 `calc_polygon`，移植为批量张量计算，使用原元数据包围盒；128 组随机位置和旋转已与 Shapely / transforms3d 参考算法对照一致

源规则的“架内”指管心落在管架包围盒的 XY 凸包内且高于盒底，不能证明试管位于某个真实孔内；深度是架的最高点减管的最低点大于 4.5 cm，竖直是试管局部 Y 轴与世界 Z 轴夹角小于 30°。元数据包围盒也不能当作碰撞表面。我们保留源规则用于对照，后续孔位、穿透与接触诊断需要单独添加。完整源任务还检查松手与机器人回位，目前没有伪造这两个条件，因此 `source_geometry_count` 只表示满足几何条件的试管数量，不是任务成功率

最新验收为 2 个环境、各 60 步，物理步长和控制步长均为 0.004 s，回合超时为 20 步。第 5 步验证物体确实运动后单独重置环境 0；其所有物体位姿和速度恢复，环境 1 保持不变。环境 1 在第 20、40、60 步超时，环境 0 在第 25、45 步超时。每次重置前，指标汇总都与当时真实观察核对一致。运行中几何数量为 0，符合没有机器人执行插入的情况

本次发现并修复了配置遗漏：UniLab 的 `reset()` 不会在没有重置事件时自动恢复物理状态，必须显式配置 `reset_scene_to_default`。验收因此不能只看计数器归零，也不能在物体尚未运动时检查位置。最终证据见 [I01-manager/receipt.json](evidence/I01-manager/receipt.json) 与 [运行配置](evidence/I01-manager/config.json)。完整状态保存在本地 `runs/I01-manager-final/states.json`；旧的弱检查结果不作为本次证据

全套 99 项检查中 92 项通过，7 项因缺少原环境依赖跳过。本次没有新增机器人视频，之前的物体落稳视频仍只代表资产场景检查

下一步是接入机械臂/手的实体、关节与控制映射，恢复可重复初态，再用动作推进完整插管流程。配置目前仅开放已验证的 MuJoCo 路径；其他后端需要通过资产、状态读取、控制和重置能力检查后才能切换，不能只改一个字符串就声称完成跨引擎迁移。接触力、穿透等物理指标将通过独立观察/指标项扩展，不支持的字段明确记为缺失
