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
