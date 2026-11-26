# 21 个任务如何接入 UniLab

UniLab 原仓库用 `uv` 管理 Python 环境。它的 MuJoCo 交互展示用浏览器 Viser：仿真后端负责计算状态，`unilab.visualization.viser_playback.run_viser_playback_from_cfg` 读取状态并显示。这里的 `play_task.py` 调用的正是这个入口，浏览器内可暂停、单步和调速。它不另建一个负责推进物理的 MuJoCo 环境。UniLab 原仓库的示例是 `uv run eval --algo sac --task g1_walk_flat --sim mujoco --render-mode viser`；我们目前是脚本控制的灵巧手任务，没有训练好的 UniLab policy，所以采用同一个 Viser 播放入口加自己的任务 session，不能直接冒充原仓库的 `eval` 任务注册。

每个任务有两个文件：`tasks/<类别>/<ID>.json` 定义来源、物体、信号、成功条件、失败条件和指标；`conf/unilab/task_catalog/<ID>.yaml` 指向该评分合同，并声明是否已有 UniLab 场景、机器人、控制器和后端。`conf/unilab/dex_task.yaml` 是统一 Hydra 入口，设置 Viser 展示与物理记录的共同约定。切换 YAML 中的后端字段并不会自动得到新后端：必须先接通同一资产、关节映射、reset、动作和观测，并核对物理能力，否则启动器拒绝运行。

当前 21 份配置都可以组合并检查评分合同。其中 G01–G05 已绑定本地双 X5 灵巧手 MuJoCo 场景，可在 UniLab `ManagerBasedRlEnv` 中加载、reset、step 和生成评分记录。G01/G02 有固定场景的 Wuji IK 尝试；G03–G05 只有 Allegro 关节探针，不是能完成任务的策略。I01 和 I02 已绑定双 X5/Wuji 与 RoboDojo 物体，在 UniLab 中通过两环境短程试跑、独立 reset 和各自评分输入读取；两者只有关节探针，没有插入策略，成功结果为未验证。I02 的固定初态是自选诊断布局。剩余 14 项仍为 `unbound`。原环境中单独跑过的任务，也不等于已经迁移到 UniLab。

本次验证的 [审计结果与运行回执](evidence/unilab-task-owners-20261126/audit.json) 保存在 `docs/evidence/unilab-task-owners-20261126/`。五个场景各走了两个物理步并通过独立 reset 检查；G01/G02 还执行了完整 IK 基线，两者的环境 0 达到当前本地草案的成功条件，环境 1 是静止对照。这些结果只证明本地固定初态下的脚本与评分链路，不代表原 Bench2Dex 成绩或其他物理后端的准确性。

在仓库根目录运行：

```bash
uv sync --locked
uv run --locked python integrations/unilab/task_owner.py audit
uv run --locked python integrations/unilab/task_owner.py smoke --task G03 --output runs/G03-smoke
uv run --locked python integrations/unilab/task_owner.py run --task G02 --output runs/G02-full
uv run --locked python integrations/unilab/play_task.py --task G02 --port 8080
```

最后一条会打印 `http://localhost:8080`，在浏览器打开即可看 UniLab Viser。`smoke` 默认只走两个物理步，用于验证加载、reset、动作下发、状态读取和评分链路；短试跑失败是预期的，因为它不会完成抓取。`run` 执行配置里的完整脚本，并保存 `receipt.json`、`evaluation.json`、轨迹与物理记录。G03–G05 的 `run` 仍只是在执行关节探针。`play_task.py` 持续循环演示，不负责写评分报告；要获得可复查的评测结果，请用 `run`。

下一步应按任务类别增加 UniLab 场景适配器：先选一个 RoboDojo 任务，固定资产版本并转换可加载的碰撞几何；确认灵巧手关节和动作约定；写初态与 reset；把 JSON 中每个评分信号绑定到实际观测；最后用同一任务配置做批量数、耗时、接触力、滑移等能力检测。缺测的物理量标成 unavailable，不用零代替。这样任务成绩与物理诊断才能来自同一次 rollout。

I01/I02 的实际启动命令是：

```bash
uv run --locked python integrations/unilab/task_owner.py audit --task I01
uv run --locked python integrations/unilab/task_owner.py smoke --task I01 --output runs/I01-smoke --steps 2
uv run --locked python integrations/unilab/task_owner.py smoke --task I02 --output runs/I02-smoke --steps 2
uv run --locked python integrations/unilab/play_task.py --task I02 --port 8080
```

I01 的[运行回执](evidence/I01-wuji-20261126/receipt.json)、I02 的[运行回执](evidence/I02-wuji-20261126/receipt.json)与[其余任务的逐项迁移状态](unilab-remaining-16.md)可供复查。`play_task.py` 对 I01/I02 也走 UniLab 原生 Viser 播放入口，但只循环展示关节探针，评分请用 `task_owner.py smoke`。下载已固定哈希的源物体可用 `uv run --locked python integrations/unilab/task_assets.py fetch --task I02`，但下载完成仅代表资产齐备，不代表场景或控制器已接通。
