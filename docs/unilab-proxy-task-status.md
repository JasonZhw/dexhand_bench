# 后续任务的 UniLab 场景诊断

这 14 项此前只有任务配置与评分合同，没有能在 UniLab 启动的物体场景。现在每项都绑定了双 X5/Wuji 灵巧手、MuJoCo、固定初态、关节探针和 UniLab `ManagerBasedRlEnv`。其中 7 项仍按代理场景管理：P02、H01、H02、H03 已分别从 [Fetch Slide](fetch-slide-parameter-adaptation.md)、[Shadow Hand 方块](hand-block-parameter-adaptation.md)、[Adroit Pen](adroit-pen-parameter-adaptation.md)、[Shadow Hand 蛋形物体](shadow-egg-parameter-adaptation.md)的原 MJCF 核对物体参数，R03 已[迁入 Adroit 门机构](adroit-door-mechanism-adaptation.md)；P01、I03、M01、M02、M03 和 P03 已[接入 RoboDojo 原物体资产](source-object-asset-integration.md)，R02 已[接入 Bench2Dex 水龙头机构](bench2dex-faucet-adaptation.md)。每个任务都在两个并行环境中完成了加载、两步推进、物体与手腕状态读取、独立 reset 检查。浏览器展示走 UniLab 原生 Viser。

**这 14 项仍都是场景诊断，不是原任务复现。** 21 份任务 JSON 中的成功、失败和指标规则仍保留；这 14 项还没有与源任务等价的 reset 和策略。R02 已读取水龙头关节角度，R03 已读取门铰链角度，其余评分信号仍为 `null`；所有短探针的成功结果也是 `null`。接触力未采集，同样记录为不可用。P01/I03/M01/M02/M03/P03/R02 的源网格已接入，但接触模型经过近似转换，还需要验证物理等价性。

![14 项自建基础形状的诊断初态总览](evidence/task-proxies-20261126/overview.png)

这张图是旧版 14 项自建几何初态总览；P01/I03/M01/M02/M03/P03 的现状请看[源物体资产截图](source-object-asset-integration.md)，R02 看[水龙头截图与视频](bench2dex-faucet-adaptation.md)，H01/H02/H03/P02/R03 的现状见各自的源 MJCF 说明。图片只供检查场景，不表示完成任务。

| 类别 | 任务 | 现在可检查的场景骨架 | 进入实际测评前的关键缺口 |
| --- | --- | --- | --- |
| 抓取 | G06 苹果梗 | 苹果球体与细梗 | 内部 SuperDex 资产、夹梗接触和提起策略 |
| 手内操作 | H01 方块、H02 笔、H03 蛋 | 三项物体的尺寸、质量、接触参数和步长已核对源 MJCF；H03 不含源文件的零质量无碰撞可视层 | 掌中初态与 Wuji 策略；H02 目标点和笔轴评分接线；H03 姿态评分与随机目标采样 |
| 插入与交接 | I03 钥匙 | 已接入源钥匙与槽网格，多凸块碰撞近似 | 孔口碰撞精度、交接/插入/旋转阶段观测 |
| 搬运 | M01 装箱、M02 工具归位、M03 堆积木 | M01 已显示源车、牙刷、锤子、鞋和盒子，M02 已显示源工具和工具箱，M03 已显示三块同组源积木 | M02 工具质量与槽位碰撞；M03 随机初态、堆叠判据与策略；M01 盒底功能点、朝向判据与源 reset |
| 推动 | P01 T 块、P02 滑物、P03 扫积木 | P01 已接入源 T 网格，P03 已显示源扫帚、簸箕和三块方块；P02 滑块尺寸、质量、桌面摩擦、求解步长取自固定版本 Fetch Slide MJCF | P01 源碰撞与 reset 对齐；P02 目标/随机初态、击球策略与位置误差接线；P03 工具碰撞、方块尺度与逐块入盒判据 |
| 转动 | R01 扳手、R02 水龙头、R03 门 | R01 仍是自建占位体；R02 已迁入源水龙头双把手机构并读出 `joint_0`；R03 已迁入源门机构并读出门轴角度 | R01 源资产；R02/R03 原 reset、把手接触与控制；接触力矩 |

7 项代理场景由 [diagnostic_proxy_scenes.json](../profiles/diagnostic_proxy_scenes.json) 声明；P01/I03/M01/M02/M03/P03/R02 由 [source_asset_scenes.json](../profiles/source_asset_scenes.json) 和[源资产构建脚本](../integrations/unilab/build_source_asset_scenes.py)声明。对应 [任务 owner](../conf/unilab/task_catalog/) 明确区分 `unilab_proxy_diagnostic` 和 `unilab_source_asset_diagnostic`；[运行脚本](../integrations/unilab/run_proxy_task_scene.py)从 UniLab 读取物体、手腕、关节与步进耗时。任何任务都不能仅靠 `kind` 或 `backend` 字符串宣称源任务已完成。

原始 X5/Wuji 网格和生成的 MJCF 沿用仓库约定，不提交 Git。新机器需先按[机械臂准备说明](x5-control.md#如何运行)和[Wuji 模型准备说明](wuji-sharpa-hands.md#本机怎么切换)恢复依赖。构建 H01/H02/H03/P02/R03 前还需分别下载固定哈希的 Farama MJCF 与许可文件（`task_assets.py fetch --task H01` 等）；随后运行 `uv run --locked python -m integrations.unilab.build_proxy_task_scenes --task all` 构建 7 项代理场景。P01/I03/M01/M02/M03/P03/R02 还需按[源物体资产接入说明](source-object-asset-integration.md)下载、校验并转换源网格。构建脚本不覆盖已有场景；修改布局后请先备份旧生成目录，再重建受影响任务。

本地复查：

```bash
cd /home/jason77/dexhand_bench
uv run --locked python integrations/unilab/task_owner.py audit
uv run --locked python integrations/unilab/task_owner.py smoke --task P02 --output runs/P02-check --steps 2
uv run --locked python integrations/unilab/play_task.py --task P02 --port 8080
```

下一步是验证 P01/I03/M01/M02/M03/P03/R02 的接触近似与原环境差多少，逐项对齐 reset，再接通至少一个代表性任务的评分信号。其余任务按可获取资产的顺序迁入。只有同一套任务条件、动作和观测在不同后端都真实可用，才能进一步比较物理误差和吞吐表现。
