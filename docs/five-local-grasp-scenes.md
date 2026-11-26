# 五个基础抓取任务：本地场景初版

2026-11-19。先把任务收敛成“从桌上抓起一个物体并保持”，用五种物体覆盖抓取区域、姿态和负载差异。现在五项都有本地资产、数值初态、UniLab 启动配置、真实观测评分和预览视频。五项默认机器人现为 X5 双臂＋左右 Allegro V3 灵巧手，每只手四根手指、16 个可控关节；原平行夹爪的手指、开合关节及对应驱动已移除。成功抓取策略尚未接入

[打开五项场景与视频总览](evidence/grasp-five/index.html)

已新增 [Wuji Hand 2.2 和 Sharpa Wave 可切换版本](wuji-sharpa-hands.md)，共用同一套五项任务和评价规则。下文保留 Allegro 默认版的运行记录

| 任务 | 来源与改编 | 本项侧重点 | 当前质量 | 配置 |
|---|---|---|---|---|
| G01 苹果本体抓取 | Bench2Dex 06 水果装碗，抽取苹果抓起阶段 | 较大的抓取区域、曲面接触 | 0.18 kg，USD 属性 | [启动](../profiles/local_grasp/G01.json) · [评分](../tasks/grasp/G01.json) |
| G02 瓶身抓取 | Bench2Dex 21 调味品装箱，抽取瓶身抓起阶段 | 抓起后保持直立 | 0.70 kg，USD 属性 | [启动](../profiles/local_grasp/G02.json) · [评分](../tasks/grasp/G02.json) |
| G03 电池抓取 | Bench2Dex 24 文具分类，抽取电池抓起阶段 | 较小圆柱体、位置容差 | 0.10 kg，源场景覆盖值 | [启动](../profiles/local_grasp/G03.json) · [评分](../tasks/grasp/G03.json) |
| G04 螺丝刀柄抓取 | Bench2Dex 12 工具装箱，抽取抓柄阶段 | 限定接触区域，不能靠杆部或掌面承托 | 0.10 kg，USD 属性 | [启动](../profiles/local_grasp/G04.json) · [评分](../tasks/grasp/G04.json) |
| G05 平底锅柄抓取 | Bench2Dex 08 平底锅上架与倒入，抽取抓柄阶段 | 偏置重心、更大负载、保持锅面姿态 | 1.20 kg，源场景覆盖值 | [启动](../profiles/local_grasp/G05.json) · [评分](../tasks/grasp/G05.json) |

这五项的动作流程都较短；锅柄是负载对照，不能据此把五项说成难度相同或已经测出严格难度排序。来源任务及其原始评分代码保留在任务 JSON 中，当前的独立抓取条件由我们拟定，并非 Bench2Dex 官方完整任务评分


## 灵巧手配置

左右手采用 [MuJoCo Menagerie 的 Allegro V3 模型](https://github.com/google-deepmind/mujoco_menagerie/tree/370c43965323c0944b433b202aac5058b5f907a9/wonik_allegro)，源模型以 BSD-2-Clause 发布，固定版本和文件哈希见 [资产清单](../asset_manifests/allegro-v3.json)。保留源手指关节范围、位置驱动增益、阻尼、质量/惯量和手内碰撞排除；这是现成 MuJoCo 模型，区别于 Bench2Dex 的 USD Panda＋Allegro 整机

本项目的虚拟安装将 Allegro 手指朝向轴 +Z 对齐到 X5 link6 的末端轴 +X，掌体原点沿该轴偏移 0.20 m。这个坐标是仿真装配配置，不是实测安装法兰。机器人使用重力补偿；拇指初始角设为 0.30 rad，以满足源模型正下限，同时调整对应局部坐标，已与原模型的正运动学对照。左右手的源关节遍历顺序不同，构建时按名称重排驱动，避免左右手指错位

当前全场景采用 implicitfast 积分、elliptic 接触锥、impratio=10，后两项沿用手模型的设置。因此不能将本轮与旧夹爪运行的穿透差异单独归因于换手。初态无机器人自碰撞；仿真安装、简化碰撞和驱动参数仍需在实际抓取中校准

## 怎么判定完成

共同的初步规则：相对 reset 高度抬升至少 5 cm，质心速度不超过 5 cm/s，指定右手持续接触，连续满足 1 秒。曾抬升到 5 cm 后再跌回 2 cm 以下，记为掉落失败。G02 另要求倾角不超过 15°；G04、G05 限制接触落在暂定柄部区域，排除掌面和其他机器人链接承托；G05 同时限制倾角不超过 15°

高度、速度、保持时间和柄部包围盒都是待校准的初稿。当前接触由每帧真实关节和物体位姿在独立 MuJoCo 模型中重建，采用 contact.dist ≤ 0；它表示几何接触，不是已验证的承载力。柄部 AABB 也不是源数据提供的语义分割。在正式比较策略前，需要补充实际抓取对照与区域标注检查

## 如何运行

本机环境和资产已准备好。从仓库根目录执行，输出目录使用一个新名字：

```bash
# 五项依次运行，输出真实状态、评分和视频
MUJOCO_GL=egl .venv-scene/bin/python integrations/unilab/run_grasp_scene.py \
  --task all --output runs/my-five-grasps --video

# 仅运行苹果；去掉 --video 可不渲染
MUJOCO_GL=egl .venv-scene/bin/python integrations/unilab/run_grasp_scene.py \
  --task G01 --output runs/my-apple --video

# 打开可交互的 MuJoCo 模型查看器；这是查看资产，不会运行 UniLab 评分流程
.venv-scene/bin/python -m mujoco.viewer \
  --mjcf=assets/generated/allegro_grasp/G01/scene.xml
```

运行链路为 `profiles/local_grasp/Gxx.json → UniLab ManagerBasedRlEnv → MuJoCo → 真实状态记录 → tasks/grasp/Gxx.json 的评分规则`。启动 JSON 引用评分 JSON，分别管理场景运行和任务协议；共享代码负责状态到评分信号的转换

`backend` 字段目前只接受 `mujoco`，其他值会明确报错。后续给同一任务增加其他后端时，仍要提供对应资产、控制与观测适配，不能仅改这个字符串就声称实现跨后端迁移。`hold` 和 `probe` 都是诊断控制器；后者让环境 0 的双臂基座和左右手的四根手指做小幅动作，环境 1 保持默认目标，没有接近、抓取和抬升策略

每项输出 `config.json`、`states.json`、`trace-0.json` / `trace-1.json`、`evaluation.json`、`physics.json`、`receipt.json`。启用渲染再生成场景录像 `preview.mp4`、右手近景录像 `hand.mp4`，以及 `preview.png`、`hand.png` 和 `object.png`。录像由记录的状态重建，仅 forward，不额外推进物理；默认 640×480、25 fps、50 帧、2 秒

## 资产准备和修改

物体来自 [Bench2Dex 官方资产集](https://huggingface.co/datasets/Bench2Dex/Assets)，固定到 `bf65215f844d1fca30750e4cc70d7266650abf0a`。六个文件合计约 7.8 MB，只下载几何和实例依赖，没有下载整套资产或外观纹理。[清单](../asset_manifests/G01-G05-local.json) 固定 URL、大小和 SHA256；原始资产与生成网格不提交 Git

新机器先按 [X5 控制接入说明](x5-control.md#如何运行) 准备机械臂基础模板及其依赖的 I01 模板，再执行：

```bash
uv pip install --python .venv-assets/bin/python -r requirements/scene-assets.txt
python3 -m dexbench assets --manifest asset_manifests/G01-G05-local.json --fetch
python3 -m dexbench assets --manifest asset_manifests/allegro-v3.json --fetch
.venv-scene/bin/python integrations/unilab/build_allegro_robot.py
.venv-assets/bin/python integrations/unilab/prepare_grasp_objects.py --task all
.venv-scene/bin/python integrations/unilab/build_grasp_scene.py --task all
```

现有本机无需重做这一步。准备脚本不覆盖已有输出；修改参数后应先把对应生成目录移到备份位置，再按依赖顺序重建。只修改 controller 或运行步数无需重建资产；修改质量、摩擦、初态须重建场景，修改缩放、朝向、碰撞近似还须重新准备网格。启动时校验场景 XML 及所有外部网格的哈希，防止旧记录与新资产混用

USD 到 MJCF 涉及显式转换：读取实例中的碰撞网格，应用源场景缩放和局部朝向，再把物体底面归一到 z=0。瓶子、电池旋转 +90°，锅旋转 -90°，锅的展示 yaw 改为 0°。G01–G04 使用单凸包；G05 当前生成 21 个凸块近似凹形锅体。显示使用导出的源碰撞表面与纯色，质量按上表，惯量按凸块体积估算，统一摩擦是我们的基线假设。这不等价于源 PhysX 的碰撞、惯量和材料行为

生成模型包含本机绝对网格路径，换机器应重建。当前复用 X5 机械臂模板是阶段性的代码复用，后续可把机械臂模板准备从 I01 中单独拆出。灵巧手由独立生成脚本加入，共享定义在 [dual_x5_allegro.json](../profiles/robots/dual_x5_allegro.json)，手指命名、动作顺序和接触分组由生成后的机器人描述统一提供

## 实际验证与物理接口

本机用 UniLab 1.5.9 固定子模块、UniSim 2.7.1、MuJoCo 3.11.0 完成五项运行，每项两个环境、500 步、2 秒仿真。两侧各 6 个机械臂关节＋16 个手指关节，共 44 个位置控制通道；原夹爪滑动关节已不存在。被驱动的手指近端关节实际运动约 0.346 rad，双臂基座约 0.08 rad；选定环境的位姿、速度和观测重置后恢复，另一个环境状态不变。十个诊断 episode 的抓取评分均为 false，原因是控制检查没有执行抓取，不能拿来报告策略成功率

物理接口已记录物体位姿、线/角速度、44 个关节状态、动作和每步耗时，并增加左右手逐指几何接触标签及掌面接触标签。`penetration_m` 是按记录位姿离线重建的最大物体接触穿透，包含桌面。`contact_force_n`、`slip_m_s` 暂不支持，明确为 null；后续替换为各后端实际提供的传感或求解数据。耗时只计 CPU `env.step`，包含 managers，排除离线接触重建和渲染；它不是做过预热、同步的正式吞吐结果

此次几何记录里，电池的最大穿透约 6.10 mm，其他对象约 1.18–1.26 mm。电池接触近似/求解设置需要继续检查，暂不作为物理准确性通过的证据。跨后端比较前需固定碰撞近似、步长、材料与控制设置，并另建物理参考。当前只是把异常记录下来，尚未据此确定根因

完整测试 134 项：127 项通过，7 项因缺少原环境依赖跳过。视频和验收记录见 [grasp-five](evidence/grasp-five/index.html)，完整逐步数据保留在本机 `runs/allegro-five-final-20261119/`

下一步先在同一套 Allegro 配置上为 G01 接入“接近—闭合—抬升—保持”的控制器并检查真实接触，再扩展到其余四项。本轮已替换灵巧手并验证关节映射和重置，但尚未把完整抓取任务标成成功。原夹爪实验作为 [历史记录](evidence/grasp-five-gripper-20261119/index.html) 保留
