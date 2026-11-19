# 双臂 X5 控制接入

当前进展：插管场景从只有物体，推进到双臂机器人可以通过 UniLab 的动作接口运动。统一 YAML 同时声明机器人实体、16 个关节、动作、观察、重置、原有几何指标与超时。没有抓取策略，没有插入成功率；这两只手是原 RoboDojo 的平行夹爪，尚未替换成灵巧手

## 资产和控制从哪里来

机器人使用固定版本 RoboDojo 数据集的 `Assets/Robots/x5/X5A.urdf`、九个 STL 和 `robot_config.yml`，共 11 个文件，由 `asset_manifests/I01-x5.json` 固定版本和 SHA256。生成脚本借助 MuJoCo 的 URDF 编译器保留惯量和关节坐标，将两个机器人放在源 `dual_x5.yml` 的位置：左右 x 为 -0.3/+0.3 m，y=-0.45 m，z=0.765 m，绕 Z 轴旋转 90°

每臂 6 个旋转关节、2 个滑动夹爪关节，名称加 left/right 前缀。夹爪初始开度使用源配置的每指 0.044 m；测试将两指和末端的世界位置与原 URDF 正运动学逐一核对。`JointPositionActionCfg` 下发相对初态的关节目标：角度单位 rad，滑动距离单位 m。当前脚本只对环境 0 的双臂基座关节下发 ±0.15 rad 平滑动作，并让四个手指收拢 0.015 m；环境 1 保持初始目标，作为并行控制对照

机械臂刚度 4400、阻尼 40、armature 0.01；夹爪刚度 2300、阻尼 100；力/力矩上限 100。数值参考源 `env/robot_manager/robot_config/x5.py`，但 MuJoCo 的位置驱动和关节阻尼不能直接视为 PhysX implicit actuator 等价实现。机器人设置重力补偿，物体保留重力；当前使用 implicitfast 积分

## 本次暴露的问题

1. 当前 UniLab 非组合场景路径以关节状态索引读取限位，而后端限位数组只含受控关节。物体 free joint 排在机械臂前时会越界。生成脚本将受控标量关节放在自由物体前，保留固定上游版本，并增加回归测试；这不是对所有混合模型都通用的上游修复
2. URDF 网格在 MuJoCo 中按凸包碰撞，原本相邻的 base_link/link1 有约 2 mm 重叠，导致基座关节动作被阻挡。本适配显式排除 URDF 直接父子链接间的碰撞，保留其他自碰撞及机器人与物体碰撞。这是一项明确的碰撞配置选择，不声称等同原 USD
3. USD 物理参数、URDF 惯量、凸包碰撞和相机外观并不自动一致。本次省略纯视觉相机网格；视频展示碰撞模型。后续接触测评前还要对齐碰撞形状、材料和控制参数

## 如何运行

先按 [物体场景说明](insert-tubes-scene.md) 安装 `.venv-scene`、生成 `assets/generated/I01-convex/scene.xml`，然后从仓库根目录执行：

```bash
uv run --no-project --python .venv-scene/bin/python python -m dexbench assets --manifest asset_manifests/I01-x5.json --fetch
uv run --no-project --python .venv-scene/bin/python python integrations/unilab/build_x5_scene.py --output assets/generated/I01-x5-control
MUJOCO_GL=egl uv run --no-project --python .venv-scene/bin/python python integrations/unilab/run_x5_control.py output=runs/I01-x5-demo video=true
```

输出目录必须是新目录。没有离屏渲染条件时使用 `video=false`，控制检查仍可运行。视频需要 ffmpeg 与 EGL；不需要桌面窗口

配置入口为 `conf/unilab/x5_control_diagnostic.yaml`，继承物体场景的观察、重置和指标，额外连接机器人动作及关节观察。输出 `config.json`、`states.json`、`receipt.json`，启用视频时再输出 `control.mp4`、`control.png`。生成 MJCF 包含本机资产路径，换机器要重新生成，不要直接拷贝生成的 XML

## 实际结果与下一步

本地 CPU MuJoCo / UniSim 2.7.1 / MuJoCo 3.11.0，两个环境各运行 500 步、2 秒，观察形状为 2×69。运动中点双臂基座关节目标误差约 0.000016 rad，四个手指约 0.11–0.12 mm；这只是这段动作的跟踪检查，不是全任务精度。选定环境重置后关节位置、速度及所有观察恢复，另一环境不变。全套 100 项检查中 93 项通过、7 项因缺少原环境依赖跳过

视频由记录的环境 0 关节和自由物体位姿重建，在独立 MuJoCo renderer 中只做 forward，不增加物理步。每 10 步一帧，640×480、25 fps、50 帧、2 秒。不能用录制耗时报告无渲染仿真吞吐

本地证据目录为 `runs/I01-x5-reviewed`；可提交的配置和验收记录位于 [evidence/I01-x5-control](evidence/I01-x5-control)。这段视频只能说明真实控制链路已接通，不能当作插管任务成功演示

下一步在相同配置和日志接口上加入末端运动、抓取和抬起的阶段检查，再做对孔与插入。接触力和穿透诊断继续走独立的观察/指标项，按后端支持能力读取；不把控制跟踪误差直接称为物理引擎准确性。完整 I01 仍保持未就绪，其他后端也尚未通过该机器人场景验收
