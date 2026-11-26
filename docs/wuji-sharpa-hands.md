# 五个抓取任务中的 Allegro、Wuji 和 Sharpa

目前三种手型都沿用双 X5 机械臂、同一组物体和 G01–G05 的评价规则。手型通过运行参数选择，机器人资产、关节控制和逐指接触分组跟着切换

[打开 Wuji / Sharpa 场景与录像](evidence/vendor-hands/index.html) · [Allegro 版及五项任务说明](five-local-grasp-scenes.md)

| 当前模型 | 手指 / 每手控制关节 | 加上双臂的控制通道 | 模型来源 |
|---|---:|---:|---|
| Allegro V3 | 四指 / 16 | 44 | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/370c43965323c0944b433b202aac5058b5f907a9/wonik_allegro) |
| Wuji Hand 2.2，裸机版 | 五指 / 20 | 52 | [Wuji 官方模型](https://github.com/wuji-technology/wuji-description/tree/ffeb47f18545e135abba4386ff745c940cb8ce28/hand2/hand2_2/body) |
| Sharpa Wave，带法兰版 | 五指 / 22 | 56 | [Sharpa 官方模型](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml/tree/d323b371d02f4fc04a0873c19c5234cae6d803dd/wave_01) |

Allegro 这一版采用三个普通手指和一个拇指，方便先建立关节控制和接触记录的基线。Wuji 和 Sharpa 增加了小指，手掌形状、指长、质量、驱动和接触布局也不同。Sharpa 这份模型的拇指有五个关节，小指多一个掌部关节，总计 22 个；其余三指各四个。Wuji 这份模型五指各四个关节

这里新增的是能进入现有 UniLab 执行链路的机器人模型。厂商模型提供的触觉相关几何或坐标系不会自动变成已接通的触觉数据，关节数量也不能直接推导任务能力排序。要比较哪只手更适合某个任务，还需要各自可用的控制器与公平的控制条件

## 配置和资产在哪里

两份共享机器人配置是 [dual_x5_wuji.json](../profiles/robots/dual_x5_wuji.json) 和 [dual_x5_sharpa.json](../profiles/robots/dual_x5_sharpa.json)。其中明确记录源模型、关节数、安装变换、手指分组、诊断动作和运行限制

[grasp_profiles.py](../integrations/unilab/grasp_profiles.py) 把 `profiles/local_grasp/Gxx.json` 与所选机器人配置合成运行配置。例如 `--robot wuji --task G01` 使用相同的苹果和 `tasks/grasp/G01.json` 评分规则，但选择 Wuji 的机器人模板和 `assets/generated/wuji_grasp/G01/scene.xml`。最终解析后的配置会保存在输出目录的 `config.json`，不会将别的手型旧场景误当成当前场景

Wuji 的 MJCF、网格与许可保存在 `assets/vendor/wuji/`，Sharpa 保存在 `assets/vendor/sharpa/`。此次仅下载模型直接依赖的文件，分别为 58 个文件约 9.0 MB、57 个文件约 15.0 MB，没有下载整个产品库。固定提交、文件 SHA256 和大小见 [Wuji 清单](../asset_manifests/wuji-hand2-2.json) 与 [Sharpa 清单](../asset_manifests/sharpa-wave.json)。许可证分别为 MIT 和 Apache-2.0，原始许可与 Sharpa NOTICE 随清单下载保留。原始资产及生成模型不提交 Git，其他机器按清单恢复

采用两家直接提供的 MuJoCo XML，保留原模型关节范围、质量/惯量、驱动、接触参数和碰撞排除。Sharpa 的 `dampratio` 会根据模型惯量计算阻尼；我们先读取独立原模型对应的阻尼，再明确写成 `kv`，避免接上机械臂后悄悄改变原驱动参数。左右手所有命名、材质和传感器引用分别加前缀，按实际关节顺序排列驱动

安装位置依据可视几何作虚拟对齐：X5 link6 网格在局部 +X 的边界为 0.084 m，Wuji 安装原点设为 0.086 m，Sharpa 设为 0.088 m，使源手模型与该边界留约 2 mm 间隔；Wuji 额外旋转掌面朝向桌面。这不是实测法兰或承载验证，现有 X5 末端基础外壳仍然保留

## 本机怎么切换

本机模型和五项场景已构建，直接从仓库根目录执行，输出目录用尚不存在的新名称

```bash
# Wuji：苹果任务，附全景和右手近景录像
MUJOCO_GL=egl .venv-scene/bin/python integrations/unilab/run_grasp_scene.py \
  --robot wuji --task G01 --output runs/my-wuji-apple --video

# Sharpa：五项依次运行
MUJOCO_GL=egl .venv-scene/bin/python integrations/unilab/run_grasp_scene.py \
  --robot sharpa --task all --output runs/my-sharpa-five --video

# 原有 Allegro 仍可使用
MUJOCO_GL=egl .venv-scene/bin/python integrations/unilab/run_grasp_scene.py \
  --robot allegro --task G01 --output runs/my-allegro-apple --video
```

换机器时，先按 [五任务说明](five-local-grasp-scenes.md) 准备 X5 基础模板和五个物体，再恢复手模型和场景

```bash
python3 -m dexbench assets --manifest asset_manifests/wuji-hand2-2.json --fetch
python3 -m dexbench assets --manifest asset_manifests/sharpa-wave.json --fetch
.venv-scene/bin/python integrations/unilab/build_vendor_robot.py --robot wuji
.venv-scene/bin/python integrations/unilab/build_vendor_robot.py --robot sharpa
.venv-scene/bin/python integrations/unilab/build_grasp_scene.py --robot wuji --task all
.venv-scene/bin/python integrations/unilab/build_grasp_scene.py --robot sharpa --task all
```

构建脚本不覆盖已有目录。改安装位或源手参数时，先将对应机器人生成目录和五项场景目录移到备份位置，再按顺序重建；仅改变控制器或运行步数无需重建资产。启动时会核对机器人配置、场景和网格哈希

## 怎么看这轮结果

本轮动作是双臂小幅运动加五指屈伸，环境 0 执行动作，环境 1 保持默认指令。视频来自真实 UniLab 状态记录，再用 MuJoCo 离线渲染，没有额外推进物理，也没有控制物体位姿。评价读取真实观测，但这个诊断控制器没有接近、闭合抓取和抬升，因此不能把它当作成功策略复现

物理接口继续记录关节状态、物体状态、动作、每步耗时、左右手逐指和掌面接触标签。接触标签及最大穿透由记录位姿离线重建，包含物体与桌面接触；接触力和滑移尚未实现，保存为 null。Wuji 和 Sharpa 都有五个手指分组，原 Allegro 仍为四组

当前集成统一使用 MuJoCo、implicitfast、4 ms 步长、elliptic 摩擦锥和重力补偿；Wuji 原独立模型配置的是 RK4、2 ms，差异已记入机器人记录。保留厂商局部参数不等于复现了厂商全部物理行为。这些控制检查也不是物理后端排名，后续比较需进一步固定或说明求解配置、碰撞近似、驱动和策略差异

本轮验收记录与视频见上方总览；完整原始运行数据保存在 `runs/vendor-hands-final-20261119/`。下一步可以挑一只手先实现苹果的接近、闭合、抬升和保持，再逐项推进其他物体，任务评价接口无需重写
