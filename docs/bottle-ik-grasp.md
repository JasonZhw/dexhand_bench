# G02 瓶子抓取：启动与结果

2026-11-21，G02 已用右侧 Wuji Hand 2.2、X5 机械臂和 UniLab / MuJoCo 跑通固定初态的 IK 脚本抓取。末尾抬高 6.74 cm，倾斜 7.86°，通过既有评分。这里的 IK 负责求机械臂关节角，手指采用预设闭合目标，尚未训练 policy

## 在本机启动

仓库在 `/home/jason77/dexhand_bench`，资产已经准备好。依赖由仓库根目录的 `pyproject.toml` 和 `uv.lock` 管理，`uv sync --locked` 会创建 `.venv`。这里运行的是 UniLab/MuJoCo，不需要 Isaac Sim 或 sudo

实时查看使用 UniLab 自带的 Viser 浏览器渲染路径。G02 的 Hydra 启动 owner 是 `conf/unilab/grasp_g02_wuji.yaml`，它继承现有 UniLab manager 环境配置，并指向 Wuji、IK 控制和 Viser。打开终端输出的 `http://localhost:8080` 后开始运行；结束后网页保留最后一帧，按 Ctrl+C 退出。不需要图形桌面，也不会保存视频

```bash
cd /home/jason77/dexhand_bench
uv sync --locked
uv run --locked python integrations/unilab/run_grasp_scene.py \
  --launch-config grasp_g02_wuji \
  --output "runs/g02-viser-$(date +%Y%m%d-%H%M%S)"
```

Viser 读取 UniLab 环境的物理状态和后端提供的 playback model；它只负责显示，不推进第二套物理仿真。任务仍由 UniLab 的 `ManagerBasedRlEnv` 和 MuJoCo 后端执行。旧的 `.venv-scene` 仅保留用于核对迁移前结果

若需要离屏保存原有视频，可继续用下面的独立命令：

```bash
cd /home/jason77/dexhand_bench
G02_RUN_DIR="runs/g02-$(date +%Y%m%d-%H%M%S)"

MUJOCO_GL=egl uv run --locked python integrations/unilab/run_grasp_scene.py \
  --task G02 --robot wuji --controller ik_grasp \
  --output "$G02_RUN_DIR" --video

xdg-open "$G02_RUN_DIR/G02/hand.mp4"
```

离屏命令完成后生成视频；`hand.mp4` 看手部，`preview.mp4` 看全景。输出目录需要是新目录，因此命令用时间戳命名

本地 uv 锁定 Python 3.10 兼容依赖、UniSim 2.7.1、MuJoCo 3.11.0 和 CPU 版 PyTorch 2.11。UniLab 1.5.9 源码由固定的 `third_party/UniLab` submodule 提供，脚本按仓库现有方式导入。MuJoCo 在 CPU 上推进；EGL 仅用于离线视频

终端显示 `success=[True, False]` 是预期结果：环境 0 做抓取，环境 1 保持 HOME，用于检查控制隔离和独立重置，不是两次抓取成功率 50%

## 配置和评分分别在哪里

| 文件 | 负责什么 |
|---|---|
| `profiles/local_grasp/G02.json` | 瓶子资产、质量、初态、后端、步长、相机和物理记录开关 |
| `profiles/robots/dual_x5_wuji.json` | Wuji 手型及关节映射；由 `--robot wuji` 选择 |
| `profiles/controllers/G02_wuji_ik.json` | 抓取位置、手掌朝向、手指闭合目标、动作时长 |
| `tasks/grasp/G02.json` | 独立任务成功条件、失败条件和统计指标 |
| `integrations/unilab/grasp_ik.py` | G01/G02 共享的分阶段 IK 控制器 |
| `integrations/unilab/run_grasp_scene.py` | 通过 UniLab 执行、采集、评分、保存视频 |

脚本自动按 G02 选择控制配置。要试自己的动作参数，可以复制这份 JSON，通过 `--controller-config 路径` 指定，不需要修改评分标准

G02 从 Bench2Dex 第 21 项“调味品装箱”中选取酱油瓶资产，拆成独立抓取任务。现在用的是本地虚拟 X5 / Wuji 装配和自拟子任务评分，不是原完整装箱任务，也不是原作者的示范或 policy

评分要求抬高至少 5 cm、质心速度不超过 5 cm/s、指定手保持接触、瓶子倾斜不超过 15°，在终末阶段连续满足 1 秒。曾抬高 5 cm 后又低于 2 cm，会触发不可撤销的掉落失败。当前初态竖直，倾角相对世界竖直轴计算；随机朝向还未开放

当前规则没有限制必须只接触瓶身某个语义区域，拇指可能接触瓶肩或瓶盖。若后续需要严格比较“瓶身抓取”，应另加经过验证的接触区域规则，不能把这次结果当成已经满足该限制

## 这次怎么抓起来的

动作依次为落稳、抬高手、到物体上方、下降、合拢、抬升、保持，共 11.2 秒。G02 将手掌相对 HOME 绕世界 Y 轴转 90°，从侧面包住瓶子，再用更充分的手指闭合维持接触。执行时只向机器人下发关节目标，瓶子始终是受重力与接触作用的动态物体，没有绑定到手上或直接改物体位姿

首次沿用掌心朝下的抓法，只带动约 5 mm；改为侧抓后，第一版闭合目标仍然没能稳住瓶子。随后调整抓取位置和闭合目标，正式 UniLab 运行通过。质量 0.7 kg、摩擦 0.6、控制增益和原有评分均未更改。快速筛选只用来选动作参数，下面的结果来自实际 UniLab 运行

| 项目 | 结果 |
|---|---|
| 最终抬升量 / 最终倾角 | 6.74 cm / 7.86° |
| 最终质心速度 | 0.126 cm/s |
| 首次完成连续保持的时刻 | 10.168 s |
| 最后 1 秒的最大倾角 / 最大质心速度 | 7.86° / 0.755 cm/s |
| 掉落事件 | 无 |
| 指定手接触法向力幅值总和峰值 | 106.18 N，不是净合力 |
| 指定手最大穿透代理 | 4.22 mm |
| 物理记录开关对照 | 2801 帧的动作、关节和物体状态完全一致，评分一致 |

接触峰值与穿透量说明这条脚本还需要做接触质量检查，任务通过不等于物理准确。碰撞体是从 USD 转出的单凸包，惯量采用均匀体积近似，材料摩擦也没有真实测量标定。当前只有一个固定初态，不能据此报告随机场景成功率

## 物理同事能拿到什么

G02 已默认启用与 G01 相同的 `ContactSource → PhysicsRecorder` 接口，输出接触组力与力矩、接触位置、穿透代理、关节和物体状态、动作阶段及步进耗时。`--physics off` 可做同动作对照；接触滑移速度暂不支持，明确记为 null。当前后端只验证了 MuJoCo，不能仅改配置名就保证其他后端可运行

具体字段和时间定义见 [物理记录说明](apple-physics-recording.md)。后续 policy 可以替换动作生成部分，继续使用这些评分和记录接口

[查看本次视频和评分](evidence/bottle-ik/index.html)。原始完整结果在 `runs/bottle-ik-20261121/G02`，失败尝试保存在 `runs/bottle-ik-20261121-v1/G02`、`runs/bottle-ik-20261121-v2/G02`

验证：完整场景环境运行 148 项测试，7 项因未配置的可选环境跳过，其余通过；增加了瓶子评分边界检查，扩展了 G01/G02 的 IK 位姿、动作连续性和控制隔离测试。G01 的各阶段关节目标与此前成功运行逐项相同
