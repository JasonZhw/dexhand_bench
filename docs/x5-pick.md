# 从关节控制推进到一次真实抓取尝试

本轮将已有双臂控制接到“接近试管 → 下降 → 闭合夹爪 → 抬起 → 保持”。执行走真实 UniLab ManagerBased 环境和 MuJoCo 物理；没有把物体绑定到手上，没有执行期写物体位姿。这里使用原 ARX X5 平行夹爪，自拟阶段控制，不是 RoboDojo 官方策略，也不是灵巧手任务成绩

**目前流程能抬起物体，但存在明显穿透，接触质量尚待改善，完整插管未完成。** 两组只改变夹爪闭合目标的对照，说明为什么需要同时保留任务结果和物理诊断

| 固定场景的两次运行 | 较紧：每指目标 5 mm | 较宽：每指目标 18.5 mm |
|---|---:|---:|
| 最后 1 秒最低高度增量 | 约 85.4 mm | 约 0 mm |
| 最后 1 秒最大位置漂移 | 约 5.5 mm | 约 0.05 mm |
| 保持阶段双指接触占比（离线几何） | 100% | 0% |
| 全程指尖穿透峰值（离线几何） | 约 10.0 mm | 0 mm |
| 保持阶段穿透峰值 | 约 5.8 mm | 0 mm |
| 高度和稳定性判据 | 通过 | 未抬起 |

这里的目标是两个滑动关节各自的位置，不是直接测量的夹爪间隙。穿透量来自当前凸包碰撞模型重建的带符号接触距离，不能当作真实机器人变形或测量真值。这两次固定初态运行不是统计成功率，不能用于引擎排行

## 统一配置如何执行

入口 `conf/unilab/x5_pick_diagnostic.yaml` 继承机器人与物体配置，额外定义目标物体、手臂、TCP 偏移、接近/抬升高度、夹爪目标、阶段步数与保持判据

- settle：125 步，等待物体落稳并记录 tube0 初始位置
- approach：500 步，移到试管上方 12 cm
- descend：300 步，将抓取中心降至试管中心
- close：250 步，闭合手指
- lift：500 步，将目标提高 10 cm
- hold：250 步，保持 1 秒，记录最低高度和最大漂移

物理与控制步长都是 0.004 s，总计 1925 步、7.7 秒。环境 0 执行动作，环境 1 保持初始关节目标。每阶段记录动作、关节位置/速度、物体位姿；末端跟踪偏差超过 15 mm 时终止该尝试并记录失败阶段。未达到持续抬升条件时也写出正常失败结果，而不是省略证据

逆运动学使用单独的 MuJoCo 运动学模型，通过雅可比求解 TCP 位置和姿态。TCP 沿 link6 的局部 X 轴偏移 0.145 m，参考原 robot_config 的 gripper_bias；这是本次抓取点约定。姿态让抓取方向朝下、夹爪沿试管横向闭合。随后平滑插值关节目标，由 UniLab 动作接口推进物理，规划模型本身不控制物体

**当前 IK 只验证终点可达，不保证路径避障。** 还没有闭环纠偏、搜索抓取点或重新抓取。这是固定布局的一条初步执行基线

## 物理测评接口接在哪里

`dexbench/pick.py` 只接收物体位置序列和阈值，不依赖仿真引擎：保持阶段每一帧都至少抬高 5 cm，并且相对保持第一帧的最大漂移不超过 1 cm。它输出 `lift_observed`，不输出完整任务成功

`pick_contact_probe.py` 是独立的 MuJoCo 诊断扩展：读取保存的关节与物体位姿，重建 close/lift/hold 阶段的两指接触和穿透峰值。它只做 forward，不推进原仿真，也不把重建出的力当作原求解器力。`contact_force_available=false`、`contact_force=null` 表示本次没有采集运行时接触力，不表示所有后端都不支持该能力

之后可以在统一记录后增加其他后端对应的接触、力、求解耗时和并行性能采集器，保持任务的阶段和成功条件一致。本次正好暴露出一个需要定位的问题：高度通过时仍有很大穿透。原因可能涉及凸包近似、夹爪闭合目标、接触参数及控制刚度，不能仅凭这次结果归因于引擎

## 运行和证据

先完成 [双臂场景与依赖准备](x5-control.md)，然后从仓库根目录执行；输出目录必须不存在：

```bash
MUJOCO_GL=egl uv run --no-project --python .venv-scene/bin/python python integrations/unilab/run_x5_pick.py output=runs/pick_closed video=true
MUJOCO_GL=egl uv run --no-project --python .venv-scene/bin/python python integrations/unilab/run_x5_pick.py output=runs/pick_open pick.close_position_m=0.0185 video=true
```

没有渲染条件可使用 `video=false`。新控制器没有增加依赖，运行时复用现有 MuJoCo/NumPy/Torch。每次保存 config、IK plan、states、contact_geometry、receipt，以及可选视频。视频每 10 步采样，192 帧、25 fps、7.68 秒；最后不足 10 步没有编码，完整 1925 步仍在状态记录中

最终本地结果为 `runs/I01-pick-closed-reviewed` 和 `runs/I01-pick-open-reviewed`。提交的简洁证据见 [evidence/I01-pick](evidence/I01-pick)，完整逐步记录和视频留在本地，不入 Git。失败的 NumPy JSON 写入路径已修复并增加回归测试，旧的 `I01-pick-open-final` 不作为成功验收证据

下一步先降低接触穿透并检查夹爪碰撞形状，在相同轨迹与初态下复测，再推进对孔插入。完整 I01 的 ready 仍为 false；当前默认较紧配置作为保留的诊断案例，不是推荐的最终物理参数
