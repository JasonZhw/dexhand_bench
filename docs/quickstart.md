> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# 本地开始运行

在整个dexbench目录内操作。核心目录查询只需要Python 3.10以上；仿真使用单独环境，避免修改Isaac Sim依赖

```bash
# 若已安装uv
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements/simulation.txt

# 或使用已有Python3.12
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements/simulation.txt
```

已有环境无需重新创建。运行下面的命令前，输出目录必须不存在，程序不会覆盖旧实验

```bash
.venv/bin/python -m dexbench list
.venv/bin/python -m dexbench profiles

# 原Shadow手方块任务，随机输入用于检查链路，绝不是成功策略
.venv/bin/python -m dexbench run \
  --profile profiles/H01-source.json --output runs/H01_seed0 \
  --seed 0 --steps 100 --action random --contacts

# 无窗口视频；需要机器支持EGL
MUJOCO_GL=egl .venv/bin/python -m dexbench run \
  --profile profiles/H02-source.json --output runs/H02_video \
  --seed 0 --steps 100 --action zero --video

# 重放前一次记录的归一化动作；需相同原环境、seed和控制周期
.venv/bin/python -m dexbench run \
  --profile profiles/H01-source.json --output runs/H01_replay \
  --seed 0 --steps 100 --actions runs/H01_seed0/actions.json --contacts

.venv/bin/python -m dexbench compare \
  --left runs/H01_seed0 --right runs/H01_replay

# 独立CPU进程批跑，明确使用吞吐模式
.venv/bin/python -m dexbench batch \
  --profile profiles/H01-source.json --output runs/H01_batch \
  --seeds 0,1,2,3 --workers 2 --steps 100 --warmup 10 --mode throughput

.venv/bin/python -m unittest discover -s tests
```

profile里physics为空表示采用原模型设置。要做求解器对照，复制profile到新文件，再填写例如`{"iterations": 10, "solver": "Newton"}`。不要覆盖原profile。重放相同动作后比较轨迹；这得到的是一致性差异，需要额外参照才能讨论准确性

每次输出包含manifest.json（版本、配置、资产/源码哈希）、initial_state.json、initial_observation.json、actions.json、summary.json；诊断模式另含steps.jsonl，视频模式另含video.mp4，异常时含error.txt

summary中的execution_completed只表示程序结束；task_outcome来自原环境终帧成功判据。预算提前结束且未满足判据时是unknown；到原环境终止或时限仍为false时才是failure。any_source_success与final_source_success都保存，避免把短暂满足误写成最终成功

资产清单下载/校验示例：

```bash
python3 -m dexbench assets --manifest asset_manifests/I01-robodojo.json
python3 -m dexbench assets --manifest asset_manifests/I01-robodojo.json --fetch
```

fetch只下载已有固定SHA256的条目，不覆盖校验失败的旧文件；未解析的条目会明确报告。官方大模型、演示数据和训练权重不会自动下载

如果终端已经加载ROS等外部Python路径，建议命令前加`env -u PYTHONPATH`。本轮发现全局PYTHONPATH会让pip看到与本项目无关的launch-ros包；清除该变量后环境依赖检查正常。不要为了这条外部警告给benchmark安装整套ROS依赖

本轮固定MuJoCo3.2.5：用3.3.7加载当前1.4.1包中的旧XML时遇到apirate属性错误。已保留失败日志，没有直接修改官方XML。上游1.4.2有对应兼容修复，但升级任务包后仍需重新记录资产/评分版本

如果模型构造就失败，manifest仍保存请求配置，backend为null，summary标记未完成并留下error.txt；此时不会有有效初态或步进记录。视频编码收尾失败同样标记未完成，不把不完整录像当作成功交付
