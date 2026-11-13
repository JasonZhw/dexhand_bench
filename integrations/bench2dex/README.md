# Bench2Dex 外部动态回放桥接

这个入口在已有 Bench2Dex 环境中运行指令回放，逐物理步记录目标关节位置、实际关节状态和指定物体状态。它不依赖主工程安装 Isaac Sim，也不是 UniLab 后端适配器

`replay_recorded.py` 基于 Bench2Dex commit `82e04a2a4e5b728a96ccf1ae7a546762d5a260c7` 的 `tools/replay/replay_commanded.py`，沿用此前本地 `work/dexbench_v0/replay_apple_recorded.py` 的记录插桩，移除了机器专属路径。上游 MIT 授权保留在 LICENSE.upstream

保留上游 PD 指令、插值、重力补偿和初始速度处理，不做控制器调优。通过环境变量 DEXBENCH_BENCH2DEX_ROOT 指定原仓库；外层监督程序负责设置它

在服务器桥接目录运行：

```bash
cd /path/to/dexhand_bench
/usr/bin/python3 -m dexbench.external_replay \
  --source-root /path/to/Bench2Dex \
  --python /path/to/isaac-env/bin/python \
  --hdf5 /path/to/apple-clip.hdf5 \
  --scene /path/to/Bench2Dex/scenes/06_fruit_bowl_loading.yaml \
  --target-object obj_013_apple_3 \
  --output runs/apple_body_02 --max-frames 71 --timeout 300
```

必须使用新的输出目录，已有结果不会覆盖。输出包括 invocation.json、replay.log、recording/manifest.json、states.jsonl、completion.json 和 result.json

2026-11-12 的 apple_body_01 运行完成 213 个物理步，Isaac Sim 5.1.0.0、Isaac Lab 0.54.2、CPU 物理设备。目标苹果原点末帧比初帧高约 0.00000006 m，整个记录高度变化约 0.00000012 m，未观察到它被抬起。这里只验证记录链路，未判定独立抓取任务成功或失败，task_outcome 保持 unknown

物体是否动态只有配置检查，尚未审计运行时标志。没有接触力、手指支撑判定、完整资产内容哈希和求解器热启动状态。不能将该结果解释为后端准确性、任务成功率或吞吐性能；也不能用它代替原始完整装水果任务的评分

## 带视频运行

监督命令追加 `--video`，会初始化源仓库的俯视相机，每个控制周期保存一张 RGB PNG。`video_frames.jsonl` 将 frame_index 与 physics_step、state_t_s 关联。原速 MP4 的 frame_index / fps 是播放时间；本次首帧对应仿真 0.05 秒，播放从 0 秒开始，故状态时间 = 播放时间 + 0.05 秒。warmup 不计入这段回放时间

运行 apple4_video_01 已确认源片段操纵的对象是 obj_013_apple_4；早先 apple_body_01 和 apple_body_video_01 跟踪的是另一颗静止苹果 apple_3，不能用它判定抬升失败。新运行仍保留 task_outcome=unknown，因为只验证了片段运动，没有完整任务与持续支撑判据

相机渲染不新增物理步，但会增加墙钟时间。视频运行不可用来报告无渲染吞吐量。PNG 序列先保留在服务器，外部编码 H264 后另存 MP4；中断或编码失败时保留状态与帧文件用于排查
