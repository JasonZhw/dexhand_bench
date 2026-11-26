> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# 抓取回放与下一批场景准备

现在新增的是一个实际跑过的 Bench2Dex 外部回放入口。它让现有服务器环境产生逐步记录，便于后续定位动作执行偏差；主工程的五个 MuJoCo 原始任务入口继续保留

苹果本体短片段跑完了 213 个物理步，日志与状态记录已取回。指定苹果没有明显上升，因此不把它标记为抓取成功，也不升级 G01 的任务完成状态。这不是苹果梗任务

物体装箱 M01 选用 RoboDojo 的 arx_x5、评测种子目录 0、布局文件 pack_objects_into_box_0.json，固定物体是 car/00007、electric_toothbrush/00002、hammer/00003、shoe/00003、box/00000。先固定这些型号和摆放，后续才能区分移植差异与随机场景差异

插管 I01 使用同一机器人配置和种子目录中的 insert_tubes_0.json，三根 test_tube/00002 配一个 test_tube_slot/00000。管架采用 SDF 碰撞近似，迁移时必须验证孔洞保持可插入，不能直接用整体凸包替代

两个固定布局已经下载并记录版本与 SHA256。依赖清单分别列出 209 和 219 个目录候选文件，其中目前本地存在 4 和 0 个。这不是精确的 USD 递归依赖闭包，目录内可能有不用的预览或源文件；不能把文件列表当成场景已经可运行

下一步先补齐插管的原始机器人与场景依赖，验证完整场景能够加载、重置、记录状态，再补装箱固定物体。先沿用原始双臂夹爪验证任务定义，换灵巧手时另记机器人、控制映射与成功条件的变化。不要把原始夹爪结果冒充灵巧手结果

物理测评接口保持动作、状态、时间步、接触能力和后端配置分离；当前未知或未支持的接触数据继续记录为 null。未来接入 UniLab 时保留同一任务定义和评分输入，只替换后端交互部分

相关文件：integrations/bench2dex/README.md、docs/evidence/external-replay-20261112.json、docs/evidence/robodojo-layouts/sources.json、asset_manifests/I01-fixed-layout-dependencies.json、asset_manifests/M01-fixed-layout-dependencies.json
