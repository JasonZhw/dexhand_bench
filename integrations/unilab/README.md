# UniLab 接入原型

当前阶段是任务配置组合与评分接口验证，不是完整仿真环境。启动配置见仓库根 README；configure.py --cfg job --resolve 只展示组合结果

check_metrics.py 在真实 UniLab MetricsManager 中输入两个合成环境的布尔判据，检查 100/20 分和部分重置。它不加载物理场景，也不验证实际插管。环境 owner 必须在每个评价步先更新 InsertTubesProgress，再调用 MetricsManager.compute；重置前先汇总指标，再清理任务状态

原工程做过接口实验，新 checkout 不附带旧虚拟环境。先按固定版本 third_party/UniLab/README.md 安装依赖，再从仓库根目录用该环境的解释器执行 integrations/unilab/check_metrics.py。脚本读取 third_party/UniLab/src，生成 docs/evidence/unilab-metrics-contract.json。当前实现是 CPU 语义原型，不是 GPU 高并发任务实现

check_scene_declaration.py 需要先恢复 I01 manifest 中的资产，校验哈希后检查 UniSim 静态场景能力声明。它不启动 SDK、不证明资产能加载。原检查在 UniSim 2.7.1 下拒绝了 USD entity_assets 多实体路线，包括 isaacsim；这不代表 Isaac Sim 本身不能读 USD。运行后生成新的场景声明报告，不能把能力声明当成实测结果

后续仍需解决机器人、桌面、地面、碰撞孔洞、质量惯性、控制映射、重置与几何判据。源任务失败标志、无效仿真和超时也需要迁移；缺失判据不能默认为 False 或成功
