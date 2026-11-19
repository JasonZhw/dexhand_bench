> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# 后续怎样接入 UniLab

本轮检查的是公开仓库 https://github.com/Motphys/UniLab ，commit `794c0bbd67771ac48407e084c0164c0e841bfd97`，不是团队指定的内部版本。该版本pyproject依赖unisim-core==2.7.1。现有服务器Isaac环境不能据此认定与它兼容

公开版本使用manager-based任务运行时和Hydra配置。任务应拆成初始化/重置、动作、观察、终止、指标等term，由owner配置组合；物理实现留在UniSim的SimBackend后面。我们现在的Gymnasium适配器是验证任务和记录协议的过渡入口，还没有接入这些manager

| 当前工程内容 | UniLab接入位置/责任 |
|---|---|
| task JSON中的语义、初态、目标 | 新任务的owner配置与任务term |
| 原成功条件与原奖励分开保存 | termination / reward / metrics manager，不能仅凭reward非零认定成功 |
| 关节顺序、动作范围、控制模式 | action manager及机器人配置 |
| 同种子reset与初态记录 | event manager、reset owner、RNG约定 |
| 物体/关节状态及接触读取 | 声明过的SimBackend能力，不调用后端私有方法 |
| 执行报告与物理诊断 | recorder / metrics及runtime diagnostics扩展 |
| 并发数与同步 | 实际backend向量化契约；目前CPU进程batch不能替代它 |

核对过的路径：`src/unilab/envs/manager_based_rl_env.py`、`src/unilab/base/contracts/env.py`、`src/unilab/managers/`、`src/unilab/conf/`。EnvProtocol采用tensor环境状态和批量动作；reset返回obs/info。我们当前的单环境Python字典不能直接冒充这个接口

迁移时先选一个任务、一个机器人和一个后端，检查资产单位/碰撞/惯性、关节映射、控制频率、重置、原成功判据，再增加第二后端。USD资源存在不代表另一个引擎能直接加载；转换后的几何和惯性必须另存版本和校验值

需要团队后续对齐：实际UniLab分支、目标手与机械臂模型、希望优先比较的两种后端、动作接口与接触读取能力。这些不会阻塞当前任务目录和原任务试跑，但会决定最终adapter实现

2026-11-12：已开始插管任务的评分接口迁移。`dexbench/insert_tubes.py` 保存原过程分及成功记忆，`integrations/unilab/insert_tubes_terms.py` 提供 MetricsTermCfg(reduce="last") 接口。源码评分对照记录在 `docs/evidence/robodojo-score-parity.json`，详细解释在 `docs/robodojo-to-unilab.md`。这还不是可运行的 UniLab 插管环境

2026-11-12 续：真实 MetricsManager 已通过两个环境与部分重置验证。I01 四物体声明和资产哈希检查完成，但 unisim-core2.7.1 的全部已登记后端拒绝当前 USD entity_assets 路径；完整物理迁移仍未运行。下一步优先解决场景装配入口，再接控制，而不是先比较后端分数。详见 integrations/unilab/README.md
