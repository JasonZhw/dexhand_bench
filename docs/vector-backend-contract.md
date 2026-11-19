> 历史材料：2026-11-11 至 2026-11-12 的工程记录，状态与路径反映当时版本；当前范围见仓库 README

# 后续原生并行后端的边界

当前batch已经能运行多个独立CPU进程；它与UniLab原生num_envs不同。以下是预留契约，不是已实现的GPU向量化runner

- num_envs必须是明确的批大小，动作形状为[num_envs, action_dim]；不能把进程数量和批大小混用
- reset接收需要重置的env_ids和各自seed，返回这些环境实际初态；一个环境结束不能悄悄重置其他环境
- step返回每环境reward、terminated、truncated和success（允许unknown）；评分、物体状态和阶段均保留env维度
- 观察接口允许按env_ids和采样频率读取接触。批内接触数量不等时使用offset/length或明确的有效mask，不能用零填充冒充未接触
- synchronize必须等待被计时的物理工作完成；异步GPU提交耗时不能当作完整仿真耗时
- 记录设备、硬件、批大小、控制/物理周期、预热步数、内存、求解参数和渲染/记录开关。记录GPU总显存占用与本进程峰值时必须区分
- 同样seed和动作在单环境/多环境下的结果差异单独保存；发现差异先排查状态隔离、重置和数值顺序，不立即认定求解器错误

原生批runner接入时，应扩展执行层并复用现有任务协议、DiagnosticProbe语义和结果字段；不可把单环境Gym字典转换成tensor后就声称UniLab适配完成
