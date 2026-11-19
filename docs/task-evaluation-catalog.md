# 21 项任务配置与评价指标初稿

每个链接指向该任务唯一 JSON，重点编辑 benchmark 区域。难度按现有五个维度解释，尚未实测排序。所有配置都是 draft，不能等同于已完成仿真迁移

`source` 表示沿用原终态判据/阈值，episode 时间与终帧汇总仍是本工程草案；复杂布尔特征还需要源几何适配。`proposed` 表示我们自拟的独立子任务或功能代理。完整原评分说明保留在 original_success/provenance 中

| 任务配置 | 参考来源 | 判据来源 | 初步完成条件 |
|---|---|---|---|
| [G01 苹果本体抓取](../tasks/grasp/G01.json) | Bench2Dex 06 | proposed | (height_gain_m ≥ 0.05 且 speed_m_s ≤ 0.05 且 hand_contact = true)；连续 1.0 s |
| [G02 瓶身抓取](../tasks/grasp/G02.json) | Bench2Dex 21 | proposed | (height_gain_m ≥ 0.05 且 speed_m_s ≤ 0.05 且 hand_contact = true 且 tilt_deg ≤ 15)；连续 1.0 s |
| [G03 小电池抓取](../tasks/grasp/G03.json) | Bench2Dex 24 | proposed | (height_gain_m ≥ 0.05 且 speed_m_s ≤ 0.05 且 hand_contact = true)；连续 1.0 s |
| [G04 螺丝刀柄抓取](../tasks/grasp/G04.json) | Bench2Dex 12 | proposed | (height_gain_m ≥ 0.05 且 speed_m_s ≤ 0.05 且 hand_contact = true 且 grasp_region_valid = true)；连续 1.0 s |
| [G05 平底锅柄抓取](../tasks/grasp/G05.json) | Bench2Dex 08 | proposed | (height_gain_m ≥ 0.05 且 speed_m_s ≤ 0.05 且 hand_contact = true 且 grasp_region_valid = true 且 tilt_deg ≤ 15)；连续 1.0 s |
| [G06 捏苹果梗提起苹果](../tasks/grasp/G06.json) | 内部已有测试 | proposed | (height_gain_m ≥ 0.05 且 speed_m_s ≤ 0.05 且 hand_contact = true 且 grasp_region_valid = true)；连续 1.0 s |
| [H01 手内方块重定向](../tasks/in_hand/H01.json) | Gymnasium Robotics Manipulate Block | source | rotation_error_rad < 0.1 |
| [H02 手内笔重定向](../tasks/in_hand/H02.json) | Adroit Pen | source | (position_error_m < 0.075 且 axis_dot > 0.95) |
| [H03 手内蛋形刚体重定向](../tasks/in_hand/H03.json) | Gymnasium Robotics Manipulate Egg | source | rotation_error_rad < 0.1 |
| [I01 插管](../tasks/insertion/I01.json) | RoboDojo Insert Tubes | source | ((tube0_depth_m > 0.045 且 tube0_inside = true 且 tube0_upright_deg < 30) 且 (tube1_depth_m > 0.045 且 tube1_inside = true 且 tube1_upright_deg < 30) 且 (tube2_depth_m > 0.045 且 tube2_inside = true 且 tube2_upright_deg < 30) 且 robots_home = true) |
| [I02 插充电器](../tasks/insertion/I02.json) | RoboDojo Plug In Charger | source | ((charger_depth_m > 0.015 且 charger_inside = true 且 charger_upright_deg < 10) 且 robots_home = true) |
| [I03 钥匙交接、插入并旋转](../tasks/insertion/I03.json) | RoboDojo Insert Key | source | (depth_m > 0.025 且 xy_error_m < 0.007 且 upright_deg < 7 且 turn_alignment_deg < 30) |
| [P01 推T形块](../tasks/push/P01.json) | RoboDojo Push T | source | (xy_error_m < 0.007 且 rotation_error_deg < 7 且 robots_home = true) |
| [P02 推滑物体到目标](../tasks/push/P02.json) | Gymnasium Robotics Fetch Slide | source | position_error_m < 0.05 |
| [P03 扫积木进簸箕](../tasks/push/P03.json) | RoboDojo Sweep Blocks | source | (shovel_left_of_broom = true 且 shovel_not_lifted = true 且 shovel_upright_deg < 5 且 (all_cubes_in_support = true 或 all_cubes_in_functional_bbox = true) 且 robots_home = true) |
| [R01 扳手拧螺母](../tasks/tool_rotation/R01.json) | 待补可复用紧固任务 | proposed | (nut_rotation_rad ≥ 1.0471975512 且 wrench_engaged = true)；连续 0.5 s |
| [R02 开关水龙头](../tasks/tool_rotation/R02.json) | Bench2Dex 67 | adapted_source_thresholds | faucet_joint_rad ≥ 1.0 → faucet_joint_rad < 0.52；终帧仍关闭 |
| [R03 解锁门闩并开门](../tasks/tool_rotation/R03.json) | Adroit Door | source | door_hinge_rad ≥ 1.35 |
| [M01 物体装箱](../tasks/transport/M01.json) | RoboDojo Pack Objects Into Box | source | (car_placed = true 且 electric_toothbrush_placed = true 且 hammer_placed = true 且 shoe_placed = true 且 box_heading_valid = true 且 robots_home = true) |
| [M02 工具归位](../tasks/transport/M02.json) | RoboDojo Store Tools In Toolbox | source | (pliers_placed = true 且 hammer_placed = true 且 tape_measure_placed = true 且 wrench_placed = true 且 robots_home = true) |
| [M03 堆积木](../tasks/transport/M03.json) | RoboDojo Stack Blocks | source | (three_stacked = true 且 robots_home = true) |

G06 苹果梗为低优先级占位。R01 的60度旋转代理不能证明达到紧固扭矩或预紧力。I03 终态条件不证明交接过程，P01 原抬升 query 生命周期尚未移植。M01/M02/M03/P03 的布尔量必须由源算法计算，不可把简化中心距离当作等价判据

每项输出 success、完成时间、失败原因、缺失观测、阶段诊断及配置中列出的 metrics。RoboDojo 原阶段得分仍未在这套通用工具中复刻，缺失物理量保持 null。使用方法见 [本地评分说明](local-task-evaluation.md)
