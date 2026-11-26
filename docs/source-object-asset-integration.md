# P01、I03 和 M02 原物体资产接入

目前 21 项任务都能打开场景，其中 P01 推 T 块、I03 插钥匙和 M02 工具归位已从自建基础形状切换到 RoboDojo 官方数据中的原物体网格。固定版本的文件清单分别校验通过 6/6、4/4、10/10 项。三项在 UniLab 的 MuJoCo 后端各用两个环境完成加载、两步推进和独立重置。[21 项当前审计](evidence/source-task-assets-20261126/all-tasks-audit.json)明确区分这 3 项、11 项代理场景、2 项插入场景诊断和 5 项本地抓取场景。

![P01 原 T 块和目标垫的本地初态](evidence/source-task-assets-20261126/P01/initial.png)

![I03 原钥匙和槽的本地初态](evidence/source-task-assets-20261126/I03/initial.png)

![M02 原工具与工具箱的本地初态](evidence/source-task-assets-20261126/M02/initial.png)

这仍是**源物体场景诊断**，不是原任务复现。我们固定了方便检查的初态，保持双 X5/Wuji、关节探针和统一 UniLab Viser 入口；还没有移植 RoboDojo 的随机 reset、任务策略、评分信号或原物理后端。运行回执把成功率和所有未接通的评分输入留为 `null`，不把“场景能动”当成“任务完成”。

接触资产也经过了明确的近似转换。P01 官方 T 块 USD 的碰撞网格标记为关闭，因此这里从原 T 网格尺寸恢复了两个接触盒；它们的组合体积与原 T 网格一致，目标垫只渲染、不参与碰撞。I03 的钥匙和槽在源 USD 使用 SDF 碰撞；MuJoCo 场景改用 CoACD 多凸块，槽生成 48 块时仍超过设定凹度阈值，钥匙孔的有效间隙尚不能认为与源环境一致。I03 还把元数据中 0 的摩擦系数改为 0.45 供当前诊断使用。这些差异都必须先单独对齐，再把任务成绩用于后端比较。

M02 的钳子、锤子、卷尺、扳手和工具箱都来自已校验的源 USD。为了让五个高面数物体可在当前场景中显示，工具网格分别简化到约 2 万面、工具箱到 3 万面，最大包围盒偏差小于 0.14 毫米；源文件本身不变。**碰撞不是源模型**：四件工具暂用源网格外接盒，工具箱暂用开放托盘五盒，内部槽形没有建出来。扳手质量和摩擦取自元数据；钳子、锤子、卷尺的元数据缺少质量和摩擦，当前数值属于明确标注的诊断假设。因此 M02 现在能查场景与运行链路，不能用来评判工具是否真的放入指定槽位或比较物理后端。转换细节见 [M02 场景回执](evidence/source-task-assets-20261126/M02/scene.json)。

本机从已下载资产重建场景：

```bash
cd /home/jason77/dexhand_bench
uv venv .venv-assets --python 3.10
uv pip install --python .venv-assets/bin/python -r requirements/scene-assets.txt
uv run --locked python integrations/unilab/task_assets.py audit --task P01
uv run --locked python integrations/unilab/task_assets.py audit --task I03
uv run --locked python integrations/unilab/task_assets.py audit --task M02
.venv-assets/bin/python integrations/unilab/build_source_asset_scenes.py --task P01
.venv-assets/bin/python integrations/unilab/build_source_asset_scenes.py --task I03
.venv-assets/bin/python integrations/unilab/build_source_asset_scenes.py --task M02
uv run --locked python integrations/unilab/play_task.py --task M02 --port 8080
```

新机器上需要先按仓库说明恢复 X5/Wuji 资产，并下载任务源文件；M02 约 92 MB，手动下载时使用 `task_assets.py fetch --task M02 --max-total-mb 100`。构建脚本为避免覆盖已验证结果，会拒绝写入已有输出目录。`scene.json` 保存源 USD、元数据、转换后网格、机器人和场景的哈希。运行回执分别见 [P01](evidence/source-task-assets-20261126/P01/receipt.json)、[I03](evidence/source-task-assets-20261126/I03/receipt.json)与 [M02](evidence/source-task-assets-20261126/M02/receipt.json)。

从新机器完整检查“源文件 → 转换后场景 → UniLab 两步运行”，在机器人与两套 Python 环境准备好后执行 `uv run --locked python scripts/verify_source_asset_integration.py --fetch`。这个检查会在缺少源文件、场景哈希失效或独立重置失败时退出报错；已有场景不会被覆盖。默认单元测试没有下载这些大资产，所以不能代替这条集成检查。

剩余 11 项仍使用基础形状。M01、M03、P03 的已有清单没有文件哈希或大小，需先固定版本和校验后再下载；R02 的 Bench2Dex 资源路径尚未形成可校验的本地物体清单。G06 苹果梗和 R01 扳手紧固需要内部或新建工程资产。H01–H03、P02、R03 可从对应的 Gymnasium Robotics/Adroit 资源继续适配。

I03 的 CoACD 转换即使固定随机种子，重复构建仍可能得到不同的多凸块与场景哈希。回执记录当前具体场景的哈希；比较不同物理后端之前，需要冻结并共享同一份转换后的碰撞资产，不能只给转换参数。
