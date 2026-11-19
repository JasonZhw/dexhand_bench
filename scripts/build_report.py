"""Build a local, static progress report from the catalog and recorded evidence."""
from pathlib import Path
import html
import json
import re

ROOT = Path(__file__).resolve().parents[1]
def read(path): return json.loads(path.read_text(encoding='utf-8'))
def esc(value): return html.escape(str(value))

tasks = [read(p) for p in sorted((ROOT/'tasks').rglob('*.json'))]
labels = {'grasp':'抓取','transport':'搬运与放置','push':'推动与滑动',
          'tool_rotation':'工具与约束旋转','in_hand':'手内调整','insertion':'插入'}
status = {'selected':'候选待核对','source_reviewed':'来源已核对','scene_ready':'场景已核验',
          'smoke_executed':'原环境已试跑','baseline_verified':'参考策略已验证'}
bands = {'introductory':'入门候选','intermediate':'中等候选','challenging':'挑战候选'}
rows=[]
for t in tasks:
    difficulty=t['difficulty']
    notes='；'.join(difficulty.get('dimension_notes',{}).values())
    source=t['source'];source_link=f'<a href="{esc(source["url"])}">{esc(source["project"])}</a>' if source.get('url') else esc(source['project'])
    details=f'<details><summary>接触与难度解释</summary><p>{esc(t.get("contact_structure","待核对"))}</p><p>{esc(notes)}</p><p>来源类型：{esc(source["kind"])} · 版本：{esc((source.get("revision") or "待固定")[:12])}</p></details>'
    rows.append(f'<tr><td>{esc(t["id"])}</td><td><strong>{esc(t["name"])}</strong>{details}</td><td>{labels[t["category"]]}</td><td>{bands.get(difficulty.get("proposed_band"),"待定")}</td><td>{status[t["status"]]}</td><td>{source_link}</td></tr>')
video_cards=[]
for task in ['H01','H02','H03','R03','P02']:
    name=next(t['name'] for t in tasks if t['id']==task)
    video_cards.append(f'<article><h3>{task} {esc(name)}</h3><video controls preload="metadata" poster="evidence/previews/{task}_start.png" src="evidence/videos/{task}-zero-smoke.mp4"></video><p>本机原环境 · 零动作试跑 · 未完成任务</p></article>')
reviewed=sum(t['status']!='selected' for t in tasks);running=sum(t['status']=='smoke_executed' for t in tasks)
testlog=(ROOT/'docs/evidence/test-suite.log').read_text();test_count=re.search(r'Ran (\d+) tests',testlog).group(1)
page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>灵巧操作 Benchmark · 工程进展</title>
<style>body{{margin:0;background:#f5f7fa;color:#21344a;font-family:system-ui,"Noto Sans CJK SC",sans-serif;line-height:1.7}}main{{max-width:1200px;margin:auto;padding:42px 26px}}h1{{font-size:34px;line-height:1.3}}h2{{margin-top:42px}}.lead{{font-size:19px;max-width:980px}}.stats,.grid{{display:grid;grid-template-columns:repeat(4,1fr);gap:16px}}.stats div,article,.note{{background:white;border:1px solid #dce3ec;border-radius:12px;padding:18px}}.stats strong{{display:block;font-size:30px;color:#2462a0}}.note{{border-left:5px solid #dca24e;margin:20px 0}}.grid{{grid-template-columns:repeat(2,1fr)}}video{{width:100%;border-radius:8px}}article p{{font-size:14px;color:#64748b}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{padding:13px;text-align:left;border-bottom:1px solid #dde5ee;vertical-align:top}}th{{background:#e8eef5}}td:first-child{{white-space:nowrap}}details{{font-size:13px;margin-top:8px;max-width:440px}}a{{color:#1d5a9d}}.table{{overflow:auto}}code{{background:#e7edf5;padding:3px 5px}}@media(max-width:700px){{.stats,.grid{{grid-template-columns:1fr 1fr}}main{{padding:20px 12px}}.grid{{grid-template-columns:1fr}}}}</style><main>
<p>工程记录 · 2026-11-11 · 本地初版</p><h1>灵巧操作 Benchmark</h1><p class="lead">用不同操作任务覆盖抓取容差、负载与重心、转矩、接触切换和双手协同。先核对原任务，再做灵巧手适配，最后迁入 UniLab</p>
<div class="stats"><div><strong>21</strong>六类候选任务</div><div><strong>{reviewed}</strong>已核对固定来源</div><div><strong>{running}</strong>原环境已试跑</div><div><strong>{test_count}</strong>测试通过</div></div>
<div class="note"><strong>现在能做什么：</strong>真实环境的加载、重置、步进、动作重放、状态与接触记录、参数覆盖、轨迹对照、CPU进程批跑<br><strong>还不能宣称什么：</strong>成功策略基线、21项全部可运行、目标灵巧手适配完成、UniLab或多物理后端已接入</div>
<p><a href="quickstart.md">运行命令</a> · <a href="physics-extension.md">物理测评接口</a> · <a href="asset-gaps.md">资产缺口</a> · <a href="unilab-migration.md">UniLab迁移</a> · <a href="framework.md">框架</a></p>
<h2>已有的运行证据</h2><p>每项运行零动作seed0、随机动作seed0/1，共15次，程序均正常结束，但任务均失败。下面是实际生成的试跑视频，不是官方成功演示。当前运行覆盖手内调整、旋转、推动三个类别，另外三类还需接通</p><div class="grid">{''.join(video_cards)}</div>
<h2>物理测评接口已经试过什么</h2><p>H01同seed、同动作重放100步，记录的刚体位置与朝向差异为0。求解迭代数从20改为1后轨迹变化，但reset后的初态也变了，因此明确标记为有初态干扰，不能把差异直接叫作物理误差。1/2个CPU worker的批跑均完成4个episode，只用于验证并发链路，不报告GPU扩展能力</p>
<p>插管架、钥匙孔和插座的官方USD声明SDF碰撞。换后端时若用单个凸包填平孔洞，任务本身就改变了；资产表示必须作为对照条件记录</p>
<h2>任务目录</h2><p>“入门/中等/挑战”是设计阶段的覆盖假设，尚未经过实验排名。不同来源、机器人和完成条件不能只按一个成功率直接比较</p><div class="table"><table><thead><tr><th>ID</th><th>任务</th><th>类型</th><th>难度假设</th><th>状态</th><th>来源</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div>
<h2>下一批工作</h2><p>优先补推T、插管和搬运场景的完整依赖，保留原评分，再进行灵巧手改编。内部苹果梗需要同事的实际模型和参数；扳手紧固需要匹配的机械结构，普通扳手搬运资产不能直接替代</p></main></html>'''
(ROOT/'docs/progress.html').write_text(page,encoding='utf-8')
print(ROOT/'docs/progress.html')
