"""Exercise real UniLab managers with synthetic task predicates, without physics."""
from pathlib import Path
import json
import sys
from types import SimpleNamespace
from importlib.metadata import version

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'third_party/UniLab'
sys.path[:0] = [str(ROOT), str(SOURCE / 'src')]

import torch
from unilab.managers import MetricsManager
from dexbench.insert_tubes import InsertTubesProgress, TubeChecks
from insert_tubes_terms import make_metrics_config

env = SimpleNamespace(num_envs=2, device=torch.device('cpu'),
                      cfg=SimpleNamespace(defer_finite_validation=False), scene={},
                      dexbench_task_state=InsertTubesProgress(2))
manager = MetricsManager(make_metrics_config(), env)
all_inserted = TubeChecks([True]*3, [True]*3, [True]*3, True, True)
one_inserted = TubeChecks([True, False, False], [True]*3, [True]*3, True, False)
for _ in range(3):
    env.dexbench_task_state.update(0, all_inserted)
    env.dexbench_task_state.update(1, one_inserted)
    manager.compute()

first = manager.reset(torch.tensor([0], dtype=torch.int64))
assert first['Episode_Metrics/source_process_score'] == 100
assert first['Episode_Metrics/source_final_success'] == 1
env.dexbench_task_state.reset([0])
assert env.dexbench_task_state.stages == [0, 1]
second = manager.reset(torch.tensor([1], dtype=torch.int64))
assert second['Episode_Metrics/source_process_score'] == 20
assert second['Episode_Metrics/source_final_success'] == 0

receipt = dict(passed=True, scope='real UniLab MetricsManager with synthetic predicates; no simulator',
               num_envs=2, partial_reset_verified=True, first=first, second=second,
               packages={n: version(n) for n in ('torch', 'unisim-core', 'prettytable')},
               unilab_source=str(SOURCE),
               limitations=['No physical scene', 'No geometry predicate migration',
                            'No robot control', 'CPU contract fixture, not GPU-vectorized task implementation'])
(ROOT / 'docs/evidence/unilab-metrics-contract.json').write_text(json.dumps(receipt, indent=2)+'\n')
print(json.dumps(receipt, indent=2))
