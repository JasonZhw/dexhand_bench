"""Downstream UniLab metric terms; not a scene or a physics backend.

The environment owner updates dexbench_task_state once per evaluation step,
before MetricsManager.compute(), and resets its selected rows on env reset.
"""


def process_score(env):
    import torch
    return torch.tensor([row['score'] for row in env.dexbench_task_state.latest],
                        dtype=torch.float32, device=env.device)


def final_success(env):
    import torch
    return torch.tensor([float(row['success']) for row in env.dexbench_task_state.latest],
                        dtype=torch.float32, device=env.device)


def inserted_count(env):
    import torch
    return torch.tensor([row['inserted_count'] for row in env.dexbench_task_state.latest],
                        dtype=torch.float32, device=env.device)


def make_metrics_config():
    from unilab.managers import MetricsTermCfg
    return {
        'source_process_score': MetricsTermCfg(func=process_score, reduce='last'),
        'source_final_success': MetricsTermCfg(func=final_success, reduce='last'),
        'inserted_tubes': MetricsTermCfg(func=inserted_count, reduce='last'),
    }
