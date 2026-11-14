"""State-only terms for scene diagnostics; no manipulation success claim."""
import torch
import hashlib
import json
from pathlib import Path

from integrations.unilab.insert_geometry import geometry_checks

ROOT = Path(__file__).resolve().parents[2]


class ObjectPose:
    def __init__(self, cfg, env):
        self.entity_name = cfg.params['entity_name']
        self.tensor_body_names = tuple(cfg.params['body_names'])
        if not self.tensor_body_names or len(set(self.tensor_body_names)) != len(self.tensor_body_names):
            raise ValueError('Declare nonempty, unique body names')

    def view(self, env):
        return env.scene.read_plan.body_tensor_view(
            env.scene[self.entity_name], self.tensor_body_names)

    def __call__(self, env, entity_name, body_names):
        if env.scene.read_plan is None:
            # ObservationManager probes dimensions before compiling its read plan.
            return torch.zeros((env.num_envs, len(self.tensor_body_names) * 7), device=env.device)
        state = self.view(env)
        return torch.cat((state.pos_w, state.quat_w), dim=-1).flatten(start_dim=1)


class MeanHeight(ObjectPose):
    def __call__(self, env, entity_name, body_names):
        return self.view(env).pos_w[..., 2].mean(dim=1)


class SourceGeometry(ObjectPose):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if self.tensor_body_names != ('tube0', 'tube1', 'tube2', 'slot'):
            raise ValueError('Source geometry requires tube0, tube1, tube2, slot in order')
        manifest = json.loads((ROOT / 'asset_manifests/I01-robodojo.json').read_text())
        boxes = {}
        for entry in manifest['files']:
            path = ROOT / entry['local_path']
            if path.name != 'metadata.json':
                continue
            raw = path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != entry['sha256']:
                raise ValueError(f'Source metadata checksum mismatch: {path}')
            boxes[path.parent.parent.name] = torch.tensor(
                json.loads(raw)['geometry']['oriented_bbox']['vertices'],
                dtype=torch.float64, device=env.device)
        self.tube_bbox, self.rack_bbox = boxes['test_tube'], boxes['test_tube_slot']

    def checks(self, env):
        state = self.view(env)
        return geometry_checks(state.pos_w, state.quat_w, self.tube_bbox, self.rack_bbox)

    def __call__(self, env, entity_name, body_names):
        if env.scene.read_plan is None:
            return torch.zeros((env.num_envs, 9), device=env.device)
        return self.checks(env).float().flatten(start_dim=1)


class SourceGeometryCount(SourceGeometry):
    def __call__(self, env, entity_name, body_names):
        return self.checks(env).all(dim=-1).sum(dim=-1).float()
