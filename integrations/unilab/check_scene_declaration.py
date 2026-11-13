"""Audit the I01 object subset against UniSim's static capability declarations.

Does not import a physics SDK or claim assets load. Collision=True is a proposed
request, not a reconstruction of RoboDojo's merged runtime configuration.
"""
from dataclasses import asdict
from hashlib import sha256
from importlib.metadata import version
import json
from pathlib import Path

from unisim.dr.types import ModelSourceDescriptor
from unisim.entities import EntityInitialState, SceneEntitySpec
from unisim.scene import SceneCfg, require_scene_composition_support
from unisim.adapters import ADAPTER_SPECS
from unisim.capabilities import get_adapter_capabilities

ROOT = Path(__file__).resolve().parents[2]
layout_path = ROOT / 'docs/evidence/robodojo-layouts/insert_tubes_0.json'
layout = json.loads(layout_path.read_text())
manifest = json.loads((ROOT / 'asset_manifests/I01-robodojo.json').read_text())
verified = {}
for item in manifest['files']:
    path = ROOT / item['local_path']
    digest = sha256(path.read_bytes()).hexdigest()
    if digest != item['sha256']:
        raise ValueError(f'Asset checksum mismatch: {path}')
    verified[item['path']] = digest

entities, overrides = [], {}
for kind, category in [('Rigid', 'test_tube'), ('Geometry', 'test_tube_slot')]:
    for item in layout[kind][category]:
        if item['scale'] != [1.0, 1.0, 1.0]:
            raise ValueError('Non-unit scale requires explicit asset conversion')
        asset = f"Assets/Object/RoboDojo/{kind}/{category}/{item['category_idx']:05d}/object.usdz"
        if asset not in verified:
            raise ValueError(f'Asset not verified: {asset}')
        entities.append(SceneEntitySpec(
            name=item['label'],
            source=ModelSourceDescriptor(str(ROOT / 'assets/robodojo' / asset)),
            asset_format='usd', kind='rigid',
            root_mode='floating' if kind == 'Rigid' else 'fixed',
            initial_state=EntityInitialState(position=tuple(item['default_pos']),
                                            quaternion=tuple(item['default_ori'])),
            collision_enabled=True,
        ))
        overrides[item['label']] = item['physics']
scene = SceneCfg(entity_assets=tuple(entities))
results = {}
for spec in ADAPTER_SPECS:
    try:
        require_scene_composition_support(scene, spec.name)
    except NotImplementedError as exc:
        results[spec.name] = {'declaration_accepted': False, 'reason': str(exc)}
    else:
        results[spec.name] = {'declaration_accepted': True}
mjcf_inventory = {}
for spec in ADAPTER_SPECS:
    declaration = get_adapter_capabilities(spec.name).get('entity.multiple', configuration={
        'entity.asset_format': 'mjcf', 'entity.variant': 'none', 'entity.kinematic': 'none',
    })
    mjcf_inventory[spec.name] = {
        'support': declaration.support.value, 'conditions_and_limits': declaration.reason,
    }
receipt = {
    'scope': 'I01 four-object proposed declaration only; no SDK load or simulation',
    'unisim_core': version('unisim-core'),
    'layout_sha256': sha256(layout_path.read_bytes()).hexdigest(),
    'verified_assets': verified,
    'entities': [asdict(entity) for entity in entities],
    'backend_declarations': results,
    'alternative_mjcf_static_inventory_only': mjcf_inventory,
    'unapplied_source_physics': overrides,
    'runnable_task': False,
    'blockers': [
        'Mass/material overrides not applied; resolve merged source runtime configuration',
        'Collision=True is proposed, not verified source behavior; validate SDF holes and contacts',
        'USDZ loader support and transitive asset references not exercised',
        'Robot, table, ground and remaining scene entities not included',
        'No action, reset, observation or geometric success predicates connected',
    ],
}
output = ROOT / 'docs/evidence/unilab-scene-declaration.json'
output.write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps({'output': str(output), 'backend_declarations': results}, indent=2))
