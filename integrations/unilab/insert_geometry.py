"""Tensor port of RoboDojo's three insert_tubes geometry predicates.

Uses source metadata boxes, not converted collision bounds. These predicates
do not test hole identity, contact, gripper release or robot return.
"""
import math
import torch
from unisim.utils.math import quat_apply


def source_geometry_signals(positions, quaternions, tube_bbox, rack_bbox):
    """Return source depth proxy, inside predicate, and upright angle.

    Shape is (env, three tubes, three values). The second value is 0/1.
    """
    checks = geometry_checks(positions, quaternions, tube_bbox, rack_bbox)
    positions, quaternions = positions.double(), quaternions.double()
    tube_bbox = tube_bbox.to(device=positions.device, dtype=positions.dtype)
    rack_bbox = rack_bbox.to(device=positions.device, dtype=positions.dtype)
    tube_world = quat_apply(quaternions[:, :3, None], tube_bbox) + positions[:, :3, None]
    rack_world = quat_apply(quaternions[:, 3, None], rack_bbox) + positions[:, 3, None]
    depth = rack_world[..., 2].amax(dim=1)[:, None] - tube_world[..., 2].amin(dim=2)
    axis = torch.tensor([0., 1., 0.], dtype=positions.dtype, device=positions.device)
    axis_z = quat_apply(quaternions[:, :3], axis)[..., 2]
    upright_deg = torch.rad2deg(torch.acos(axis_z.clamp(-1, 1)))
    return torch.stack((depth, checks[..., 1].double(), upright_deg), dim=-1)


def geometry_checks(positions, quaternions, tube_bbox, rack_bbox):
    """Return (environment, tube, depth/inside/upright) boolean checks.

Body order is tube0, tube1, tube2, slot; positions are meters, quaternions wxyz.
The polygon test uses all supporting edges of the projected rack box so it
also handles tilted racks, without a CPU polygon library in the step loop.
"""
    if positions.ndim != 3 or positions.shape[1:] != (4, 3):
        raise ValueError('Expected positions shaped (N, 4, 3)')
    if quaternions.shape != (*positions.shape[:2], 4):
        raise ValueError('Expected wxyz quaternions shaped (N, 4, 4)')
    positions, quaternions = positions.double(), quaternions.double()
    tube_bbox = tube_bbox.to(device=positions.device, dtype=positions.dtype)
    rack_bbox = rack_bbox.to(device=positions.device, dtype=positions.dtype)
    if tube_bbox.shape != (8, 3) or rack_bbox.shape != (8, 3):
        raise ValueError('Expected eight source bounding-box vertices')
    if not all(torch.isfinite(value).all() for value in (positions, quaternions, tube_bbox, rack_bbox)):
        raise ValueError('Geometry inputs must be finite')
    if not torch.allclose(quaternions.norm(dim=-1), torch.ones_like(quaternions[..., 0]), atol=1e-5, rtol=0):
        raise ValueError('Geometry requires unit wxyz quaternions')
    tube_world = quat_apply(quaternions[:, :3, None], tube_bbox) + positions[:, :3, None]
    rack_world = quat_apply(quaternions[:, 3, None], rack_bbox) + positions[:, 3, None]
    depth = rack_world[..., 2].amax(dim=1)[:, None] - tube_world[..., 2].amin(dim=2) > .045

    polygon = rack_world[..., :2]
    i, j = torch.triu_indices(8, 8, offset=1, device=positions.device)
    starts, edges = polygon[:, i], polygon[:, j] - polygon[:, i]
    cross_vertices = (edges[:, :, None, 0] * (polygon[:, None, :, 1] - starts[:, :, None, 1])
                      - edges[:, :, None, 1] * (polygon[:, None, :, 0] - starts[:, :, None, 0]))
    eps = torch.finfo(positions.dtype).eps * 16
    nondegenerate = cross_vertices.abs().amax(dim=2) > eps
    positive = (cross_vertices >= -eps).all(dim=2) & nondegenerate
    negative = (cross_vertices <= eps).all(dim=2) & nondegenerate
    cross_points = (edges[:, None, :, 0] * (positions[:, :3, None, 1] - starts[:, None, :, 1])
                    - edges[:, None, :, 1] * (positions[:, :3, None, 0] - starts[:, None, :, 0]))
    inside_xy = ((~positive[:, None] | (cross_points > eps))
                 & (~negative[:, None] | (cross_points < -eps))).all(dim=2)
    inside = (inside_xy & nondegenerate.any(dim=1)[:, None]
              & (positions[:, :3, 2] > rack_world[..., 2].amin(dim=1)[:, None]))
    axis = torch.tensor([0., 1., 0.], dtype=positions.dtype, device=positions.device)
    upright = quat_apply(quaternions[:, :3], axis)[..., 2] > math.cos(math.radians(30))
    return torch.stack((depth, inside, upright), dim=-1)
