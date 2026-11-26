"""UniLab observations for one free grasp object."""
import torch
from integrations.unilab.object_scene_terms import ObjectPose


class GraspObjectState(ObjectPose):
    def __call__(self, env, entity_name, body_names):
        if env.scene.read_plan is None:
            return torch.zeros((env.num_envs, 13), device=env.device)
        view = self.view(env)
        return torch.cat((view.pos_w, view.quat_w, view.lin_vel_w, view.ang_vel_w), dim=-1).flatten(start_dim=1)
