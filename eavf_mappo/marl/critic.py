from __future__ import annotations

import torch
from torch import nn

from eavf_mappo.marl.actor import SmallCNN


class CentralizedCritic(nn.Module):
    def __init__(self, d_model: int = 128):
        super().__init__()
        self.global_bev_cnn = SmallCNN(in_ch=5, out_dim=d_model)
        self.agg = nn.Sequential(
            nn.LazyLinear(d_model),
            nn.ReLU(),
            nn.Linear(d_model, d_model),
            nn.ReLU(),
        )
        self.value_head = nn.Linear(d_model * 2, 1)

    def forward(self, global_bev, all_poses, all_targets, coverage, collision, comm_feat):
        ctx = self.global_bev_cnn(global_bev)
        aux = torch.cat(
            [
                _flatten(all_poses, ctx),
                _flatten(all_targets, ctx),
                _flatten(coverage, ctx),
                _flatten(collision, ctx),
                _flatten(comm_feat, ctx),
            ],
            dim=-1,
        )
        aux_ctx = self.agg(aux)
        return self.value_head(torch.cat([ctx, aux_ctx], dim=-1)).squeeze(-1)


def _flatten(value, reference):
    tensor = torch.as_tensor(value, dtype=reference.dtype, device=reference.device)
    batch = reference.shape[0]
    if tensor.ndim == 0:
        return tensor.view(1, 1).expand(batch, 1)
    if tensor.shape[0] != batch:
        raise ValueError(f"expected a leading dimension of {batch}, got shape {tuple(tensor.shape)}")
    return tensor.reshape(batch, -1)
