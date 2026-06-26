from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass
class RolloutBatch:
    bev: torch.Tensor
    tokens: torch.Tensor
    c_analytic: torch.Tensor
    mask: torch.Tensor
    actions: torch.Tensor
    old_logp: torch.Tensor
    rewards: torch.Tensor
    dones: torch.Tensor
    values: torch.Tensor
    returns: torch.Tensor | None = None
    adv: torch.Tensor | None = None
    delta: torch.Tensor | None = None
    special_mask: torch.Tensor | None = None
    global_bev: torch.Tensor | None = None
    all_poses: torch.Tensor | None = None
    all_targets: torch.Tensor | None = None
    coverage: torch.Tensor | None = None
    collision: torch.Tensor | None = None
    comm_feat: torch.Tensor | None = None


class RolloutBuffer:
    def __init__(self):
        self.items: list[RolloutBatch] = []

    def add(self, batch: RolloutBatch) -> None:
        self.items.append(batch)

    def as_batch(self) -> RolloutBatch:
        if not self.items:
            raise ValueError("rollout buffer is empty")
        fields = RolloutBatch.__dataclass_fields__.keys()
        values = {}
        for field in fields:
            present = [getattr(item, field) is not None for item in self.items]
            if any(present) and not all(present):
                raise ValueError(
                    f"field '{field}' is set on some rollout items but not others; "
                    "concatenating would misalign the batch"
                )
            parts = [getattr(item, field) for item in self.items if getattr(item, field) is not None]
            values[field] = torch.cat(parts, dim=0) if parts else None
        return RolloutBatch(**values)

    def minibatches(self, minibatch_size: int):
        yield from iter_minibatches(self.as_batch(), minibatch_size)


def slice_batch(batch: RolloutBatch, idx) -> RolloutBatch:
    values = {}
    for field in RolloutBatch.__dataclass_fields__.keys():
        value = getattr(batch, field)
        values[field] = value[idx] if value is not None else None
    return RolloutBatch(**values)


def iter_minibatches(batch: RolloutBatch, minibatch_size: int):
    total = batch.actions.shape[0]
    indices = torch.randperm(total, device=batch.actions.device)
    for start in range(0, total, minibatch_size):
        idx = indices[start : start + minibatch_size]
        yield slice_batch(batch, idx)
