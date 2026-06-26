from __future__ import annotations

import math
from dataclasses import dataclass, field

import torch
from torch.nn.utils import clip_grad_norm_

from eavf_mappo.marl.rollout_buffer import RolloutBatch
from eavf_mappo.marl.rollout_buffer import iter_minibatches


class RunningMeanStd:
    def __init__(self, eps: float = 1.0e-4):
        self.mean = 0.0
        self.var = 1.0
        self.count = float(eps)

    @property
    def std(self) -> float:
        return math.sqrt(max(self.var, 1.0e-12))

    def update(self, values: torch.Tensor) -> None:
        flat = torch.as_tensor(values, dtype=torch.float32).detach().reshape(-1)
        if flat.numel() == 0:
            return
        batch_mean = float(flat.mean().cpu())
        batch_var = float(flat.var(unbiased=False).cpu()) if flat.numel() > 1 else 0.0
        batch_count = float(flat.numel())
        self._update_from_moments(batch_mean, batch_var, batch_count)

    def _update_from_moments(self, batch_mean: float, batch_var: float, batch_count: float) -> None:
        delta = batch_mean - self.mean
        total_count = self.count + batch_count
        new_mean = self.mean + delta * batch_count / total_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m_2 = m_a + m_b + delta * delta * self.count * batch_count / total_count
        self.mean = new_mean
        self.var = m_2 / total_count
        self.count = total_count


@dataclass
class MAPPOConfig:
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_eps: float = 0.2
    entropy_coef: float = 0.05
    value_coef: float = 0.5
    residual_reg: float = 0.01
    update_epochs: int = 8
    minibatch_size: int = 512
    max_grad_norm: float = 0.5
    normalize_returns: bool = True
    value_clip_eps: float | None = 0.2
    return_norm_eps: float = 1.0e-8
    return_rms: RunningMeanStd = field(default_factory=RunningMeanStd)


def normalize_tensor(tensor, eps: float = 1.0e-8):
    values = torch.as_tensor(tensor, dtype=torch.float32)
    mean = values.mean()
    std = values.std() if values.numel() > 1 else torch.zeros((), dtype=values.dtype, device=values.device)
    if (not bool(torch.isfinite(std))) or float(std.detach().cpu()) <= eps:
        std = torch.ones((), dtype=values.dtype, device=values.device)
        return torch.zeros_like(values), mean, std
    return (values - mean) / (std + eps), mean, std


def compute_value_loss(values, old_values, returns, clip_eps: float | None = None):
    values = torch.as_tensor(values, dtype=torch.float32)
    old_values = torch.as_tensor(old_values, dtype=values.dtype, device=values.device)
    returns = torch.as_tensor(returns, dtype=values.dtype, device=values.device)
    value_loss = (values - returns).pow(2)
    if clip_eps is None:
        return value_loss.mean()
    clipped_values = old_values + torch.clamp(values - old_values, -clip_eps, clip_eps)
    clipped_loss = (clipped_values - returns).pow(2)
    return torch.max(value_loss, clipped_loss).mean()


def compute_gae(rewards, values, dones, gamma: float = 0.99, lam: float = 0.95):
    rewards = torch.as_tensor(rewards, dtype=torch.float32)
    values = torch.as_tensor(values, dtype=torch.float32)
    dones = torch.as_tensor(dones, dtype=torch.float32)
    t_steps = rewards.shape[0]
    if values.shape[0] == t_steps:
        # No explicit bootstrap value was supplied for the state after the
        # last transition. Repeating the last value estimate is a softer,
        # less biased default than forcing 0 (which wrongly assumes the
        # final step is always a true terminal state, even when it is only
        # a rollout truncation).
        values = torch.cat([values, values[-1:]])
    elif values.shape[0] != t_steps + 1:
        raise ValueError("values must have length T or T+1 (T+1 includes the bootstrap value)")

    adv = torch.zeros_like(rewards)
    last_gae = 0.0
    for t in reversed(range(t_steps)):
        not_done = 1.0 - dones[t]
        delta = rewards[t] + gamma * values[t + 1] * not_done - values[t]
        last_gae = delta + gamma * lam * not_done * last_gae
        adv[t] = last_gae
    returns = adv + values[:t_steps]
    return adv, returns


def update(buffer, actor, critic, opt, cfg: MAPPOConfig):
    if isinstance(buffer, (list, tuple)):
        full = _prepare_multi_agent_batch(buffer, cfg)
    else:
        full = buffer.as_batch()
        adv, returns = compute_gae(full.rewards, full.values, full.dones, cfg.gamma, cfg.gae_lambda)
        full.adv = adv
        full.returns = returns
    raw_return_mean = float(full.returns.mean().detach().cpu())
    raw_return_std = float(full.returns.std().detach().cpu()) if full.returns.numel() > 1 else 0.0
    if cfg.normalize_returns:
        cfg.return_rms.update(full.returns)
        return_mean = torch.as_tensor(cfg.return_rms.mean, dtype=full.returns.dtype, device=full.returns.device)
        return_std = torch.as_tensor(
            max(cfg.return_rms.std, cfg.return_norm_eps),
            dtype=full.returns.dtype,
            device=full.returns.device,
        )
        full.returns = (full.returns - return_mean) / return_std
    full.adv, _, _ = normalize_tensor(full.adv)

    stats = []
    for _ in range(cfg.update_epochs):
        for batch in iter_minibatches(full, cfg.minibatch_size):
            logp, entropy, delta = actor.evaluate(
                batch.bev,
                batch.tokens,
                batch.c_analytic,
                batch.mask,
                batch.actions,
                special_mask=batch.special_mask,
            )
            log_ratio = logp - batch.old_logp
            ratio = torch.exp(log_ratio)
            unclipped = ratio * batch.adv
            clipped = torch.clamp(ratio, 1.0 - cfg.clip_eps, 1.0 + cfg.clip_eps) * batch.adv
            actor_loss = -torch.min(unclipped, clipped).mean()

            values = critic(
                batch.global_bev,
                batch.all_poses,
                batch.all_targets,
                batch.coverage,
                batch.collision,
                batch.comm_feat,
            )
            value_loss = compute_value_loss(values, batch.values, batch.returns, cfg.value_clip_eps)
            residual_loss = delta.abs().mean()
            approx_kl = ((ratio - 1.0) - log_ratio).mean()
            loss = (
                actor_loss
                + cfg.value_coef * value_loss
                - cfg.entropy_coef * entropy.mean()
                + cfg.residual_reg * residual_loss
            )
            opt.zero_grad()
            loss.backward()
            clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()), cfg.max_grad_norm)
            opt.step()
            stats.append(
                {
                    "loss": float(loss.detach()),
                    "actor_loss": float(actor_loss.detach()),
                    "value_loss": float(value_loss.detach()),
                    "entropy": float(entropy.mean().detach()),
                    "return_mean": raw_return_mean,
                    "return_std": raw_return_std,
                    "value_pred_mean": float(values.mean().detach()),
                    "approx_kl": float(approx_kl.detach()),
                }
            )
    return stats


def _prepare_multi_agent_batch(buffers, cfg: MAPPOConfig) -> RolloutBatch:
    batches = []
    for buffer in buffers:
        batch = buffer.as_batch()
        adv, returns = compute_gae(batch.rewards, batch.values, batch.dones, cfg.gamma, cfg.gae_lambda)
        batch.adv = adv
        batch.returns = returns
        batches.append(batch)
    return _concat_batches(batches)


def _concat_batches(batches: list[RolloutBatch]) -> RolloutBatch:
    values = {}
    for field in RolloutBatch.__dataclass_fields__.keys():
        parts = [getattr(batch, field) for batch in batches if getattr(batch, field) is not None]
        values[field] = torch.cat(parts, dim=0) if parts else None
    return RolloutBatch(**values)
