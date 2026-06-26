from __future__ import annotations

import torch
from torch import nn
from torch.distributions import Categorical


class MLP(nn.Module):
    def __init__(self, in_dim: int, out_dim: int, hidden_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, out_dim),
        )

    def forward(self, x):
        return self.net(x)


class SmallCNN(nn.Module):
    def __init__(self, in_ch: int = 5, out_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_ch, 32, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(64, out_dim),
            nn.ReLU(),
        )

    def forward(self, x):
        return self.net(x)


class ResidualFrontierActor(nn.Module):
    def __init__(self, d_token: int, d_model: int = 128, tau: float = 0.5, special_bias_init: float = -2.0):
        super().__init__()
        self.tau = float(tau)
        self.bev_cnn = SmallCNN(in_ch=5, out_dim=d_model)
        self.token_enc = MLP(d_token, d_model, hidden_dim=d_model)
        self.attn = nn.MultiheadAttention(d_model, num_heads=4, batch_first=True)
        self.delta_head = MLP(d_model, 1, hidden_dim=d_model)
        self.special_head = MLP(d_model, 2, hidden_dim=d_model)
        # frontier_logits = -(c_analytic - delta) and c_analytic is a non-negative
        # Voronoi cost, so a real frontier pick starts out with a negative logit.
        # special_head's output has no such scale constraint and defaults to ~0
        # at init, which gives the "continue"/"release" actions an unearned head
        # start over every frontier; the policy can collapse into always
        # releasing before it ever tries exploring. Bias the special logits
        # negative at init so they start on comparable footing with typical
        # normalized frontier costs instead of structurally dominating them.
        with torch.no_grad():
            self.special_head.net[-1].bias.fill_(special_bias_init)

    def forward(self, bev, tokens, c_analytic, mask):
        del c_analytic
        mask = mask.to(dtype=torch.bool, device=tokens.device)
        ctx = self.bev_cnn(bev)
        tok = self.token_enc(tokens)

        query = tok + ctx.unsqueeze(1)
        key_padding_mask = ~mask
        # A row with no valid tokens would mask out every key, which makes
        # nn.MultiheadAttention produce NaN. Temporarily unmask those rows;
        # the result is discarded below anyway since delta gets zeroed there.
        fully_masked = ~mask.any(dim=-1)
        if fully_masked.any():
            key_padding_mask = key_padding_mask.clone()
            key_padding_mask[fully_masked] = False
        attended, _ = self.attn(query=query, key=tok, value=tok, key_padding_mask=key_padding_mask)
        delta = self.delta_head(attended).squeeze(-1)
        delta = self.tau * torch.tanh(delta)
        delta = delta.masked_fill(~mask, 0.0)
        special = self.special_head(ctx)
        return delta, special

    def evaluate(self, bev, tokens, c_analytic, mask, actions, special_bias=None, special_mask=None):
        logits, delta = action_logits(self, bev, tokens, c_analytic, mask, special_bias, special_mask)
        dist = Categorical(logits=logits)
        return dist.log_prob(actions), dist.entropy(), delta


def action_logits(actor, bev, tokens, c_analytic, mask, special_bias=None, special_mask=None):
    delta, special = actor(bev, tokens, c_analytic, mask)
    mask = mask.to(dtype=torch.bool, device=c_analytic.device)
    frontier_logits = -(c_analytic - delta)
    frontier_logits = frontier_logits.masked_fill(~mask, -1.0e9)

    if special_bias is None:
        bias = torch.zeros(2, dtype=special.dtype, device=special.device)
    else:
        bias = torch.as_tensor(special_bias, dtype=special.dtype, device=special.device)
    special_logits = special + bias
    if special_mask is not None:
        special_mask = special_mask.to(dtype=torch.bool, device=special.device)
        special_logits = special_logits.masked_fill(~special_mask, -1.0e9)
    logits = torch.cat([frontier_logits, special_logits], dim=-1)
    return logits, delta


def select_action(actor, bev, tokens, c_analytic, mask, special_bias=None, special_mask=None, training: bool = True):
    logits, _ = action_logits(actor, bev, tokens, c_analytic, mask, special_bias, special_mask)
    dist = Categorical(logits=logits)
    action = dist.sample() if training else torch.argmax(logits, dim=-1)
    return action, dist.log_prob(action), dist.entropy()
