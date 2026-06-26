from __future__ import annotations

import numpy as np
import torch
from torch import nn


class CommunicationScheduler(nn.Module):
    def __init__(self, d_token: int, extra_feat: int = 3, hidden_dim: int = 64):
        super().__init__()
        self.d_token = int(d_token)
        self.extra_feat = int(extra_feat)
        self.score_head = nn.Sequential(
            nn.Linear(self.d_token + self.extra_feat, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )

    def importance(self, token, robot, comm_state=None):
        del comm_state
        token_vec = self._fit_token_vector(token.to_vector())
        intent_feat = 1.0 if getattr(robot, "target_frontier_id", None) is not None else 0.0
        extra = np.asarray(
            [token.information_gain, token.overlap_risk, intent_feat],
            dtype=np.float32,
        )
        feat = np.concatenate([token_vec, extra], axis=0)
        device = next(self.parameters()).device
        tensor = torch.as_tensor(feat, dtype=torch.float32, device=device).unsqueeze(0)
        return torch.sigmoid(self.score_head(tensor)).squeeze()

    def schedule(self, robot_messages, scores, B_max: float, msg_size: float):
        if msg_size <= 0:
            raise ValueError("msg_size must be positive")
        score_values = torch.as_tensor(scores, dtype=torch.float32).detach().cpu().numpy()
        order = np.argsort(-score_values)
        sent = []
        used = 0.0
        for idx in order:
            if used + msg_size <= B_max:
                sent.append(robot_messages[int(idx)])
                used += msg_size
        return sent

    def _fit_token_vector(self, vector):
        vector = np.asarray(vector, dtype=np.float32)
        if vector.shape[0] == self.d_token:
            return vector
        if vector.shape[0] > self.d_token:
            return vector[: self.d_token]
        padded = np.zeros(self.d_token, dtype=np.float32)
        padded[: vector.shape[0]] = vector
        return padded
