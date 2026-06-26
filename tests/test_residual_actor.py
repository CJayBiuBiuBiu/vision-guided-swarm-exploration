import unittest

import torch
from torch import nn

from eavf_mappo.marl.actor import ResidualFrontierActor, select_action


class ZeroDeltaActor(nn.Module):
    def forward(self, bev, tokens, c_analytic, mask):
        batch, k = c_analytic.shape
        return torch.zeros(batch, k), torch.full((batch, 2), -100.0)


class SpecialDominantActor(nn.Module):
    def forward(self, bev, tokens, c_analytic, mask):
        batch, k = c_analytic.shape
        return torch.zeros(batch, k), torch.full((batch, 2), 100.0)


class ResidualActorTests(unittest.TestCase):
    def test_forward_returns_bounded_delta_and_special_logits(self):
        actor = ResidualFrontierActor(d_token=10, d_model=32, tau=0.25)
        bev = torch.zeros(2, 5, 64, 64)
        tokens = torch.randn(2, 4, 10)
        cost = torch.zeros(2, 4)
        mask = torch.tensor([[True, True, False, False], [True, False, True, False]])

        delta, special = actor(bev, tokens, cost, mask)

        self.assertEqual(delta.shape, (2, 4))
        self.assertEqual(special.shape, (2, 2))
        self.assertLessEqual(delta.abs().max().item(), 0.25 + 1e-6)
        self.assertEqual(delta[0, 2].item(), 0.0)

    def test_special_head_starts_with_negative_bias(self):
        actor = ResidualFrontierActor(d_token=10, d_model=32, tau=0.25)

        bias = actor.special_head.net[-1].bias.detach()

        self.assertTrue(torch.allclose(bias, torch.full_like(bias, -2.0)))

    def test_zero_delta_greedy_action_equals_analytic_argmin(self):
        actor = ZeroDeltaActor()
        bev = torch.zeros(1, 5, 64, 64)
        tokens = torch.zeros(1, 4, 10)
        cost = torch.tensor([[3.0, 1.0, 0.1, 2.0]])
        mask = torch.tensor([[True, True, False, True]])

        action, _, _ = select_action(actor, bev, tokens, cost, mask, training=False)

        self.assertEqual(action.item(), 1)

    def test_special_mask_can_disable_noop_actions(self):
        actor = SpecialDominantActor()
        bev = torch.zeros(1, 5, 64, 64)
        tokens = torch.zeros(1, 4, 10)
        cost = torch.tensor([[0.2, 1.0, 2.0, 3.0]])
        mask = torch.tensor([[True, True, False, False]])
        special_mask = torch.tensor([[False, False]])

        action, _, _ = select_action(actor, bev, tokens, cost, mask, special_mask=special_mask, training=False)

        self.assertEqual(action.item(), 0)


if __name__ == "__main__":
    unittest.main()
