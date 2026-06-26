import unittest

from eavf_mappo.exploration.frontier_token import FrontierToken
from eavf_mappo.marl.action_adapter import CONTINUE_ACTION, RELEASE_ACTION, decode_action


def token(frontier_id: int, valid: bool = True):
    return FrontierToken(
        frontier_id=frontier_id,
        x=0.0,
        y=0.0,
        distance_to_robot=0.0,
        path_cost=0.0,
        information_gain=0.0,
        obstacle_risk=0.0,
        collision_risk=0.0,
        overlap_risk=0.0,
        comm_uncertainty=0.0,
        assigned_flag=0,
        valid=valid,
    )


class ActionAdapterTests(unittest.TestCase):
    def test_frontier_actions_decode_to_frontier_ids(self):
        topk = [token(12), token(42), token(-1, valid=False)]

        self.assertEqual(decode_action(0, topk), 12)
        self.assertEqual(decode_action(1, topk), 42)

    def test_special_actions_decode_to_env_sentinels(self):
        topk = [token(12), token(42)]

        self.assertEqual(decode_action(2, topk), CONTINUE_ACTION)
        self.assertEqual(decode_action(3, topk), RELEASE_ACTION)

    def test_invalid_token_action_falls_back_to_continue(self):
        topk = [token(12), token(-1, valid=False)]

        self.assertEqual(decode_action(1, topk), CONTINUE_ACTION)
        self.assertEqual(decode_action(99, topk), CONTINUE_ACTION)


if __name__ == "__main__":
    unittest.main()
