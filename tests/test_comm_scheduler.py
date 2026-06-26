import unittest

import torch

from eavf_mappo.communication.communication_scheduler import CommunicationScheduler
from eavf_mappo.communication.message import FrontierMessage
from eavf_mappo.exploration.frontier_token import FrontierToken
from eavf_mappo.robots.robot_state import RobotState


class CommunicationSchedulerTests(unittest.TestCase):
    def test_schedule_respects_bandwidth_and_score_order(self):
        messages = [
            FrontierMessage(i, (0.0, 0.0), i, (float(i), 0.0), 0.0, 0.0, "exploring", 0)
            for i in range(4)
        ]
        scores = torch.tensor([0.1, 0.9, 0.3, 0.7])

        sent = CommunicationScheduler(d_token=10).schedule(messages, scores, B_max=2, msg_size=1)

        self.assertEqual([m.robot_id for m in sent], [1, 3])

    def test_importance_returns_probability(self):
        token = FrontierToken(0, 0.0, 0.0, 1.0, 1.0, 5.0, 0.1, 0.2, 0.3, 0.4, 0)
        robot = RobotState(0, 0.0, 0.0, 0.0, 0.0, 0.0, None, [], False, 0, 0)
        scheduler = CommunicationScheduler(d_token=10)

        score = scheduler.importance(token, robot, {})

        self.assertGreaterEqual(score.item(), 0.0)
        self.assertLessEqual(score.item(), 1.0)


if __name__ == "__main__":
    unittest.main()
