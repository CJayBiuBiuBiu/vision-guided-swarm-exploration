import unittest

from eavf_mappo.control.local_controller import LocalController
from eavf_mappo.robots.robot_state import RobotState


class LocalControllerTests(unittest.TestCase):
    def test_collision_recovery_commands_reverse_motion_without_path(self):
        controller = LocalController(v_max=0.5, omega_max=1.5)
        robot = RobotState(0, 0.0, 0.0, 0.0, 0.0, 0.0, None, [], True, 1, 0)

        v, omega = controller.compute_command(robot, [], None, [], in_collision=True)

        self.assertLess(v, 0.0)
        self.assertNotEqual(omega, 0.0)


if __name__ == "__main__":
    unittest.main()
