"""Gripper Command's kernel. Run from the repo root:
    PYTHONPATH=common:utilities/gripper_command python3 -m unittest discover -s utilities/gripper_command/test
"""
import unittest

from gripper_command.kernel import GripperCommandKernel


def kernel(**params):
    emitted = []
    k = GripperCommandKernel("grip", {"joint_id": "jaw", "open_position": 0.0, "closed_position": 0.02, **params},
                             on_emit=lambda pin, data: emitted.append((pin, data)), on_log=lambda _m: None)
    return k, emitted


def press(k, down=True):
    k.process_input("button", {"data": down})


def positions(emitted):
    return [d["positions"][0] for pin, d in emitted if pin == "trajectory"]


class ToggleTest(unittest.TestCase):
    def test_each_press_flips_and_releases_do_nothing(self):
        k, out = kernel()
        press(k)
        press(k, False)
        press(k)
        press(k, False)
        self.assertEqual(positions(out), [0.02, 0.0])

    def test_a_held_button_repeating_true_is_one_press(self):
        k, out = kernel()
        for _ in range(5):
            press(k)
        self.assertEqual(positions(out), [0.02])

    def test_trajectory_is_addressed_to_the_joint(self):
        k, out = kernel()
        press(k)
        self.assertEqual(out[0], ("trajectory", {"name": ["jaw"], "positions": [0.02], "velocities": [], "effort": []}))
        self.assertEqual(out[1], ("closed", {"data": True}))

    def test_start_closed_and_invert(self):
        k, out = kernel(start_closed=True)
        press(k)
        self.assertEqual(positions(out), [0.0])
        k, out = kernel(invert=True)
        press(k)
        self.assertEqual(positions(out), [0.0])


class HoldTest(unittest.TestCase):
    def test_closed_while_held(self):
        k, out = kernel(mode="hold")
        press(k)
        press(k, False)
        self.assertEqual(positions(out), [0.02, 0.0])


class MissingLimitsTest(unittest.TestCase):
    def test_sends_nothing_without_the_joints_limits(self):
        logs = []
        k = GripperCommandKernel("grip", {"joint_id": "jaw"}, on_emit=lambda *a: logs.append(("emit", a)), on_log=logs.append)
        press(k)
        self.assertFalse([x for x in logs if isinstance(x, tuple)])
        self.assertTrue(any("travel limits" in str(x) for x in logs))


if __name__ == "__main__":
    unittest.main()
