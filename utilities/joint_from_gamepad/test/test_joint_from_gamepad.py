"""Joint From Gamepad's kernel. Run from the repo root:
    PYTHONPATH=common:utilities/joint_from_gamepad python3 -m unittest discover -s utilities/joint_from_gamepad/test
"""
import unittest

from joint_from_gamepad.kernel import JointFromGamepadKernel


def kernel(**params):
    emitted = []
    k = JointFromGamepadKernel(
        "pan_jog",
        {"joint_id": "pan", "speed": 1.0, "rate_hz": 50, "min_position": -1.0, "max_position": 1.0, "max_lead": 0, **params},
        on_emit=lambda pin, data: emitted.append((pin, data)),
        on_log=lambda _m: None,
    )
    return k, emitted


def stick(k, **axes):
    k.process_input("axes", {"left_x": 0.0, "left_y": 0.0, "right_x": 0.0, "right_y": 0.0, "connected": True, **axes})


def state(k, position, name="pan"):
    k.process_input("joint_state", {"name": [name], "position": [position], "velocity": [0.0], "effort": [0.0]})


def tick(k, n=1):
    for _ in range(n):
        k.poll()


def targets(emitted):
    return [d["positions"][0] for pin, d in emitted if pin == "trajectory"]


class StickIsASpeedTest(unittest.TestCase):
    def test_full_stick_for_one_second_moves_speed_radians(self):
        k, out = kernel()
        state(k, 0.0)
        stick(k, left_x=1.0)
        tick(k, 50)
        self.assertAlmostEqual(targets(out)[-1], 1.0, places=6)
        self.assertEqual(len(targets(out)), 50)

    def test_half_stick_half_speed(self):
        k, out = kernel()
        state(k, 0.0)
        stick(k, left_x=0.5)
        tick(k, 50)
        self.assertAlmostEqual(targets(out)[-1], 0.5, places=6)

    def test_trajectory_is_addressed_to_the_joint(self):
        k, out = kernel()
        stick(k, left_x=1.0)
        tick(k)
        self.assertEqual(out[0], ("trajectory", {"name": ["pan"], "positions": [0.02], "velocities": [], "effort": []}))


class DirectionTest(unittest.TestCase):
    def test_stick_up_is_positive(self):
        # Raw stick Y is positive pushed DOWN.
        k, out = kernel(axis="left_y")
        stick(k, left_y=-1.0)
        tick(k)
        self.assertGreater(targets(out)[0], 0)

    def test_invert(self):
        k, out = kernel(invert=True)
        stick(k, left_x=1.0)
        tick(k)
        self.assertLess(targets(out)[0], 0)

    def test_only_the_chosen_axis(self):
        k, out = kernel(axis="right_x")
        stick(k, left_x=1.0, left_y=1.0)
        tick(k, 5)
        self.assertEqual(targets(out), [])


class RestAndLimitsTest(unittest.TestCase):
    def test_sends_nothing_at_rest(self):
        k, out = kernel()
        state(k, 0.3)
        stick(k)
        tick(k, 10)
        self.assertEqual(out, [])

    def test_each_push_starts_from_the_joints_real_position(self):
        k, out = kernel()
        state(k, 0.0)
        stick(k, left_x=1.0)
        tick(k, 10)  # target 0.2
        stick(k)
        tick(k)
        state(k, 0.5)  # something else moved it meanwhile (a slider, a test)
        stick(k, left_x=1.0)
        tick(k)
        self.assertAlmostEqual(targets(out)[-1], 0.52, places=6)

    def test_clamped_to_the_joints_travel(self):
        k, out = kernel()
        state(k, 0.9)
        stick(k, left_x=1.0)
        tick(k, 50)
        self.assertEqual(targets(out)[-1], 1.0)
        k, out = kernel()
        stick(k, left_x=-1.0)
        tick(k, 100)
        self.assertEqual(targets(out)[-1], -1.0)

    def test_a_lagging_joint_doesnt_wind_up(self):
        k, out = kernel(max_lead=0.1)
        state(k, 0.0)  # the joint never moves
        stick(k, left_x=1.0)
        tick(k, 50)
        self.assertAlmostEqual(targets(out)[-1], 0.1, places=6)

    def test_a_disconnected_gamepad_stops_the_joint(self):
        k, out = kernel()
        stick(k, left_x=1.0)
        tick(k)
        k.process_input("axes", {"left_x": 1.0, "connected": False})
        tick(k, 5)
        self.assertEqual(len(targets(out)), 1)

    def test_ignores_another_joints_state(self):
        k, out = kernel()
        state(k, 0.7, name="tilt")
        stick(k, left_x=1.0)
        tick(k)
        self.assertAlmostEqual(targets(out)[0], 0.02, places=6)


class MissingJointTest(unittest.TestCase):
    def test_sends_nothing_without_a_joint(self):
        logs = []
        k = JointFromGamepadKernel("jog", {}, on_emit=lambda *a: logs.append(("emit", a)), on_log=logs.append)
        stick(k, left_x=1.0)
        tick(k)
        self.assertFalse([x for x in logs if isinstance(x, tuple)])
        self.assertTrue(any("needs its joint" in str(x) for x in logs))


if __name__ == "__main__":
    unittest.main()
