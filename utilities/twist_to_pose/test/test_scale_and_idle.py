"""linear_scale / angular_scale and hold_when_idle. Run from the repo root:
    PYTHONPATH=common:utilities/twist_to_pose python3 -m unittest discover -s utilities/twist_to_pose/test
"""
import unittest

from twist_to_pose.kernel import TwistToPoseKernel


def kernel(**params):
    out = []
    k = TwistToPoseKernel("t", {"rate_hz": 10, "frame": "world", "require_seed": True, "require_enable": False, **params},
                          on_emit=lambda pin, data: out.append(data), on_log=lambda _m: None)
    return k, out


def seed(k, x=0.0, y=0.0, z=0.0):
    k.process_input("seed_pose", {"position": {"x": x, "y": y, "z": z}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}})


def twist(k, x=0.0, z=0.0):
    k.process_input("cmd_vel", {"linear": {"x": x, "y": 0.0, "z": z}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}})


class ScaleTest(unittest.TestCase):
    def test_full_stick_is_the_scaled_speed(self):
        k, out = kernel(linear_scale=0.1)
        seed(k)
        twist(k, x=1.0)
        for _ in range(10):  # one second at 10 Hz
            k.poll()
        self.assertAlmostEqual(out[-1]["position"]["x"], 0.1)

    def test_default_scale_passes_through(self):
        k, out = kernel()
        seed(k)
        twist(k, x=1.0)
        for _ in range(10):
            k.poll()
        self.assertAlmostEqual(out[-1]["position"]["x"], 1.0)


class IdleTest(unittest.TestCase):
    def test_holding_is_the_default(self):
        k, out = kernel()
        seed(k)
        k.poll()
        self.assertEqual(len(out), 1)

    def test_silent_at_rest_and_resumes_from_the_hands_real_pose(self):
        k, out = kernel(hold_when_idle=False, linear_scale=0.1)
        seed(k)
        for _ in range(5):
            k.poll()
        self.assertEqual(out, [])
        twist(k, x=1.0)
        k.poll()
        self.assertAlmostEqual(out[-1]["position"]["x"], 0.01)
        twist(k)  # released
        k.poll()
        n = len(out)
        # Meanwhile the arm was moved by something else: IK reports the hand at x = 0.3.
        seed(k, x=0.3)
        k.poll()
        self.assertEqual(len(out), n)
        twist(k, x=1.0)
        k.poll()
        self.assertAlmostEqual(out[-1]["position"]["x"], 0.31)


if __name__ == "__main__":
    unittest.main()
