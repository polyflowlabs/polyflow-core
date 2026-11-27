"""Face's kernel. Run from the repo root:
    PYTHONPATH=common:utilities/face python3 -m unittest discover -s utilities/face/test
"""
import unittest

from face.kernel import FaceKernel


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def kernel(**params):
    emitted = []
    clock = Clock()
    FaceKernel.clock = clock  # read in setup(), so set before construction
    k = FaceKernel("face", {"display_id": "oled-1", **params}, on_emit=lambda pin, data: emitted.append((pin, data)), on_log=lambda _m: None)
    k.clock = clock
    return k, emitted, clock


def commands(emitted):
    return [d for pin, d in emitted if pin == "hw_command"]


def drive(k, forward=0.0, turn=0.0):
    k.process_input("cmd_vel", {"linear": {"x": forward, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": turn}})


class FaceTest(unittest.TestCase):
    def tearDown(self):
        FaceKernel.clock = staticmethod(__import__("time").monotonic)

    def test_commands_are_addressed_to_the_display(self):
        k, out, _ = kernel()
        k.poll()
        self.assertEqual(commands(out)[-1], {"display_id": "oled-1", "expression": "neutral", "gaze_x": 0.0, "gaze_y": 0.0})

    def test_eyes_follow_a_turn_and_look_up_going_forward(self):
        k, out, _ = kernel()
        drive(k, forward=1.0, turn=0.5)
        self.assertEqual(commands(out)[-1]["gaze_x"], 0.5)
        self.assertEqual(commands(out)[-1]["gaze_y"], -0.4)
        drive(k, forward=-1.0, turn=-1.0)
        self.assertEqual((commands(out)[-1]["gaze_x"], commands(out)[-1]["gaze_y"]), (-1.0, 0.4))

    def test_a_button_flashes_its_expression_then_it_returns(self):
        k, out, clock = kernel(flash_s=2)
        k.process_input("buttons", {"a": True})
        self.assertEqual(commands(out)[-1]["expression"], "happy")
        k.process_input("buttons", {"a": False})
        clock.t += 2.5
        k.poll()
        self.assertEqual(commands(out)[-1]["expression"], "neutral")

    def test_a_held_button_is_one_press(self):
        k, out, clock = kernel(flash_s=1)
        k.process_input("buttons", {"b": True})
        clock.t += 1.5
        k.process_input("buttons", {"b": True})
        self.assertEqual(commands(out)[-1]["expression"], "neutral")

    def test_sleepy_when_idle_and_woken_by_driving(self):
        k, out, clock = kernel(idle_after_s=5)
        clock.t += 6
        k.poll()
        self.assertEqual(commands(out)[-1]["expression"], "sleepy")
        drive(k, forward=0.5)
        self.assertEqual(commands(out)[-1]["expression"], "neutral")

    def test_a_standing_expression_overrides_until_cleared(self):
        k, out, clock = kernel()
        k.process_input("set_expression", {"data": "Angry"})
        self.assertEqual(commands(out)[-1]["expression"], "angry")
        k.process_input("set_expression", {"data": "made-up"})
        self.assertEqual(commands(out)[-1]["expression"], "neutral")

    def test_only_changes_are_sent_with_a_slow_keepalive(self):
        k, out, clock = kernel()
        k.poll()
        k.poll()
        clock.t += 0.5
        k.poll()
        self.assertEqual(len(commands(out)), 1)
        clock.t += 0.6
        k.poll()
        self.assertEqual(len(commands(out)), 2)

    def test_no_display_sends_nothing(self):
        k, out, _ = kernel(display_id="")
        drive(k, forward=1.0)
        self.assertEqual(commands(out), [])


if __name__ == "__main__":
    unittest.main()
