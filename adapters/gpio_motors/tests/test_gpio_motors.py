"""The three GPIO drivers against a fake GPIO backend: the TB6612 truth table, step counts and a
trapezoidal profile for the StepStick, servo pulse widths, and pins shared between drivers."""

import math
import unittest

from polyflow_adapter_sdk import (
    JOINT_MODE_POSITION,
    JOINT_MODE_TORQUE,
    JOINT_MODE_VELOCITY,
    LEVEL_ERROR,
    LIFECYCLE_ENABLE,
    LIFECYCLE_ESTOP,
    MOTOR_MODE_DUTY,
    MOTOR_MODE_IDLE,
    MOTOR_MODE_SPEED,
)

from gpio_motor_adapter.gpio import PinInUse, pin_param
from gpio_motor_adapter.servo_adapter import GpioServoAdapter
from gpio_motor_adapter.stepstick_adapter import StepPlanner, StepStickAdapter
from gpio_motor_adapter.tb6612_adapter import TB6612MotorAdapter


class FakeGpio:
    """Pin levels, PWM duty, servo pulse and queued pulse trains, as the drivers left them."""

    def __init__(self):
        self.owners, self.level, self.duty, self.servo_us = {}, {}, {}, {}
        self.trains = []  # (pin, frequency, count)
        self.stopped = []
        self.playing = set()  # pins whose last train hasn't finished (tests set it)

    def claim_output(self, pin, owner, level=0):
        if pin in self.owners and self.owners[pin] != owner:
            raise PinInUse(f"GPIO{pin} is already used by '{self.owners[pin]}'")
        self.owners[pin] = owner
        self.level[pin] = level

    def write(self, pin, level):
        assert pin in self.owners, f"GPIO{pin} written without being claimed"
        self.level[pin] = level

    def pwm(self, pin, frequency_hz, duty_percent):
        self.duty[pin] = duty_percent

    def servo(self, pin, pulse_us, frequency_hz=50.0):
        self.servo_us[pin] = pulse_us

    def pulses(self, pin, frequency_hz, count):
        self.trains.append((pin, frequency_hz, count))

    def busy(self, pin):
        return pin in self.playing

    def stop(self, pin):
        self.stopped.append(pin)
        self.duty[pin] = 0
        self.servo_us[pin] = 0

    def release(self, pin):
        self.owners.pop(pin, None)

    def close(self):
        pass

    def steps(self, pin):
        return sum(c for p, _, c in self.trains if p == pin)


class FakeHost:
    def __init__(self):
        self.states, self.statuses, self.polls = [], [], []

    def publish_state(self, target_id, state_type, data):
        self.states.append((target_id, state_type, data))

    def report_status(self, target_id, level, code, message, values):
        self.statuses.append((level, code, message))

    def schedule(self, rate_hz, fn):
        self.polls.append(fn)

        class H:
            def cancel(self):
                pass

        return H()

    def log(self, message):
        pass


def make(cls, params, fake=None, target="j1"):
    fake = fake or FakeGpio()
    host = FakeHost()
    a = cls(host, target, params)
    a.backend_factory = lambda: fake
    a.configure()
    a.start()
    return a, fake, host


TB = {"pwm_pin": 12, "in1_pin": 5, "in2_pin": 6, "max_speed_rad_s": 10}


class PinParamTest(unittest.TestCase):
    def test_forms(self):
        self.assertEqual(pin_param({"p": 12}, "p"), 12)
        self.assertEqual(pin_param({"p": "GPIO17"}, "p"), 17)
        self.assertIsNone(pin_param({}, "p", required=False))
        with self.assertRaises(ValueError):
            pin_param({}, "p")
        with self.assertRaises(ValueError):
            pin_param({"p": 40}, "p")  # a header position, not a BCM number


class TB6612Test(unittest.TestCase):
    def test_forward_reverse_and_duty(self):
        m, g, _ = make(TB6612MotorAdapter, TB)
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.5})
        self.assertEqual((g.level[5], g.level[6], g.duty[12]), (1, 0, 50.0))
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": -0.25})
        self.assertEqual((g.level[5], g.level[6], g.duty[12]), (0, 1, 25.0))

    def test_speed_is_scaled_by_full_scale_speed(self):
        m, g, _ = make(TB6612MotorAdapter, TB)
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_SPEED, "value": 5.0})  # half of 10 rad/s
        self.assertEqual(g.duty[12], 50.0)
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_SPEED, "value": 30.0})  # clamped
        self.assertEqual(g.duty[12], 100.0)

    def test_zero_and_idle_short_brake(self):
        m, g, _ = make(TB6612MotorAdapter, TB)
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.8})
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_IDLE})
        self.assertEqual((g.level[5], g.level[6], g.duty[12]), (1, 1, 0))  # H/H = short brake

    def test_coast_when_asked(self):
        m, g, _ = make(TB6612MotorAdapter, {**TB, "brake_on_stop": False})
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.0})
        self.assertEqual((g.level[5], g.level[6]), (0, 0))

    def test_invert_and_min_duty(self):
        m, g, _ = make(TB6612MotorAdapter, {**TB, "invert": True, "min_duty": 0.2})
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.5})
        self.assertEqual((g.level[5], g.level[6]), (0, 1))
        self.assertAlmostEqual(g.duty[12], 60.0)

    def test_estop_latches_until_enable_and_drops_standby(self):
        m, g, _ = make(TB6612MotorAdapter, {**TB, "stby_pin": 16})
        self.assertEqual(g.level[16], 1)
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.5})
        m.estop()
        self.assertEqual((g.duty[12], g.level[16]), (0, 0))
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.5})
        self.assertEqual(g.duty[12], 0)
        m.on_command("HardwareLifecycle", {"request": LIFECYCLE_ENABLE})
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.5})
        self.assertEqual((g.duty[12], g.level[16]), (50.0, 1))

    def test_watchdog_stops_a_lapsed_command(self):
        m, g, host = make(TB6612MotorAdapter, TB)
        m.on_command("MotorCommand", {"mode": MOTOR_MODE_DUTY, "value": 0.5, "timeout_s": 0.01})
        m._last_cmd -= 1.0
        host.polls[-1]()
        self.assertEqual(g.duty[12], 0)
        self.assertEqual(host.statuses[-1][1], "watchdog_timeout")
        self.assertEqual(host.states[-1][2]["mode"], MOTOR_MODE_IDLE)

    def test_two_channels_on_one_pin_is_refused(self):
        g = FakeGpio()
        make(TB6612MotorAdapter, TB, g, "left")
        right, _, host = make(TB6612MotorAdapter, {"pwm_pin": 13, "in1_pin": 5, "in2_pin": 20}, g, "right")
        self.assertEqual(host.statuses[-1][:2], (LEVEL_ERROR, "pin_unavailable"))
        self.assertIn("left", host.statuses[-1][2])
        self.assertIsNone(right._gpio)

    def test_a_pin_twice_in_one_channel_is_a_config_error(self):
        with self.assertRaises(ValueError):
            TB6612MotorAdapter(FakeHost(), "x", {"pwm_pin": 12, "in1_pin": 12, "in2_pin": 6}).configure()


class PlannerTest(unittest.TestCase):
    def run_until_idle(self, p, dt=0.01, limit=10000):
        trace = []
        for _ in range(limit):
            trace.append(p.tick(dt))
            if p.idle:
                break
        return trace

    def test_reaches_the_target_exactly_without_overshoot(self):
        p = StepPlanner(max_speed=2000, accel=4000)
        p.move_to(3000)
        trace = self.run_until_idle(p)
        self.assertEqual(p.position, 3000)
        self.assertTrue(all(s >= 0 for s in trace))
        # Trapezoid: 0.5 s to reach 2000 steps/s (500 steps each way) and ~1 s cruising.
        self.assertAlmostEqual(len(trace) * 0.01, 2.0, delta=0.1)
        self.assertLessEqual(max(trace), 2000 * 0.01 + 1)

    def test_short_moves_and_reversal(self):
        p = StepPlanner(max_speed=2000, accel=4000)
        for target in (7, -5, 0, 1):
            p.move_to(target)
            self.run_until_idle(p)
            self.assertEqual(p.position, target)

    def test_retarget_mid_move_turns_round_smoothly(self):
        p = StepPlanner(max_speed=1000, accel=2000)
        p.move_to(10000)
        for _ in range(50):
            p.tick(0.01)
        p.move_to(0)
        trace = self.run_until_idle(p)
        self.assertEqual(p.position, 0)
        # It decelerates first: no tick changes speed by more than the accel allows, give or take the whole
        # steps a tick is quantized to (one step per 10 ms tick = 100 steps/s, either side of zero).
        speeds = [s / 0.01 for s in trace]
        self.assertTrue(all(abs(b - a) <= 2000 * 0.01 + 2 * 100 for a, b in zip(speeds, speeds[1:])))

    def test_velocity_mode_ramps_then_holds(self):
        p = StepPlanner(max_speed=1000, accel=1000)
        p.run_at(-500)
        for _ in range(100):
            p.tick(0.01)
        self.assertAlmostEqual(p.velocity, -500)
        p.halt()
        self.assertEqual(p.tick(0.01), 0)


ST = {"step_pin": 17, "dir_pin": 27, "steps_per_rev": 200, "microsteps": 16, "gear_ratio": 5, "max_speed_rad_s": 2, "accel_rad_s2": 8}


class StepStickTest(unittest.TestCase):
    def drive(self, s, seconds):
        for _ in range(int(seconds * 100)):
            s.step(0.01)

    def test_a_joint_angle_becomes_counted_steps(self):
        s, g, host = make(StepStickAdapter, ST)
        steps_per_rad = 200 * 16 * 5 / (2 * math.pi)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 1.0})
        self.drive(s, 3)
        self.assertEqual(g.steps(17), round(steps_per_rad))
        self.assertEqual(g.level[27], 1)
        self.assertAlmostEqual(s.position_rad, 1.0, places=3)
        host.polls[-1]()
        self.assertAlmostEqual(host.states[-1][2]["position"], 1.0, places=3)
        self.assertFalse(host.states[-1][2]["closed_loop"])

    def test_going_back_flips_dir(self):
        s, g, _ = make(StepStickAdapter, ST)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.5})
        self.drive(s, 3)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": -0.5})
        self.drive(s, 3)
        self.assertEqual(g.level[27], 0)
        self.assertAlmostEqual(s.position_rad, -0.5, places=3)

    def test_a_reversal_waits_for_the_last_train_to_finish(self):
        s, g, _ = make(StepStickAdapter, ST)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.2})
        self.drive(s, 2)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.0})
        g.playing.add(17)  # the forward train is still going out
        sent = len(g.trains)
        for _ in range(3):
            self.assertEqual(s.step(0.01), 0)
        self.assertEqual((len(g.trains), g.level[27]), (sent, 1))  # nothing sent, DIR untouched
        g.playing.clear()
        self.drive(s, 2)
        self.assertEqual(g.level[27], 0)
        self.assertEqual(s.planner.position, 0)
        self.assertEqual(sum(c for p, _, c in g.trains if p == 17), 2 * round(0.2 * 200 * 16 * 5 / (2 * math.pi)))

    def test_invert_flips_dir_not_the_count(self):
        s, g, _ = make(StepStickAdapter, {**ST, "invert": True})
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.5})
        self.drive(s, 3)
        self.assertEqual(g.level[27], 0)
        self.assertAlmostEqual(s.position_rad, 0.5, places=3)

    def test_pulse_trains_are_evenly_timed_per_tick(self):
        s, g, _ = make(StepStickAdapter, ST)
        s.on_command("JointCommand", {"mode": JOINT_MODE_VELOCITY, "has_velocity": True, "velocity": 1.0})
        self.drive(s, 1)
        pin, freq, count = g.trains[-1]
        self.assertEqual(pin, 17)
        self.assertAlmostEqual(freq, count / 0.01)

    def test_step_rate_is_capped_and_said_so(self):
        s, _, host = make(StepStickAdapter, {**ST, "max_speed_rad_s": 50})
        self.assertEqual(host.statuses[-1][1], "rate_limited")
        self.assertLessEqual(s.planner.max_speed, 20000)

    def test_estop_holds_and_latches(self):
        s, g, _ = make(StepStickAdapter, ST)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 2.0})
        self.drive(s, 0.3)
        s.on_command("HardwareLifecycle", {"request": LIFECYCLE_ESTOP})
        held = s.planner.position
        self.assertIn(17, g.stopped)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 2.0})
        self.drive(s, 1)
        self.assertEqual(s.planner.position, held)
        s.on_command("HardwareLifecycle", {"request": LIFECYCLE_ENABLE})
        self.drive(s, 1)
        self.assertEqual(s.planner.position, held)  # enable alone doesn't resume the old move

    def test_torque_is_refused(self):
        s, _, host = make(StepStickAdapter, ST)
        s.on_command("JointCommand", {"mode": JOINT_MODE_TORQUE, "has_torque": True, "torque": 1.0})
        self.assertEqual(host.statuses[-1][1], "mode_unsupported")

    def test_enable_pin_is_active_low(self):
        s, g, _ = make(StepStickAdapter, {**ST, "enable_pin": 22})
        self.assertEqual(g.level[22], 0)
        s.on_command("HardwareLifecycle", {"request": 4})  # DISABLE
        self.assertEqual(g.level[22], 1)


class GpioServoTest(unittest.TestCase):
    def test_position_to_pulse(self):
        s, g, _ = make(GpioServoAdapter, {"signal_pin": 18})
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.0})
        self.assertAlmostEqual(g.servo_us[18], 1500)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": -math.pi / 2})
        self.assertAlmostEqual(g.servo_us[18], 500)

    def test_estop_goes_limp_and_enable_returns(self):
        s, g, _ = make(GpioServoAdapter, {"signal_pin": 18})
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.4})
        s.estop()
        self.assertEqual(g.servo_us[18], 0)
        s.on_command("HardwareLifecycle", {"request": LIFECYCLE_ENABLE})
        self.assertAlmostEqual(g.servo_us[18], 1500 + 0.4 / math.pi * 2000)

    def test_a_pin_another_driver_has_is_refused(self):
        g = FakeGpio()
        make(TB6612MotorAdapter, TB, g, "wheel")
        _, _, host = make(GpioServoAdapter, {"signal_pin": 12}, g, "pan")
        self.assertEqual(host.statuses[-1][:2], (LEVEL_ERROR, "pin_unavailable"))


if __name__ == "__main__":
    unittest.main()
