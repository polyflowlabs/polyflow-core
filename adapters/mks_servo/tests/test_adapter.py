"""The adapter end to end against a simulated driver: startup, position and velocity, lifecycle, state."""

import math
import unittest

from polyflow_adapter_sdk import (
    JOINT_MODE_POSITION,
    JOINT_MODE_TORQUE,
    JOINT_MODE_VELOCITY,
    LEVEL_ERROR,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ESTOP,
)

from mks_servo_adapter import protocol as p
from mks_servo_adapter.can_adapter import MksServoCANAdapter

from .fake_servo import FakeBus, FakeHost, FakeMksServo


def make(params=None, servo_id=3, adapter_id=3):
    servo = FakeMksServo(can_id=servo_id)
    bus = FakeBus(servo)
    host = FakeHost()
    adapter = MksServoCANAdapter(host, "shoulder", {"can_id": adapter_id, **(params or {})})
    adapter.transport_factory = lambda _a: bus
    adapter.configure()
    adapter.start()
    return adapter, servo, bus, host


def codes(bus):
    return [data[0] for _id, data in bus.sent]


def poll(adapter, host):
    host.polls[-1]()
    return host.states[-1][2]


class StartupTest(unittest.TestCase):
    def test_puts_the_driver_in_bus_mode_and_enables_it(self):
        adapter, servo, bus, host = make()
        self.assertEqual(codes(bus)[:3], [p.SET_WORK_MODE, p.SET_RESPONSE, p.ENABLE])
        self.assertEqual(servo.mode, p.MODE_SR_VFOC)
        self.assertTrue(servo.enabled)
        self.assertEqual((servo.respond, servo.active), (True, False))
        self.assertNotIn(p.SET_ZERO, codes(bus))
        self.assertTrue(all(can_id == 3 for can_id, _ in bus.sent))

    def test_zero_on_start_sets_the_present_position_as_zero(self):
        servo_counts_before = 5000
        servo = FakeMksServo(can_id=1)
        servo.counts = servo_counts_before
        bus, host = FakeBus(servo), FakeHost()
        adapter = MksServoCANAdapter(host, "j", {"can_id": 1, "zero_on_start": True})
        adapter.transport_factory = lambda _a: bus
        adapter.configure()
        adapter.start()
        self.assertEqual(servo.counts, 0)

    def test_a_silent_driver_is_reported_by_its_can_id(self):
        adapter, servo, bus, host = make(servo_id=4, adapter_id=3)  # set to 4, deployed as 3
        errors = [s for s in host.statuses if s[1] == LEVEL_ERROR]
        self.assertEqual(errors[0][2], "no_reply")
        self.assertIn("CAN id 3", errors[0][3])
        self.assertFalse(poll(adapter, host)["connected"])

    def test_rejects_the_broadcast_id(self):
        adapter = MksServoCANAdapter(FakeHost(), "j", {"can_id": 0})
        with self.assertRaises(ValueError):
            adapter.configure()


class PositionTest(unittest.TestCase):
    def test_a_joint_target_moves_the_motor_through_the_gear_ratio(self):
        adapter, servo, bus, host = make({"gear_ratio": 26.85, "move_speed_rpm": 1200})
        adapter.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.5})

        expected = round(0.5 * 26.85 * 16384 / (2 * math.pi))
        self.assertEqual(servo.target, expected)
        self.assertEqual(bus.sent[-1][1], p.move_absolute(3, expected, 1200, 100))

        for _ in range(400):
            servo.step(0.01)
        state = poll(adapter, host)
        self.assertAlmostEqual(state["position"], 0.5, places=3)
        self.assertTrue(state["connected"])
        self.assertEqual(state["joint_id"], "shoulder")

    def test_the_commands_velocity_sets_the_move_speed(self):
        adapter, servo, bus, host = make({"gear_ratio": 10})
        adapter.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 1.0, "has_velocity": True, "velocity": 2.0})
        speed = int.from_bytes(bus.sent[-1][1][1:3], "big")
        self.assertEqual(speed, round(2.0 * 10 * 60 / (2 * math.pi)))

    def test_invert_flips_command_and_reading(self):
        adapter, servo, bus, host = make({"invert": True})
        adapter.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 1.0})
        self.assertLess(servo.target, 0)
        for _ in range(200):
            servo.step(0.01)
        self.assertAlmostEqual(poll(adapter, host)["position"], 1.0, places=3)

    def test_a_target_beyond_int24_is_clamped_and_reported(self):
        adapter, servo, bus, host = make({"gear_ratio": 100})
        adapter.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 10000.0})
        self.assertEqual(servo.target, p.INT24_MAX)
        self.assertIn("target_out_of_range", [s[2] for s in host.statuses])


class VelocityTest(unittest.TestCase):
    def test_velocity_runs_the_motor_and_reads_back(self):
        adapter, servo, bus, host = make({"gear_ratio": 5})
        adapter.on_command("JointCommand", {"mode": JOINT_MODE_VELOCITY, "has_velocity": True, "velocity": -1.0})
        expected_rpm = round(1.0 * 5 * 60 / (2 * math.pi))
        self.assertEqual(servo.rpm, -expected_rpm)
        state = poll(adapter, host)
        self.assertAlmostEqual(state["velocity"], -expected_rpm * 2 * math.pi / 60 / 5, places=6)

    def test_torque_is_reported_once_as_unsupported(self):
        adapter, servo, bus, host = make()
        for _ in range(3):
            adapter.on_command("JointCommand", {"mode": JOINT_MODE_TORQUE, "has_torque": True, "torque": 1.0})
        self.assertEqual([s[2] for s in host.statuses].count("torque_unsupported"), 1)


class LifecycleTest(unittest.TestCase):
    def test_disable_and_estop(self):
        adapter, servo, bus, host = make()
        adapter.on_command("JointCommand", {"mode": JOINT_MODE_VELOCITY, "has_velocity": True, "velocity": 3.0})
        adapter.on_command("HardwareLifecycle", {"request": LIFECYCLE_ESTOP})
        self.assertEqual(servo.rpm, 0)
        self.assertEqual(codes(bus)[-1], p.EMERGENCY_STOP)
        adapter.on_command("HardwareLifecycle", {"request": LIFECYCLE_DISABLE})
        self.assertFalse(servo.enabled)

    def test_estop_hook(self):
        adapter, servo, bus, host = make()
        adapter.estop()
        self.assertEqual(codes(bus)[-1], p.EMERGENCY_STOP)

    def test_shutdown_stops_but_keeps_holding(self):
        adapter, servo, bus, host = make()
        adapter.on_command("JointCommand", {"mode": JOINT_MODE_VELOCITY, "has_velocity": True, "velocity": 3.0})
        adapter.shutdown()
        self.assertEqual(servo.rpm, 0)
        self.assertTrue(servo.enabled)
        self.assertTrue(bus.closed)

    def test_connection_drops_when_the_driver_goes_quiet(self):
        adapter, servo, bus, host = make({"reply_timeout_s": 0.0})
        servo.powered = False
        self.assertFalse(poll(adapter, host)["connected"])


class BadFramesTest(unittest.TestCase):
    def test_a_corrupt_reply_is_counted_not_trusted(self):
        adapter, servo, bus, host = make()
        servo.outbox.append((3, bytes([p.READ_ENCODER_ADDITION, 0, 0, 0, 0, 0x40, 0x00, 0x00])))  # wrong checksum
        poll(adapter, host)
        self.assertIn("bad_frames", [s[2] for s in host.statuses])


if __name__ == "__main__":
    unittest.main()
