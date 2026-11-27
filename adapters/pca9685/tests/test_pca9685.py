"""The chip against the PCA9685 datasheet, and both adapters against a fake I²C bus."""

import math
import unittest

from polyflow_adapter_sdk import JOINT_MODE_POSITION, JOINT_MODE_VELOCITY, LIFECYCLE_DISABLE, LIFECYCLE_ENABLE, LEVEL_ERROR

from pca9685_adapter import chip as c
from pca9685_adapter.board_adapter import PCA9685BoardAdapter
from pca9685_adapter.servo_adapter import PCA9685ServoAdapter


class FakeI2C:
    """Registers of one PCA9685, written through the smbus2 calls the chip uses."""

    def __init__(self, address=0x40):
        self.address = address
        self.regs = {}
        self.writes = []
        self.closed = False

    def write_byte_data(self, address, register, value):
        if address != self.address:
            raise OSError(121, "Remote I/O error")  # what Linux reports for an absent device
        self.regs[register] = value
        self.writes.append((register, value))

    def read_byte_data(self, address, register):
        return self.regs.get(register, 0)

    def write_i2c_block_data(self, address, register, data):
        if address != self.address:
            raise OSError(121, "Remote I/O error")
        for i, v in enumerate(data):
            self.regs[register + i] = v
        self.writes.append((register, list(data)))

    def channel(self, n):
        base = c.LED0_ON_L + 4 * n
        on = self.regs.get(base, 0) | (self.regs.get(base + 1, 0) << 8)
        off = self.regs.get(base + 2, 0) | (self.regs.get(base + 3, 0) << 8)
        return on, off

    def close(self):
        self.closed = True


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


class ChipTest(unittest.TestCase):
    def test_prescale_matches_the_datasheet(self):
        # Datasheet 7.3.5: 200 Hz → PRE_SCALE 0x1E; and the servo standard, 50 Hz → 121.
        self.assertEqual(c.prescale_for(200), 0x1E)
        self.assertEqual(c.prescale_for(50), 121)
        self.assertEqual(c.prescale_for(10), 255)  # clamped to the chip's range

    def test_init_sets_the_frequency_while_asleep(self):
        bus = FakeI2C()
        chip = c.PCA9685(bus)
        chip.init(50)
        sleep_at = bus.writes.index((c.MODE1, c.MODE1_SLEEP | c.MODE1_ALLCALL))
        prescale_at = bus.writes.index((c.PRE_SCALE, 121))
        self.assertLess(sleep_at, prescale_at)
        self.assertEqual(bus.regs[c.MODE1], c.MODE1_RESTART | c.MODE1_AI | c.MODE1_ALLCALL)
        self.assertEqual(bus.regs[c.ALL_LED_OFF_H], c.FULL_OFF)
        self.assertAlmostEqual(chip.frequency_hz, 25e6 / (4096 * 122))

    def test_pulse_width_in_counts(self):
        bus = FakeI2C()
        chip = c.PCA9685(bus)
        chip.init(50)
        off = chip.set_pulse_us(3, 1500)
        self.assertEqual(off, round(1500e-6 * chip.frequency_hz * 4096))
        self.assertEqual(bus.channel(3), (0, off))

    def test_off_is_full_off(self):
        bus = FakeI2C()
        chip = c.PCA9685(bus)
        chip.init(50)
        chip.off(5)
        self.assertEqual(bus.regs[c.LED0_ON_L + 4 * 5 + 3], c.FULL_OFF)


def board(bus=None, params=None):
    bus = bus or FakeI2C()
    host = FakeHost()
    b = PCA9685BoardAdapter(host, "hat", {"i2c_bus": 1, **(params or {})})
    b.bus_factory = lambda n: bus
    b.configure()
    b.start()
    return b, bus, host


def servo(chip, params=None):
    host = FakeHost()
    s = PCA9685ServoAdapter(host, "pan", {"channel": 2, **(params or {})}, resources={"pca9685_board": chip})
    s.configure()
    s.start()
    return s, host


def pulse_of(bus, channel, chip):
    return bus.channel(channel)[1] / 4096 / chip.frequency_hz * 1e6


class AdapterTest(unittest.TestCase):
    def test_board_shares_an_initialised_chip(self):
        b, bus, host = board()
        self.assertIsNotNone(b.get_resource())
        self.assertEqual(bus.regs[c.PRE_SCALE], 121)

    def test_board_reports_a_missing_chip(self):
        b, bus, host = board(params={"address": "0x41"})  # nothing there
        self.assertIsNone(b.get_resource())
        self.assertEqual(host.statuses[-1][1], "chip_not_found")
        self.assertIn("0x41", host.statuses[-1][2])

    def test_a_joint_position_becomes_a_pulse(self):
        b, bus, _ = board()
        chip = b.get_resource()
        s, host = servo(chip)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.0})
        self.assertAlmostEqual(pulse_of(bus, 2, chip), 1500, delta=3)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": math.pi / 2})
        self.assertAlmostEqual(pulse_of(bus, 2, chip), 2500, delta=3)
        host.polls[-1]()
        self.assertAlmostEqual(host.states[-1][2]["position"], math.pi / 2)
        self.assertTrue(host.states[-1][2]["connected"])

    def test_beyond_travel_is_clamped_and_reported_truthfully(self):
        b, bus, _ = board()
        s, host = servo(b.get_resource())
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 3.0})
        host.polls[-1]()
        self.assertAlmostEqual(host.states[-1][2]["position"], math.pi / 2)

    def test_invert_zero_and_gear_ratio(self):
        b, bus, _ = board()
        chip = b.get_resource()
        s, _ = servo(chip, {"invert": True, "zero_rad": 0.5, "gear_ratio": 2})
        self.assertAlmostEqual(s.servo_angle(0.1), 0.5 - 0.2)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.0})
        self.assertAlmostEqual(pulse_of(bus, 2, chip), 1500 + 0.5 / math.pi * 2000, delta=3)

    def test_velocity_is_refused(self):
        b, bus, _ = board()
        s, host = servo(b.get_resource())
        s.on_command("JointCommand", {"mode": JOINT_MODE_VELOCITY, "has_velocity": True, "velocity": 1.0})
        self.assertEqual(host.statuses[-1][1], "mode_unsupported")

    def test_disable_goes_limp_and_enable_returns(self):
        b, bus, _ = board()
        chip = b.get_resource()
        s, _ = servo(chip)
        s.on_command("JointCommand", {"mode": JOINT_MODE_POSITION, "has_position": True, "position": 0.3})
        s.on_command("HardwareLifecycle", {"request": LIFECYCLE_DISABLE})
        self.assertEqual(bus.regs[c.LED0_ON_L + 4 * 2 + 3], c.FULL_OFF)
        s.on_command("HardwareLifecycle", {"request": LIFECYCLE_ENABLE})
        self.assertAlmostEqual(pulse_of(bus, 2, chip), 1500 + 0.3 / math.pi * 2000, delta=3)

    def test_servo_command_and_torque_off(self):
        b, bus, _ = board()
        chip = b.get_resource()
        s, _ = servo(chip)
        s.on_command("ServoCommand", {"position": -math.pi / 2})
        self.assertAlmostEqual(pulse_of(bus, 2, chip), 500, delta=3)
        s.on_command("ServoCommand", {"position": 0.0, "torque_enable": False})
        self.assertEqual(bus.regs[c.LED0_ON_L + 4 * 2 + 3], c.FULL_OFF)

    def test_estop_on_the_board_stops_every_channel(self):
        b, bus, _ = board()
        b.estop()
        self.assertEqual(bus.regs[c.ALL_LED_OFF_H], c.FULL_OFF)

    def test_a_servo_without_its_board_says_so(self):
        s = PCA9685ServoAdapter(FakeHost(), "pan", {"channel": 0}, resources={})
        s.configure()
        s.start()
        self.assertEqual(s._host.statuses[-1][:2], (LEVEL_ERROR, "board_missing"))

    def test_rejects_a_channel_off_the_board(self):
        s = PCA9685ServoAdapter(FakeHost(), "pan", {"channel": 16})
        with self.assertRaises(ValueError):
            s.configure()


if __name__ == "__main__":
    unittest.main()
