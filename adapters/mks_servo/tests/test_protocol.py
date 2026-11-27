"""Every frame builder against the manual's own worked examples (CAN User Manual V1.0.9)."""

import math
import unittest

from mks_servo_adapter import protocol as p


def hexs(data: bytes) -> str:
    return data.hex(" ").upper()


class ChecksumTest(unittest.TestCase):
    def test_manual_examples(self):
        # 1.11.1: "01 83 0C 80 CRC", CRC = (0x01 + 0x83 + 0x0C + 0x80) & 0xFF = 0x10
        self.assertEqual(p.checksum(0x01, bytes([0x83, 0x0C, 0x80])), 0x10)
        # Part 4: read encoder "01 30 CRC", CRC = 0x31
        self.assertEqual(hexs(p.read(1, p.READ_ENCODER_CARRY)), "30 31")


class CommandFramesTest(unittest.TestCase):
    def test_speed_mode_forward_and_reverse(self):
        # 10.1: "Send 01 F6 01 40 02 3A, the motor rotates forward at acc=2, speed=320 (RPM)"
        self.assertEqual(hexs(p.speed_mode(1, 320, 2)), "F6 01 40 02 3A")
        # "Send 01 F6 81 40 02 BA, the motor reverses at acc=2, speed=320"
        self.assertEqual(hexs(p.speed_mode(1, -320, 2)), "F6 81 40 02 BA")

    def test_speed_mode_stop(self):
        # 10.2: "Send 01 F6 00 00 02 F9" — decelerate to a stop at acc=2
        self.assertEqual(hexs(p.speed_mode(1, 0, 2)), "F6 00 00 02 F9")

    def test_absolute_coordinate_moves(self):
        # 11.4.1: "Send 01 F5 02 58 02 00 40 00 92" — 600 RPM, acc 2, to 0x4000
        self.assertEqual(hexs(p.move_absolute(1, 0x4000, 600, 2)), "F5 02 58 02 00 40 00 92")
        # "Send 01 F5 02 58 02 FF C0 00 11" — to -0x4000
        self.assertEqual(hexs(p.move_absolute(1, -0x4000, 600, 2)), "F5 02 58 02 FF C0 00 11")
        # 11.4.3: "01 F5 01 2C 02 02 80 00 A7" — 300 RPM, acc 2, to 0x28000
        self.assertEqual(hexs(p.move_absolute(1, 0x28000, 300, 2)), "F5 01 2C 02 02 80 00 A7")

    def test_absolute_stop_commands(self):
        # 11.4.2: "Send 01 F5 00 00 04 00 00 00 FA" (decelerate) and "01 F5 00 00 00 00 00 00 F6" (at once)
        self.assertEqual(hexs(p.move_absolute(1, 0, 0, 4)), "F5 00 00 04 00 00 00 FA")
        self.assertEqual(hexs(p.move_absolute(1, 0, 0, 0)), "F5 00 00 00 00 00 00 F6")

    def test_setup_commands(self):
        # 11.4.3 steps: "01 82 05 88" (bus FOC mode), "01 92 93" (set zero), "01 31 32" (read), "01 F7 F8" (e-stop)
        self.assertEqual(hexs(p.set_work_mode(1, p.MODE_SR_VFOC)), "82 05 88")
        self.assertEqual(hexs(p.read(1, p.SET_ZERO)), "92 93")
        self.assertEqual(hexs(p.read(1, p.READ_ENCODER_ADDITION)), "31 32")
        self.assertEqual(hexs(p.emergency_stop(1)), "F7 F8")

    def test_another_can_id_changes_only_the_checksum(self):
        self.assertEqual(hexs(p.read(2, p.READ_ENCODER_ADDITION)), "31 33")
        # The id is added in full and masked: id 0x101 checks like 0x01.
        self.assertEqual(p.read(0x101, p.READ_ENCODER_ADDITION)[-1], p.read(1, p.READ_ENCODER_ADDITION)[-1])

    def test_limits(self):
        self.assertEqual(p.speed_mode(1, 5000, 300)[1:4], bytes([0x0B, 0xB8, 0xFF]))  # 3000 RPM, acc 255
        with self.assertRaises(ValueError):
            p.move_absolute(1, 0x800000, 100, 2)


class RepliesTest(unittest.TestCase):
    def reply(self, can_id, code, payload):
        body = bytes([code]) + payload
        return body + bytes([p.checksum(can_id, body)])

    def test_accumulated_encoder_int48(self):
        # 5.1.2: one turn from 0x3FF0 the other way reads 0xFFFFFFFFFFF0 — i.e. -16 counts
        code, payload = p.parse(1, self.reply(1, 0x31, bytes.fromhex("FFFFFFFFFFF0")))
        self.assertEqual((code, p.decode_addition(payload)), (0x31, -16))
        _, payload = p.parse(1, self.reply(1, 0x31, (0x7FF0).to_bytes(6, "big")))
        self.assertEqual(p.decode_addition(payload), 0x7FF0)

    def test_speed_is_signed(self):
        _, payload = p.parse(1, self.reply(1, 0x32, (-320).to_bytes(2, "big", signed=True)))
        self.assertEqual(p.decode_speed(payload), -320)

    def test_bad_checksum_is_rejected(self):
        frame = bytearray(self.reply(1, 0xF5, b"\x01"))
        frame[-1] ^= 0xFF
        with self.assertRaises(p.ProtocolError):
            p.parse(1, bytes(frame))

    def test_fixed_checksum_mode(self):
        # 3.14: a driver set to the fixed check sends 0x6B in place of the sum.
        self.assertEqual(p.parse(1, bytes([0xF3, 0x01, 0x6B]), fixed_checksum=0x6B), (0xF3, b"\x01"))


class UnitsTest(unittest.TestCase):
    def test_one_turn_is_16384_counts(self):
        self.assertEqual(p.radians_to_counts(2 * math.pi), 0x4000)
        self.assertAlmostEqual(p.counts_to_radians(0x2000), math.pi)
        self.assertAlmostEqual(p.rad_s_to_rpm(2 * math.pi), 60.0)


if __name__ == "__main__":
    unittest.main()
