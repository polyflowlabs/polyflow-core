"""
The MKS SERVO42D / 57D / 28D CAN protocol, as bytes. Pure: no bus, no clock, no hardware.

Source: "MKS SERVO42D/57D Stepper Motor CAN User Manual V1.0.9" (Makerbase, firmware V1.0.9),
github.com/makerbase-motor/MKS-SERVO42D-57D, "User Manual/V1.0.9". Section numbers below are that
manual's. The SERVO28D uses the same CAN protocol (its own manual is not published separately).

Frame (Part 4): a STANDARD (11-bit) CAN frame whose id is the driver's CAN id (1..2047, default 1;
0 is broadcast, which no driver answers). Data, at most 8 bytes:

    byte 1        the command code
    bytes 2..n-1  the command's data, MULTI-BYTE FIELDS BIG-ENDIAN (the manual's worked examples:
                  `01 F6 01 40 02 3A` is speed 0x140 = 320 RPM, high byte first)
    byte n        checksum = (CAN id + every preceding data byte) & 0xFF   (1.11.1, Part 4)

A reply comes back on the same CAN id with the same code in byte 1 (Part 4, "uplink").

Units: the encoder has 16384 counts per motor revolution (0x4000, 1.11.2); "coordinates" for the
F5 absolute move are the accumulated multi-turn encoder value read by 0x31 (11.4, note 1). Speeds
are RPM, 0..3000 (9.1), calibrated at 16/32/64 microsteps; acceleration is 0..255 (9.1).

Direction. The manual is inconsistent: 5.1.2's prose says one turn clockwise adds 0x4000, while its
own example (and 5.1.1's) adds 0x4000 for a turn counter-clockwise; 5.1.3 says a CCW run reads as a
positive speed, and 10.1 calls direction bit 0 (CCW) "forward". This module takes the worked
examples' convention — CCW is positive, direction bit 0 — and the adapter's `invert` param flips a
joint whose real motor turns the other way (checked on first power-up).
"""

import math
from typing import Optional, Tuple

COUNTS_PER_REV = 0x4000
MAX_SPEED_RPM = 3000
MAX_ACC = 255
INT24_MAX = 0x7FFFFF
INT24_MIN = -0x7FFFFF  # the manual's stated range is (-8388607, +8388607)

# Command codes (section in the manual).
READ_ENCODER_CARRY = 0x30      # 5.1.1  carry int32 + value uint16
READ_ENCODER_ADDITION = 0x31   # 5.1.2  accumulated value int48 — the F5 coordinate
READ_SPEED = 0x32              # 5.1.3  rpm int16
READ_ENABLE = 0x3A             # 5.1.7  uint8
RELEASE_STALL = 0x3D           # 5.4.4
READ_STALL = 0x3E              # 5.4.3  uint8
SET_WORK_MODE = 0x82           # 5.2.2
SET_RESPONSE = 0x8C            # 5.2.12
SET_ZERO = 0x92                # 7.3.5  current position becomes coordinate 0
QUERY_STATUS = 0xF1            # 9.2.1
ENABLE = 0xF3                  # 9.2.2
SPEED_MODE = 0xF6              # 10.1
EMERGENCY_STOP = 0xF7          # 9.2.3
MOVE_ABSOLUTE = 0xF5           # 11.4.1 absolute coordinate

# 5.2.2 work modes. Only the SR_ (serial/bus) modes accept bus motion commands (Part 9).
MODE_CR_OPEN, MODE_CR_CLOSE, MODE_CR_VFOC = 0x00, 0x01, 0x02
MODE_SR_OPEN, MODE_SR_CLOSE, MODE_SR_VFOC = 0x03, 0x04, 0x05

# 9.2.1 run status.
STATUS_QUERY_FAILED, STATUS_STOPPED, STATUS_ACCELERATING, STATUS_DECELERATING, STATUS_FULL_SPEED, STATUS_HOMING, STATUS_CALIBRATING = range(7)

# F5/F6 reply status (10.1, 11.4.1).
RUN_FAILED, RUN_STARTING, RUN_COMPLETE, RUN_END_LIMIT = 0, 1, 2, 3


class ProtocolError(ValueError):
    """A reply that isn't a valid frame: too short, or a checksum that doesn't add up."""


def checksum(can_id: int, data: bytes) -> int:
    """(CAN id + every data byte) & 0xFF — the manual's CHECKSUM 8bit (1.11.1)."""
    return (int(can_id) + sum(data)) & 0xFF


def frame(can_id: int, code: int, payload: bytes = b"") -> bytes:
    """The data field of a command: code, payload, checksum."""
    body = bytes([code & 0xFF]) + bytes(payload)
    if len(body) + 1 > 8:
        raise ValueError(f"MKS frame too long: {len(body) + 1} bytes")
    return body + bytes([checksum(can_id, body)])


def parse(can_id: int, data: bytes, fixed_checksum: Optional[int] = None) -> Tuple[int, bytes]:
    """A reply's (code, payload), with the checksum verified. `fixed_checksum` is for a driver set
    to the 0x6B fixed-check mode (3.14); otherwise the SUM-8 checksum is checked."""
    data = bytes(data)
    if len(data) < 2:
        raise ProtocolError(f"MKS reply too short: {data.hex()}")
    body, crc = data[:-1], data[-1]
    expected = fixed_checksum if fixed_checksum is not None else checksum(can_id, body)
    if crc != expected:
        raise ProtocolError(f"MKS reply checksum {crc:#04x} != {expected:#04x}: {data.hex()}")
    return body[0], body[1:]


def _int_be(raw: bytes, signed: bool) -> int:
    return int.from_bytes(raw, "big", signed=signed)


def _int24_be(value: int) -> bytes:
    return (int(value) & 0xFFFFFF).to_bytes(3, "big")


def clamp_speed(rpm: float) -> int:
    return max(0, min(MAX_SPEED_RPM, int(round(abs(rpm)))))


def clamp_acc(acc: float) -> int:
    return max(0, min(MAX_ACC, int(round(acc))))


# ── commands ────────────────────────────────────────────────────────────────

def read(can_id: int, code: int) -> bytes:
    """A read-only query with no data: 0x30, 0x31, 0x32, 0x3A, 0x3E, 0xF1, 0x3D, 0x92 …"""
    return frame(can_id, code)


def set_work_mode(can_id: int, mode: int) -> bytes:
    return frame(can_id, SET_WORK_MODE, bytes([mode & 0xFF]))


def set_response(can_id: int, respond: bool, active: bool) -> bytes:
    """5.2.12: `respond` — answer commands at all; `active` — also report when a move completes."""
    return frame(can_id, SET_RESPONSE, bytes([1 if respond else 0, 1 if active else 0]))


def enable(can_id: int, on: bool) -> bytes:
    return frame(can_id, ENABLE, bytes([1 if on else 0]))


def emergency_stop(can_id: int, priority: bool = False) -> bytes:
    """9.2.3. `priority` stops immediately regardless of current (`F7 01`)."""
    return frame(can_id, EMERGENCY_STOP, bytes([1]) if priority else b"")


def speed_mode(can_id: int, rpm: float, acc: int) -> bytes:
    """10.1: run continuously at `rpm` (sign = direction: positive is CCW, direction bit 0).
    rpm 0 is the stop command (10.2): with acc > 0 it decelerates, with acc 0 it stops at once."""
    speed = clamp_speed(rpm)
    direction = 1 if rpm < 0 else 0
    return frame(can_id, SPEED_MODE, bytes([(direction << 7) | ((speed >> 8) & 0x0F), speed & 0xFF, clamp_acc(acc)]))


def move_absolute(can_id: int, coordinate: int, rpm: float, acc: int) -> bytes:
    """11.4.1: run to an absolute coordinate (accumulated encoder counts, int24) at `rpm` (0..3000),
    `acc` (0..255). Speed 0 with coordinate 0 is its stop command (11.4.2)."""
    if not INT24_MIN <= int(coordinate) <= INT24_MAX:
        raise ValueError(f"MKS coordinate {coordinate} outside int24")
    speed = clamp_speed(rpm)
    return frame(can_id, MOVE_ABSOLUTE, speed.to_bytes(2, "big") + bytes([clamp_acc(acc)]) + _int24_be(coordinate))


# ── replies ─────────────────────────────────────────────────────────────────

def decode_addition(payload: bytes) -> int:
    """0x31 reply: the accumulated multi-turn encoder value, int48 big-endian (5.1.2)."""
    if len(payload) < 6:
        raise ProtocolError(f"0x31 reply payload too short: {payload.hex()}")
    return _int_be(payload[:6], signed=True)


def decode_speed(payload: bytes) -> int:
    """0x32 reply: rpm, int16 big-endian; positive is CCW (5.1.3)."""
    if len(payload) < 2:
        raise ProtocolError(f"0x32 reply payload too short: {payload.hex()}")
    return _int_be(payload[:2], signed=True)


def decode_status(payload: bytes) -> int:
    """A one-byte status reply (F1, F3, F5, F6, F7, 82, 8C, 3A, 3E, 3D, 92)."""
    if len(payload) < 1:
        raise ProtocolError("status reply has no payload")
    return payload[0]


# ── units ───────────────────────────────────────────────────────────────────

def radians_to_counts(motor_rad: float) -> int:
    return int(round(motor_rad * COUNTS_PER_REV / (2.0 * math.pi)))


def counts_to_radians(counts: int) -> float:
    return counts * (2.0 * math.pi) / COUNTS_PER_REV


def rad_s_to_rpm(motor_rad_s: float) -> float:
    return motor_rad_s * 60.0 / (2.0 * math.pi)


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * (2.0 * math.pi) / 60.0
