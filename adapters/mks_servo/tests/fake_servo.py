"""
A simulated MKS SERVO driver that speaks the CAN protocol from the manual (V1.0.9), for the tests.

It checks every frame's checksum, ignores frames for other CAN ids (and answers nothing when it is
switched off — a dead or misconfigured driver), keeps a position in encoder counts, moves toward an F5
target or runs at an F6 speed as simulated time advances, and answers reads with the manual's reply
layouts. Bus motion is refused outside an SR_ (bus) work mode, as on the real driver (Part 9).
"""

from collections import deque
from typing import Deque, List, Optional, Tuple

from mks_servo_adapter import protocol as p


class FakeMksServo:
    def __init__(self, can_id: int = 1) -> None:
        self.can_id = can_id
        self.powered = True
        self.mode = p.MODE_CR_VFOC  # factory default (5.2.2)
        self.respond = True
        self.active = True
        self.enabled = False
        self.counts = 0.0
        self.rpm = 0.0
        self.target: Optional[int] = None
        self.received: List[bytes] = []
        """Every valid frame addressed to this driver, in order."""
        self.bad_frames = 0
        self.outbox: Deque[Tuple[int, bytes]] = deque()

    # ── the bus side ──

    def receive(self, can_id: int, data: bytes) -> None:
        if not self.powered or can_id != self.can_id:
            return
        data = bytes(data)
        if p.checksum(can_id, data[:-1]) != data[-1]:
            self.bad_frames += 1
            return
        self.received.append(data)
        code, payload = data[0], data[1:-1]
        handler = {
            p.READ_ENCODER_ADDITION: lambda: self._reply(code, int(round(self.counts)).to_bytes(6, "big", signed=True)),
            p.READ_SPEED: lambda: self._reply(code, int(round(self.rpm)).to_bytes(2, "big", signed=True)),
            p.READ_ENABLE: lambda: self._reply(code, bytes([1 if self.enabled else 0])),
            p.SET_WORK_MODE: lambda: self._set_mode(payload),
            p.SET_RESPONSE: lambda: self._set_response(payload),
            p.ENABLE: lambda: self._enable(payload),
            p.SET_ZERO: lambda: self._zero(),
            p.EMERGENCY_STOP: lambda: self._estop(),
            p.SPEED_MODE: lambda: self._speed(payload),
            p.MOVE_ABSOLUTE: lambda: self._move(payload),
        }.get(code)
        if handler:
            handler()

    def _reply(self, code: int, payload: bytes, unsolicited: bool = False) -> None:
        if not self.respond or (unsolicited and not self.active):
            return
        body = bytes([code]) + payload
        self.outbox.append((self.can_id, body + bytes([p.checksum(self.can_id, body)])))

    # ── commands ──

    def _set_mode(self, payload: bytes) -> None:
        self.mode = payload[0]
        self._reply(p.SET_WORK_MODE, b"\x01")

    def _set_response(self, payload: bytes) -> None:
        self.respond, self.active = bool(payload[0]), bool(payload[1])
        self.respond = True  # it answers this one either way, then applies it
        self._reply(p.SET_RESPONSE, b"\x01")
        self.respond = bool(payload[0])

    def _enable(self, payload: bytes) -> None:
        self.enabled = bool(payload[0])
        self._reply(p.ENABLE, b"\x01")

    def _zero(self) -> None:
        self.counts = 0.0
        self.target = None
        self._reply(p.SET_ZERO, b"\x01")

    def _estop(self) -> None:
        self.rpm = 0.0
        self.target = None
        self._reply(p.EMERGENCY_STOP, b"\x01")

    def _bus_mode(self) -> bool:
        return self.mode in (p.MODE_SR_OPEN, p.MODE_SR_CLOSE, p.MODE_SR_VFOC)

    def _speed(self, payload: bytes) -> None:
        if not (self._bus_mode() and self.enabled):
            return self._reply(p.SPEED_MODE, bytes([p.RUN_FAILED]))
        direction = payload[0] >> 7
        speed = ((payload[0] & 0x0F) << 8) | payload[1]
        self.target = None
        self.rpm = -speed if direction else speed
        self._reply(p.SPEED_MODE, bytes([p.RUN_STARTING]))

    def _move(self, payload: bytes) -> None:
        if not (self._bus_mode() and self.enabled):
            return self._reply(p.MOVE_ABSOLUTE, bytes([p.RUN_FAILED]))
        speed = int.from_bytes(payload[0:2], "big")
        coordinate = int.from_bytes(payload[3:6], "big", signed=True)
        if speed == 0:  # 11.4.2 stop
            self.target, self.rpm = None, 0.0
            return self._reply(p.MOVE_ABSOLUTE, bytes([p.RUN_COMPLETE]))
        self.target = coordinate
        self.rpm = speed if coordinate >= self.counts else -speed
        self._reply(p.MOVE_ABSOLUTE, bytes([p.RUN_STARTING]))

    # ── time ──

    def step(self, dt: float) -> None:
        if not self.powered:
            return
        delta = self.rpm / 60.0 * p.COUNTS_PER_REV * dt
        if self.target is None:
            self.counts += delta
            return
        remaining = self.target - self.counts
        if abs(delta) >= abs(remaining):
            self.counts = float(self.target)
            self.target, self.rpm = None, 0.0
            self._reply(p.MOVE_ABSOLUTE, bytes([p.RUN_COMPLETE]), unsolicited=True)
        else:
            self.counts += delta


class FakeBus:
    """The transport the adapter sees, wired straight to one simulated driver."""

    def __init__(self, servo: FakeMksServo) -> None:
        self.servo = servo
        self.sent: List[Tuple[int, bytes]] = []
        self.closed = False

    def send(self, can_id: int, data: bytes) -> None:
        self.sent.append((can_id, bytes(data)))
        self.servo.receive(can_id, data)

    def recv(self, timeout: float):
        return self.servo.outbox.popleft() if self.servo.outbox else None

    def close(self) -> None:
        self.closed = True


class FakeHost:
    """The system-manager side: records state, diagnostics and the poll callback."""

    def __init__(self) -> None:
        self.states: List[Tuple[str, str, dict]] = []
        self.statuses: List[Tuple[str, int, str, str]] = []
        self.polls = []
        self.logs: List[str] = []

    def publish_state(self, target_id, state_type, data):
        self.states.append((target_id, state_type, data))

    def report_status(self, target_id, level, code, message, values):
        self.statuses.append((target_id, level, code, message))

    def schedule(self, rate_hz, fn):
        self.polls.append(fn)

        class Handle:
            def cancel(self_inner):
                pass

        return Handle()

    def log(self, message):
        self.logs.append(message)
