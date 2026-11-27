"""
One MKS SERVO driver on a CAN bus: sends commands, collects replies, keeps the last known state.

The bus sits behind `Transport` (send a frame, receive one with a timeout), so the same code runs on a
real SocketCAN interface (`PythonCanTransport`) and in the tests against a simulated driver.
"""

import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Tuple

from . import protocol as p


class Transport(Protocol):
    """A CAN bus as this driver needs it: standard-id frames in and out."""

    def send(self, can_id: int, data: bytes) -> None: ...

    def recv(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        """The next frame addressed to this driver's id, or None after `timeout` seconds."""
        ...

    def close(self) -> None: ...


class PythonCanTransport:
    """SocketCAN through python-can, filtered to one CAN id so each driver's socket only sees its own
    replies. Linux delivers every frame to every open socket on the interface, so one transport per
    driver (as `odrive_can` does) shares the bus without a broker. The interface itself — bitrate,
    `ip link set can0 up` — is the OS's job (polyflow-os `can0-up` / `can1-up`)."""

    def __init__(self, can_id: int, channel: str = "can0", interface: str = "socketcan", bitrate: Optional[int] = None) -> None:
        import can  # imported here so the protocol and the tests need no python-can

        self._can = can
        kwargs = {"interface": interface, "channel": channel, "can_filters": [{"can_id": int(can_id), "can_mask": 0x7FF, "extended": False}]}
        if bitrate and interface != "socketcan":
            kwargs["bitrate"] = int(bitrate)
        self._bus = can.Bus(**kwargs)

    def send(self, can_id: int, data: bytes) -> None:
        self._bus.send(self._can.Message(arbitration_id=int(can_id), data=bytes(data), is_extended_id=False))

    def recv(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        msg = self._bus.recv(timeout=timeout)
        if msg is None or msg.is_extended_id or msg.is_error_frame:
            return None
        return int(msg.arbitration_id), bytes(msg.data)

    def close(self) -> None:
        self._bus.shutdown()


@dataclass
class AxisState:
    counts: Optional[int] = None
    """Accumulated encoder value (0x31): the coordinate the F5 move targets."""
    rpm: Optional[int] = None
    run_status: Optional[int] = None
    """The last F5/F6 reply status (RUN_STARTING / RUN_COMPLETE / …)."""
    last_reply_s: Optional[float] = None


class MksServoAxis:
    """One driver at `can_id`. Methods return the reply status where the command is acknowledged,
    or None when no reply came (`timeout`)."""

    def __init__(self, transport: Transport, can_id: int, clock: Callable[[], float] = time.monotonic, timeout_s: float = 0.05) -> None:
        self.transport = transport
        self.can_id = int(can_id)
        self.clock = clock
        self.timeout_s = timeout_s
        self.state = AxisState()
        self.errors = 0
        """Replies dropped as malformed (bad checksum, wrong length) — surfaced as a diagnostic."""

    # ── sending ──

    def send(self, data: bytes) -> None:
        self.transport.send(self.can_id, data)

    def request(self, data: bytes, timeout: Optional[float] = None) -> Optional[int]:
        """Send and wait for the reply with the same code; returns its status byte. Any other frame that
        arrives meanwhile (a poll reply, a move completing) is still taken into the state."""
        code = data[0]
        self.send(data)
        deadline = self.clock() + (self.timeout_s if timeout is None else timeout)
        while True:
            remaining = deadline - self.clock()
            if remaining <= 0:
                return None
            reply = self._receive(remaining)
            if reply is not None and reply[0] == code:
                return p.decode_status(reply[1]) if reply[1] else None

    # ── replies ──

    def drain(self, timeout: float = 0.0) -> int:
        """Take every reply waiting on the bus into the state; returns how many. The first wait is
        `timeout`, then it only takes what's already there."""
        n = 0
        while self._receive(timeout if n == 0 else 0.0) is not None:
            n += 1
        return n

    def _receive(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        frame = self.transport.recv(timeout)
        if frame is None:
            return None
        can_id, data = frame
        if can_id != self.can_id:
            return None
        try:
            code, payload = p.parse(can_id, data)
        except p.ProtocolError:
            self.errors += 1
            return None
        self.state.last_reply_s = self.clock()
        try:
            if code == p.READ_ENCODER_ADDITION:
                self.state.counts = p.decode_addition(payload)
            elif code == p.READ_SPEED:
                self.state.rpm = p.decode_speed(payload)
            elif code in (p.MOVE_ABSOLUTE, p.SPEED_MODE):
                self.state.run_status = p.decode_status(payload)
        except p.ProtocolError:
            self.errors += 1
        return code, payload

    # ── commands ──

    def configure_bus_mode(self) -> Optional[int]:
        """Bus FOC mode (SR_vFOC): the only mode family that accepts motion over CAN (Part 9)."""
        return self.request(p.set_work_mode(self.can_id, p.MODE_SR_VFOC))

    def set_response(self, respond: bool = True, active: bool = False) -> Optional[int]:
        return self.request(p.set_response(self.can_id, respond, active))

    def enable(self, on: bool) -> Optional[int]:
        return self.request(p.enable(self.can_id, on))

    def set_zero(self) -> Optional[int]:
        return self.request(p.read(self.can_id, p.SET_ZERO))

    def move_to(self, counts: int, rpm: float, acc: int) -> None:
        """Fire and forget: a stream of position targets must not wait on each acknowledgement (F5
        accepts a new target mid-move, 11.4 note 2). The reply is taken in by the next drain."""
        self.send(p.move_absolute(self.can_id, counts, rpm, acc))

    def run_at(self, rpm: float, acc: int) -> None:
        self.send(p.speed_mode(self.can_id, rpm, acc))

    def stop(self, acc: int = 0) -> None:
        self.send(p.speed_mode(self.can_id, 0, acc))

    def emergency_stop(self) -> None:
        self.send(p.emergency_stop(self.can_id, priority=True))

    def poll(self) -> None:
        """Ask for position and speed; the replies land in the state on the next drain."""
        self.send(p.read(self.can_id, p.READ_ENCODER_ADDITION))
        self.send(p.read(self.can_id, p.READ_SPEED))

    def reply_age_s(self) -> Optional[float]:
        return None if self.state.last_reply_s is None else self.clock() - self.state.last_reply_s

    def close(self) -> None:
        self.transport.close()
