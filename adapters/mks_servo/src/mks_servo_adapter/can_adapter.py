"""
MksServoCANAdapter — one joint driven by an MKS SERVO42D / 57D / 28D over CAN.

Consumes `JointCommand` (position and velocity) and `HardwareLifecycle`; publishes `JointState` by
polling the driver's encoder and speed. Units at the boundary are the joint's SI units (radians,
rad/s); `gear_ratio` (motor turns per joint turn, from the joint's transmission) converts to the
motor side, and `invert` flips a motor that turns the other way.

Startup, in order: bus FOC mode (`82 05`), answer commands but don't report move completion
(`8C 01 00` — the position stream is polled, and an unsolicited reply per target would double the bus
load), enable (`F3 01`), and optionally set the present position as zero (`92`). Without a zero the
power-on position is the zero (11.4.3), so the arm is powered up in its home pose.

Positions go out as F5 absolute-coordinate moves, which take a new target mid-move (11.4 note 2), so a
stream of targets from a trajectory follows smoothly. Torque mode has no MKS equivalent.
"""

import math
from typing import Any, Callable, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    JOINT_MODE_POSITION,
    JOINT_MODE_TORQUE,
    JOINT_MODE_VELOCITY,
    LEVEL_ERROR,
    LEVEL_OK,
    LEVEL_WARN,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ENABLE,
    LIFECYCLE_ESTOP,
)

from . import protocol as p
from .axis import MksServoAxis, PythonCanTransport, Transport


class MksServoCANAdapter(HardwareAdapter):
    driver_name = "mks_servo_can"
    command_types = ["JointCommand", "HardwareLifecycle"]
    state_types = ["JointState"]

    transport_factory: Optional[Callable[["MksServoCANAdapter"], Transport]] = None
    """Tests set this to wire a simulated driver; the robot uses SocketCAN."""

    def configure(self) -> None:
        self.can_id = int(self.params["can_id"])
        if not 1 <= self.can_id <= 2047:
            raise ValueError(f"mks_servo_can: can_id {self.can_id} outside 1..2047 (0 is broadcast)")
        self.channel = str(self.params.get("channel", "can0"))
        self.interface = str(self.params.get("interface", "socketcan"))
        bitrate = self.params.get("bitrate")
        self.bitrate = int(bitrate) if bitrate else None

        self.gear_ratio = float(self.params.get("gear_ratio", 1.0) or 1.0)
        self.sign = -1.0 if bool(self.params.get("invert", False)) else 1.0
        self.move_speed_rpm = float(self.params.get("move_speed_rpm", 600))
        self.max_speed_rpm = min(float(self.params.get("max_speed_rpm", p.MAX_SPEED_RPM)), p.MAX_SPEED_RPM)
        self.acc = int(self.params.get("acceleration", 100))
        self.poll_hz = float(self.params.get("poll_hz", 50.0))
        self.reply_timeout_s = float(self.params.get("reply_timeout_s", 0.5))
        self.zero_on_start = bool(self.params.get("zero_on_start", False))

        self._axis: Optional[MksServoAxis] = None
        self._mode = JOINT_MODE_POSITION
        self._enabled = False
        self._poll_handle: Any = None
        self._warned_torque = False
        self._errors_reported = 0

    # ── lifecycle ──

    def start(self) -> None:
        try:
            transport = self.transport_factory(self) if self.transport_factory else PythonCanTransport(self.can_id, self.channel, self.interface, self.bitrate)
        except ImportError:
            self.report_status(LEVEL_ERROR, "python_can_missing", "python-can is not installed")
            return
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "bus_open_failed", f"{self.channel}: {exc}")
            return
        self._axis = MksServoAxis(transport, self.can_id)

        # Each step answers 1 on success; no answer at all means nothing is listening at this id —
        # the driver is off, on another bitrate, or set to another CAN id.
        if self._axis.configure_bus_mode() is None:
            self.report_status(
                LEVEL_ERROR,
                "no_reply",
                f"No MKS driver answered at CAN id {self.can_id} on {self.channel}. Check its power, its CanID menu setting, and that its CanRate matches the bus.",
                {"can_id": str(self.can_id), "channel": self.channel},
            )
        self._axis.set_response(respond=True, active=False)
        if self.zero_on_start:
            self._axis.set_zero()
        self._enabled = self._axis.enable(True) == 1
        if self._enabled:
            self.report_status(LEVEL_OK, "ready", f"MKS driver {self.can_id} enabled on {self.channel}")

        self._poll_handle = self.schedule_poll(self.poll_hz, self._poll_state)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "JointCommand":
            self._joint_command(data)
        elif command_type == "HardwareLifecycle":
            self._lifecycle(data)

    def estop(self) -> None:
        if self._axis is not None:
            try:
                self._axis.emergency_stop()
            except Exception as exc:
                self.log(f"estop failed: {exc}")

    def shutdown(self) -> None:
        if self._poll_handle is not None:
            try:
                self._poll_handle.cancel()
            except Exception:
                pass
            self._poll_handle = None
        axis, self._axis = self._axis, None
        if axis is None:
            return
        # Stop, but stay enabled: a driver that lets go drops an arm. Disabling is an explicit
        # HardwareLifecycle DISABLE.
        try:
            axis.stop(acc=self.acc)
        except Exception:
            pass
        try:
            axis.close()
        except Exception as exc:
            self.log(f"bus close error: {exc}")

    # ── commands ──

    def _joint_command(self, data: dict) -> None:
        axis = self._axis
        if axis is None:
            return
        mode = int(data.get("mode", self._mode))
        try:
            if mode == JOINT_MODE_POSITION and data.get("has_position"):
                counts = self._joint_to_counts(float(data["position"]))
                if not p.INT24_MIN <= counts <= p.INT24_MAX:
                    self.report_status(LEVEL_WARN, "target_out_of_range", f"Target {data['position']:.3f} rad is beyond the driver's ±{p.INT24_MAX} count range at gear ratio {self.gear_ratio}.")
                    counts = max(p.INT24_MIN, min(p.INT24_MAX, counts))
                rpm = self.move_speed_rpm
                if data.get("has_velocity") and float(data.get("velocity", 0.0)) != 0.0:
                    rpm = abs(self._joint_rad_s_to_rpm(float(data["velocity"])))
                axis.move_to(counts, min(rpm, self.max_speed_rpm), self.acc)
                self._mode = mode
            elif mode == JOINT_MODE_VELOCITY and data.get("has_velocity"):
                rpm = self.sign * self._joint_rad_s_to_rpm(float(data["velocity"]))
                axis.run_at(max(-self.max_speed_rpm, min(self.max_speed_rpm, rpm)), self.acc)
                self._mode = mode
            elif mode == JOINT_MODE_TORQUE:
                if not self._warned_torque:
                    self._warned_torque = True
                    self.report_status(LEVEL_WARN, "torque_unsupported", "MKS SERVO drivers have no torque mode; use position or velocity.")
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))

    def _lifecycle(self, data: dict) -> None:
        axis = self._axis
        if axis is None:
            return
        request = int(data.get("request", -1))
        try:
            if request == LIFECYCLE_ENABLE:
                self._enabled = axis.enable(True) == 1
            elif request == LIFECYCLE_DISABLE:
                axis.enable(False)
                self._enabled = False
            elif request == LIFECYCLE_ESTOP:
                axis.emergency_stop()
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "lifecycle_failed", str(exc), {"request": str(request)})

    # ── state ──

    def _poll_state(self) -> None:
        axis = self._axis
        if axis is None:
            return
        try:
            axis.drain(0.0)  # last cycle's replies
            axis.poll()
            axis.drain(0.002)
        except Exception as exc:
            self.log(f"poll failed: {exc}")
            return

        if axis.errors > self._errors_reported:
            self._errors_reported = axis.errors
            self.report_status(LEVEL_WARN, "bad_frames", f"{axis.errors} malformed replies from CAN id {self.can_id} (checksum or length)")

        age = axis.reply_age_s()
        connected = age is not None and age < self.reply_timeout_s
        state = axis.state
        self.publish_state(
            "JointState",
            {
                "joint_id": self.target_id,
                "position": self._counts_to_joint(state.counts) if state.counts is not None else 0.0,
                "velocity": self.sign * p.rpm_to_rad_s(state.rpm) / self.gear_ratio if state.rpm is not None else 0.0,
                "effort": 0.0,
                "mode": self._mode,
                "closed_loop": self._enabled,
                "connected": connected,
            },
        )

    # ── units ──

    def _joint_to_counts(self, joint_rad: float) -> int:
        return p.radians_to_counts(self.sign * joint_rad * self.gear_ratio)

    def _counts_to_joint(self, counts: int) -> float:
        return self.sign * p.counts_to_radians(counts) / self.gear_ratio

    def _joint_rad_s_to_rpm(self, joint_rad_s: float) -> float:
        return p.rad_s_to_rpm(joint_rad_s * self.gear_ratio)
