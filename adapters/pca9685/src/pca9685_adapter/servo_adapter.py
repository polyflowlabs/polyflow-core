"""
PCA9685ServoAdapter — one hobby (PWM) servo on a PCA9685 channel, driven as a JOINT.

Consumes `JointCommand` (position, the joint's radians) — what a Joint Controller sends — and
`ServoCommand` (position) for the servo-shaped nodes; publishes `JointState`. A PWM servo has no feedback,
so the state echoes the last commanded position (`connected` while the chip is there).

The mapping is linear: [angle_min_rad, angle_max_rad] of SERVO travel → [pulse_min_us, pulse_max_us],
with the joint at `zero_rad` of servo travel when the joint reads 0 and `gear_ratio` servo radians per
joint radian (a servo horn on a linkage or gears). `invert` flips a servo mounted the other way round.
"""

import math
from typing import Any, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    JOINT_MODE_POSITION,
    LEVEL_ERROR,
    LEVEL_WARN,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ENABLE,
    LIFECYCLE_ESTOP,
)

from .chip import PCA9685


class PCA9685ServoAdapter(HardwareAdapter):
    driver_name = "pca9685_servo"
    command_types = ["JointCommand", "ServoCommand", "HardwareLifecycle"]
    state_types = ["JointState"]
    requires = ["pca9685_board"]

    def configure(self) -> None:
        self.channel = int(self.params["channel"])
        if not 0 <= self.channel <= 15:
            raise ValueError(f"pca9685_servo: channel {self.channel} outside 0..15")
        self.pulse_min_us = float(self.params.get("pulse_min_us", 500))
        self.pulse_max_us = float(self.params.get("pulse_max_us", 2500))
        self.angle_min_rad = float(self.params.get("angle_min_rad", -math.pi / 2))
        self.angle_max_rad = float(self.params.get("angle_max_rad", math.pi / 2))
        self.zero_rad = float(self.params.get("zero_rad", 0.0))
        self.gear_ratio = float(self.params.get("gear_ratio", 1.0) or 1.0)
        self.sign = -1.0 if bool(self.params.get("invert", False)) else 1.0
        self.state_hz = float(self.params.get("state_hz", 10.0))

        self._chip: Optional[PCA9685] = None
        self._poll: Any = None
        self._enabled = True
        self._position = 0.0  # joint radians, last commanded
        self._commanded = False

    def start(self) -> None:
        chip = self.resources.get("pca9685_board")
        if chip is None:
            self.report_status(LEVEL_ERROR, "board_missing", f"No pca9685_board for servo '{self.target_id}' (channel {self.channel}).")
            return
        self._chip = chip
        if self.state_hz > 0:
            self._poll = self.schedule_poll(self.state_hz, self._publish_state)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "JointCommand":
            mode = int(data.get("mode", JOINT_MODE_POSITION))
            if mode != JOINT_MODE_POSITION:
                self.report_status(LEVEL_WARN, "mode_unsupported", "A hobby servo takes positions only.")
                return
            if data.get("has_position"):
                self._move(float(data["position"]))
        elif command_type == "ServoCommand":
            if "torque_enable" in data and not data["torque_enable"]:
                self._release()
                return
            self._move(float(data.get("position", 0.0)))
        elif command_type == "HardwareLifecycle":
            request = int(data.get("request", -1))
            if request in (LIFECYCLE_DISABLE, LIFECYCLE_ESTOP):
                self._release()
            elif request == LIFECYCLE_ENABLE:
                self._enabled = True
                if self._commanded:
                    self._move(self._position)

    # ── the mapping ──

    def servo_angle(self, joint_rad: float) -> float:
        """Joint radians → servo radians, clamped to the servo's travel."""
        angle = self.zero_rad + self.sign * joint_rad * self.gear_ratio
        lo, hi = sorted((self.angle_min_rad, self.angle_max_rad))
        return max(lo, min(hi, angle))

    def pulse_us(self, joint_rad: float) -> float:
        span = self.angle_max_rad - self.angle_min_rad
        if span == 0:
            return (self.pulse_min_us + self.pulse_max_us) / 2
        t = (self.servo_angle(joint_rad) - self.angle_min_rad) / span
        return self.pulse_min_us + t * (self.pulse_max_us - self.pulse_min_us)

    def _move(self, joint_rad: float) -> None:
        if self._chip is None:
            return
        self._enabled = True
        try:
            self._chip.set_pulse_us(self.channel, self.pulse_us(joint_rad))
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))
            return
        # What the servo can actually reach, back in joint radians.
        self._position = (self.servo_angle(joint_rad) - self.zero_rad) / (self.sign * self.gear_ratio)
        self._commanded = True

    def _release(self) -> None:
        self._enabled = False
        if self._chip is not None:
            try:
                self._chip.off(self.channel)
            except Exception as exc:
                self.log(f"release failed: {exc}")

    def _publish_state(self) -> None:
        self.publish_state(
            "JointState",
            {
                "joint_id": self.target_id,
                "position": self._position,
                "velocity": 0.0,
                "effort": 0.0,
                "mode": JOINT_MODE_POSITION,
                "closed_loop": self._enabled,
                "connected": self._chip is not None,
            },
        )

    def estop(self) -> None:
        self._release()

    def shutdown(self) -> None:
        if self._poll is not None:
            try:
                self._poll.cancel()
            except Exception:
                pass
            self._poll = None
