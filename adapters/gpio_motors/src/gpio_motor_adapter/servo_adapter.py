"""
GpioServoAdapter: one hobby (PWM) servo whose signal wire goes straight to a Pi GPIO pin, driven as a
JOINT.

The same mapping as `pca9685_servo` (servo travel [angle_min_rad, angle_max_rad] → [pulse_min_us,
pulse_max_us], the joint at `zero_rad` of travel, `gear_ratio` servo radians per joint radian), with the
pulses timed by lgpio instead of a PCA9685. Software-timed pulses jitter by a few microseconds under
load, which a servo shows as a faint buzz; for more than a couple of servos, a PCA9685 HAT is the better
part.

The servo's power must NOT come from the Pi's 5 V pin: a stalled MG996R draws over 2 A. Feed it from
its own supply and join the grounds.
"""

import math
from typing import Any, Callable, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    JOINT_MODE_POSITION,
    LEVEL_ERROR,
    LEVEL_WARN,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ENABLE,
    LIFECYCLE_ESTOP,
)

from . import gpio
from .gpio import GpioBackend, pin_param


class GpioServoAdapter(HardwareAdapter):
    driver_name = "gpio_servo"
    command_types = ["JointCommand", "ServoCommand", "HardwareLifecycle"]
    state_types = ["JointState"]

    backend_factory: Optional[Callable[[], GpioBackend]] = None

    def configure(self) -> None:
        self.pin = pin_param(self.params, "signal_pin")
        self.frequency_hz = float(self.params.get("frequency_hz", 50.0))
        self.pulse_min_us = float(self.params.get("pulse_min_us", 500))
        self.pulse_max_us = float(self.params.get("pulse_max_us", 2500))
        self.angle_min_rad = float(self.params.get("angle_min_rad", -math.pi / 2))
        self.angle_max_rad = float(self.params.get("angle_max_rad", math.pi / 2))
        self.zero_rad = float(self.params.get("zero_rad", 0.0))
        self.gear_ratio = float(self.params.get("gear_ratio", 1.0) or 1.0)
        self.sign = -1.0 if bool(self.params.get("invert", False)) else 1.0
        self.state_hz = float(self.params.get("state_hz", 10.0))

        self._gpio: Optional[GpioBackend] = None
        self._poll: Any = None
        self._enabled = True
        self._position = 0.0
        self._commanded = False

    def start(self) -> None:
        try:
            backend = gpio.acquire(self.backend_factory)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "gpio_unavailable", f"Can't open the Pi's GPIO ({exc}).")
            return
        try:
            backend.claim_output(self.pin, self.target_id, 0)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "pin_unavailable", f"gpio_servo '{self.target_id}' on GPIO{self.pin}: {exc}")
            gpio.release(backend)
            return
        self._gpio = backend
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

    def servo_angle(self, joint_rad: float) -> float:
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
        if self._gpio is None:
            return
        self._enabled = True
        try:
            self._gpio.servo(self.pin, self.pulse_us(joint_rad), self.frequency_hz)
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))
            return
        self._position = (self.servo_angle(joint_rad) - self.zero_rad) / (self.sign * self.gear_ratio)
        self._commanded = True

    def _release(self) -> None:
        self._enabled = False
        if self._gpio is not None:
            try:
                self._gpio.stop(self.pin)
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
                "connected": self._gpio is not None,
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
        if self._gpio is not None:
            self._release()
            self._gpio.release(self.pin)
            gpio.release(self._gpio)
            self._gpio = None
