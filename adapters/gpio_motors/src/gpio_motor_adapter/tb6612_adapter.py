"""
TB6612MotorAdapter: one brushed DC motor on one channel of a TB6612FNG dual H-bridge, driven from the
Pi's GPIO.

Three pins per channel, from the Toshiba datasheet's truth table:

    IN1  IN2  PWM   →  output
     H    L   PWM      forward, at the PWM's duty
     L    H   PWM      reverse, at the PWM's duty
     H    H    x       short brake
     L    L    H       stop (coast)

STBY must be high for either channel to run. Wire it to 3.3 V, or give it a pin (`stby_pin`); a pin is
set high at start and dropped on e-stop, which stops both channels at once.

Consumes `MotorCommand` (what a Motor Controller sends: SPEED in rad/s, DUTY in [-1, 1], IDLE) and
`HardwareLifecycle`; publishes `MotorState`. There is no encoder input here, so this is open loop:
SPEED is scaled to duty by `max_speed_rad_s`, the speed the motor makes at full duty (the deploy fills
it in from the joint's top speed, which is also the Motor Controller's full scale, so full stick is full
duty). The state echoes the commanded value.

A watchdog stops the motor when a command's `timeout_s` lapses, so a dropped link doesn't leave a rover
driving.
"""

import time
from typing import Any, Callable, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    LEVEL_ERROR,
    LEVEL_OK,
    LEVEL_WARN,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ENABLE,
    LIFECYCLE_ESTOP,
    MOTOR_MODE_DUTY,
    MOTOR_MODE_IDLE,
    MOTOR_MODE_SPEED,
)

from . import gpio
from .gpio import GpioBackend, pin_param


class TB6612MotorAdapter(HardwareAdapter):
    driver_name = "tb6612_motor"
    command_types = ["MotorCommand", "HardwareLifecycle"]
    state_types = ["MotorState"]

    backend_factory: Optional[Callable[[], GpioBackend]] = None
    """Tests wire a fake; the robot uses lgpio."""

    def configure(self) -> None:
        self.pwm_pin = pin_param(self.params, "pwm_pin")
        self.in1_pin = pin_param(self.params, "in1_pin")
        self.in2_pin = pin_param(self.params, "in2_pin")
        self.stby_pin = pin_param(self.params, "stby_pin", required=False)
        pins = [p for p in (self.pwm_pin, self.in1_pin, self.in2_pin, self.stby_pin) if p is not None]
        if len(set(pins)) != len(pins):
            raise ValueError(f"tb6612_motor '{self.target_id}': its pins must all differ ({pins})")
        self.pwm_hz = float(self.params.get("pwm_hz", 1000.0))
        self.max_speed_rad_s = float(self.params.get("max_speed_rad_s", 5.0) or 5.0)
        self.min_duty = float(self.params.get("min_duty", 0.0))
        self.sign = -1.0 if bool(self.params.get("invert", False)) else 1.0
        self.brake_on_stop = bool(self.params.get("brake_on_stop", True))
        self.state_hz = float(self.params.get("state_hz", 10.0))

        self._gpio: Optional[GpioBackend] = None
        self._poll: Any = None
        self._enabled = True
        self._mode = MOTOR_MODE_IDLE
        self._value = 0.0
        self._duty = 0.0
        self._last_cmd = 0.0
        self._timeout_s = 0.0

    def start(self) -> None:
        try:
            backend = gpio.acquire(self.backend_factory)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "gpio_unavailable", f"Can't open the Pi's GPIO ({exc}).")
            return
        try:
            for pin in (self.pwm_pin, self.in1_pin, self.in2_pin):
                backend.claim_output(pin, self.target_id, 0)
            if self.stby_pin is not None:
                backend.claim_output(self.stby_pin, self.target_id, 1)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "pin_unavailable", f"tb6612_motor '{self.target_id}': {exc}")
            gpio.release(backend)
            return
        self._gpio = backend
        self._stop()
        self.report_status(LEVEL_OK, "ready", f"TB6612 channel on PWM GPIO{self.pwm_pin}, IN1 GPIO{self.in1_pin}, IN2 GPIO{self.in2_pin}")
        if self.state_hz > 0:
            self._poll = self.schedule_poll(self.state_hz, self._tick)

    # ── commands ──

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "MotorCommand":
            self._motor_command(data)
        elif command_type == "HardwareLifecycle":
            request = int(data.get("request", -1))
            if request == LIFECYCLE_ENABLE:
                self._enabled = True
                self._set_standby(True)
            elif request in (LIFECYCLE_DISABLE, LIFECYCLE_ESTOP):
                self._enabled = False
                self._stop()

    def _motor_command(self, data: dict) -> None:
        if self._gpio is None:
            return
        if not self._enabled:
            self._stop()
            return
        mode = int(data.get("mode", MOTOR_MODE_IDLE))
        value = float(data.get("value", 0.0))
        if mode == MOTOR_MODE_SPEED:
            duty = value / self.max_speed_rad_s
        elif mode == MOTOR_MODE_DUTY:
            duty = value
        elif mode == MOTOR_MODE_IDLE:
            duty, value = 0.0, 0.0
        else:
            self.report_status(LEVEL_WARN, "unknown_mode", f"MotorCommand.mode={mode}")
            return
        try:
            self.drive(duty)
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))
            return
        self._mode, self._value = mode, value
        self._last_cmd = time.monotonic()
        self._timeout_s = float(data.get("timeout_s", 0.0))

    # ── the H-bridge ──

    def drive(self, duty: float) -> None:
        """Signed duty in [-1, 1] → IN1/IN2 direction and PWM duty. 0 brakes (or coasts)."""
        assert self._gpio is not None
        duty = max(-1.0, min(1.0, duty * self.sign))
        if abs(duty) < 1e-6:
            self._stop()
            return
        magnitude = self.min_duty + (1.0 - self.min_duty) * abs(duty)
        forward = duty > 0
        # Direction first, with PWM low, so the bridge never sees a transient shoot-through state.
        self._gpio.pwm(self.pwm_pin, self.pwm_hz, 0)
        self._gpio.write(self.in1_pin, 1 if forward else 0)
        self._gpio.write(self.in2_pin, 0 if forward else 1)
        self._gpio.pwm(self.pwm_pin, self.pwm_hz, magnitude * 100.0)
        self._duty = magnitude if forward else -magnitude

    def _stop(self) -> None:
        self._mode, self._value, self._duty = MOTOR_MODE_IDLE, 0.0, 0.0
        if self._gpio is None:
            return
        try:
            self._gpio.pwm(self.pwm_pin, self.pwm_hz, 0)
            level = 1 if self.brake_on_stop else 0  # H/H short brake, L/L coast
            self._gpio.write(self.in1_pin, level)
            self._gpio.write(self.in2_pin, level)
        except Exception as exc:
            self.log(f"stop failed: {exc}")

    def _set_standby(self, run: bool) -> None:
        if self._gpio is not None and self.stby_pin is not None:
            try:
                self._gpio.write(self.stby_pin, 1 if run else 0)
            except Exception as exc:
                self.log(f"STBY write failed: {exc}")

    # ── periodic ──

    def _tick(self) -> None:
        if self._timeout_s > 0 and self._mode != MOTOR_MODE_IDLE and time.monotonic() - self._last_cmd > self._timeout_s:
            self._stop()
            self._timeout_s = 0.0
            self.report_status(LEVEL_WARN, "watchdog_timeout", "MotorCommand watchdog fired — stopped")
        self.publish_state(
            "MotorState",
            {
                "motor_id": self.target_id,
                "mode": self._mode,
                "commanded_value": self._value,
                "measured_value": self._value,  # open loop: no encoder read here
                "current": 0.0,
                "connected": self._gpio is not None,
            },
        )

    def estop(self) -> None:
        self._enabled = False
        self._stop()
        self._set_standby(False)

    def shutdown(self) -> None:
        if self._poll is not None:
            try:
                self._poll.cancel()
            except Exception:
                pass
            self._poll = None
        if self._gpio is not None:
            self._stop()
            self._set_standby(False)
            for pin in (self.pwm_pin, self.in1_pin, self.in2_pin, self.stby_pin):
                if pin is not None:
                    self._gpio.release(pin)
            gpio.release(self._gpio)
            self._gpio = None
