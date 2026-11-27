"""
StepStickAdapter: one bipolar stepper on a step/dir driver (A4988, DRV8825, TMC2208/2209 in standalone
mode, and anything else in the StepStick footprint), driven as a JOINT from the Pi's GPIO.

Two pins: STEP (one pulse = one microstep) and DIR. EN is wired to GND so the driver is always
energized; give `enable_pin` (active low, as on every StepStick) to let DISABLE de-energize it.

There is no encoder, so the joint's position is the steps the driver has been sent, counted from where
the motor sat at power-on. **The robot must be at its zero pose when it starts** (or `zero_on_start` is
turned off and something else homes it). A stepper that is overloaded skips steps and the count is then
wrong: size the motor with margin.

Motion is a trapezoidal profile, planned in steps by `StepPlanner` and run at `control_hz`: each tick
queues that tick's whole steps as one pulse train (lgpio `tx_pulse`), so the motor sees evenly spaced
pulses. Units:

    microsteps per joint radian = steps_per_rev × microsteps × gear_ratio / 2π

where `gear_ratio` is the whole reduction from motor to joint (the deploy fills it in: the motor's own
gearbox × any belt or gears). `max_speed_rad_s` and `accel_rad_s2` are the JOINT's, and the step rate is
capped at `max_step_rate_hz` (software-timed pulses top out in the tens of kHz).

E-stop stops the pulses at once and keeps the motor energized, so an arm holds where it is rather than
dropping under its own weight.
"""

import math
from typing import Any, Callable, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    JOINT_MODE_POSITION,
    JOINT_MODE_VELOCITY,
    LEVEL_ERROR,
    LEVEL_OK,
    LEVEL_WARN,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ENABLE,
    LIFECYCLE_ESTOP,
)

from . import gpio
from .gpio import GpioBackend, pin_param


class StepPlanner:
    """A trapezoidal profile in microsteps. `tick(dt)` returns the signed whole steps to issue now."""

    def __init__(self, max_speed: float, accel: float) -> None:
        self.max_speed = max(1.0, float(max_speed))  # steps/s
        self.accel = max(1.0, float(accel))  # steps/s²
        self.position = 0  # steps issued
        self.velocity = 0.0  # steps/s
        self.target: Optional[int] = 0
        self.target_velocity = 0.0
        self._carry = 0.0

    def move_to(self, steps: int) -> None:
        self.target = int(steps)

    def run_at(self, steps_per_s: float) -> None:
        self.target = None
        self.target_velocity = max(-self.max_speed, min(self.max_speed, float(steps_per_s)))

    def halt(self) -> None:
        """Stop now, where we are (no deceleration — this is the e-stop)."""
        self.velocity = 0.0
        self._carry = 0.0
        self.target = self.position
        self.target_velocity = 0.0

    @property
    def idle(self) -> bool:
        return self.velocity == 0.0 and (self.target is None and self.target_velocity == 0.0 or self.target == self.position)

    def tick(self, dt: float) -> int:
        if dt <= 0:
            return 0
        dv = self.accel * dt
        if self.target is not None:
            remaining = self.target - self.position
            if remaining == 0 and abs(self.velocity) <= dv:
                self.velocity, self._carry = 0.0, 0.0
                return 0
            direction = 1.0 if remaining > 0 else -1.0
            stopping = self.velocity * self.velocity / (2.0 * self.accel)
            moving_toward = self.velocity * direction > 0
            desired = 0.0 if moving_toward and stopping >= abs(remaining) else direction * self.max_speed
        else:
            desired = self.target_velocity
        self.velocity += max(-dv, min(dv, desired - self.velocity))
        self.velocity = max(-self.max_speed, min(self.max_speed, self.velocity))

        exact = self.velocity * dt + self._carry
        steps = int(exact)  # toward zero; the fraction carries to the next tick
        self._carry = exact - steps

        if self.target is not None:
            remaining = self.target - self.position
            # Never past the target: land on it and stop.
            if remaining == 0 or (steps != 0 and (steps > 0) == (remaining > 0) and abs(steps) >= abs(remaining)):
                steps = remaining
                if abs(self.velocity) <= 2 * dv or remaining == 0:
                    self.velocity, self._carry = 0.0, 0.0
            # Crawling at the end of a decel with a step or two left: finish rather than stall at zero speed.
            elif steps == 0 and self.velocity == 0.0:
                steps = 1 if remaining > 0 else -1
        self.position += steps
        return steps


class StepStickAdapter(HardwareAdapter):
    driver_name = "stepstick_stepper"
    command_types = ["JointCommand", "HardwareLifecycle"]
    state_types = ["JointState"]

    backend_factory: Optional[Callable[[], GpioBackend]] = None

    def configure(self) -> None:
        self.step_pin = pin_param(self.params, "step_pin")
        self.dir_pin = pin_param(self.params, "dir_pin")
        self.enable_pin = pin_param(self.params, "enable_pin", required=False)
        pins = [p for p in (self.step_pin, self.dir_pin, self.enable_pin) if p is not None]
        if len(set(pins)) != len(pins):
            raise ValueError(f"stepstick_stepper '{self.target_id}': its pins must all differ ({pins})")

        self.steps_per_rev = float(self.params.get("steps_per_rev", 200))
        self.microsteps = float(self.params.get("microsteps", 16))
        self.gear_ratio = float(self.params.get("gear_ratio", 1.0) or 1.0)
        self.sign = -1 if bool(self.params.get("invert", False)) else 1
        self.control_hz = float(self.params.get("control_hz", 100.0))
        self.state_hz = float(self.params.get("state_hz", 20.0))
        self.max_step_rate_hz = float(self.params.get("max_step_rate_hz", 20000.0))
        self.zero_on_start = bool(self.params.get("zero_on_start", True))

        self.steps_per_rad = self.steps_per_rev * self.microsteps * self.gear_ratio / (2 * math.pi)
        max_speed = float(self.params.get("max_speed_rad_s", 1.0)) * self.steps_per_rad
        self.rate_limited = max_speed > self.max_step_rate_hz
        self.planner = StepPlanner(min(max_speed, self.max_step_rate_hz), float(self.params.get("accel_rad_s2", 2.0)) * self.steps_per_rad)
        if not self.zero_on_start:
            start = float(self.params.get("start_position_rad", 0.0))
            self.planner.position = self.planner.target = int(round(start * self.steps_per_rad))

        self._gpio: Optional[GpioBackend] = None
        self._polls: list = []
        self._enabled = True
        self._dir_level: Optional[int] = None
        self._mode = JOINT_MODE_POSITION

    def start(self) -> None:
        try:
            backend = gpio.acquire(self.backend_factory)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "gpio_unavailable", f"Can't open the Pi's GPIO ({exc}).")
            return
        try:
            backend.claim_output(self.step_pin, self.target_id, 0)
            backend.claim_output(self.dir_pin, self.target_id, 0)
            if self.enable_pin is not None:
                backend.claim_output(self.enable_pin, self.target_id, 0)  # low = enabled
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "pin_unavailable", f"stepstick_stepper '{self.target_id}': {exc}")
            gpio.release(backend)
            return
        self._gpio = backend
        if self.rate_limited:
            self.report_status(
                LEVEL_WARN,
                "rate_limited",
                f"max_speed_rad_s needs more than {self.max_step_rate_hz:.0f} steps/s at {self.microsteps:g} microsteps; capped at "
                f"{self.max_step_rate_hz / self.steps_per_rad:.2f} rad/s. Fewer microsteps would allow more.",
            )
        else:
            self.report_status(LEVEL_OK, "ready", f"Stepper on STEP GPIO{self.step_pin}, DIR GPIO{self.dir_pin}, {self.steps_per_rad:.1f} steps/rad")
        if self.control_hz > 0:
            self._polls.append(self.schedule_poll(self.control_hz, lambda: self.step(1.0 / self.control_hz)))
        if self.state_hz > 0:
            self._polls.append(self.schedule_poll(self.state_hz, self._publish_state))

    # ── commands ──

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "JointCommand":
            if not self._enabled:
                return
            mode = int(data.get("mode", JOINT_MODE_POSITION))
            if mode == JOINT_MODE_POSITION and data.get("has_position"):
                self._mode = mode
                self.planner.move_to(int(round(float(data["position"]) * self.steps_per_rad)))
            elif mode == JOINT_MODE_VELOCITY and data.get("has_velocity"):
                self._mode = mode
                self.planner.run_at(float(data["velocity"]) * self.steps_per_rad)
            elif mode not in (JOINT_MODE_POSITION, JOINT_MODE_VELOCITY):
                self.report_status(LEVEL_WARN, "mode_unsupported", "A stepper takes positions or velocities, not torque.")
        elif command_type == "HardwareLifecycle":
            request = int(data.get("request", -1))
            if request == LIFECYCLE_ESTOP:
                self.estop()
            elif request == LIFECYCLE_DISABLE:
                self.estop()
                self._set_enabled_pin(False)
            elif request == LIFECYCLE_ENABLE:
                self._enabled = True
                self._set_enabled_pin(True)

    # ── motion ──

    def step(self, dt: float) -> int:
        """One control tick: plan, then queue this tick's pulses. Returns the signed steps sent."""
        if self._gpio is None or not self._enabled:
            return 0
        steps = self.planner.tick(dt)
        if steps == 0:
            return 0
        level = 1 if steps * self.sign > 0 else 0
        try:
            if level != self._dir_level:
                self._gpio.write(self.dir_pin, level)
                self._dir_level = level
            self._gpio.pulses(self.step_pin, abs(steps) / dt, abs(steps))
        except Exception as exc:
            self.report_status(LEVEL_WARN, "step_write_failed", str(exc))
        return steps

    @property
    def position_rad(self) -> float:
        return self.planner.position / self.steps_per_rad

    def _publish_state(self) -> None:
        self.publish_state(
            "JointState",
            {
                "joint_id": self.target_id,
                "position": self.position_rad,
                "velocity": self.planner.velocity / self.steps_per_rad,
                "effort": 0.0,
                "mode": self._mode,
                "closed_loop": False,
                "connected": self._gpio is not None,
            },
        )

    def _set_enabled_pin(self, on: bool) -> None:
        if self._gpio is not None and self.enable_pin is not None:
            try:
                self._gpio.write(self.enable_pin, 0 if on else 1)
            except Exception as exc:
                self.log(f"EN write failed: {exc}")

    def estop(self) -> None:
        self._enabled = False  # until ENABLE: a command queued behind the e-stop must not move it
        self.planner.halt()
        if self._gpio is not None:
            try:
                self._gpio.stop(self.step_pin)
            except Exception as exc:
                self.log(f"stop failed: {exc}")

    def shutdown(self) -> None:
        for poll in self._polls:
            try:
                poll.cancel()
            except Exception:
                pass
        self._polls = []
        if self._gpio is not None:
            self.estop()
            for pin in (self.step_pin, self.dir_pin, self.enable_pin):
                if pin is not None:
                    self._gpio.release(pin)
            gpio.release(self._gpio)
            self._gpio = None
