from typing import Optional

from common.polyflow_kernel import PolyflowKernel

AXES = ("left_x", "left_y", "right_x", "right_y")


class JointFromGamepadKernel(PolyflowKernel):
    """
    Moves ONE joint from one gamepad stick axis: hold the stick and the joint turns at a speed
    proportional to how far it is pushed; let go and it stops where it is. Emits a
    JointTrajectoryPoint (a position) for the joint's Joint Controller, so a pan-tilt head, a turret
    or a lift can be aimed by hand on the real robot, where nothing else commands it.

    The stick is a VELOCITY: each tick the target position moves by speed × stick × dt, clamped to
    the joint's travel limits. Stick right and stick UP are positive (the gamepad's raw Y is positive
    when pushed down, so it is negated here); `invert` flips the direction for a joint whose positive
    rotation looks the other way.

    At rest the node sends nothing, so other commands (a slider, a sim test) aren't overridden, and
    the next push starts again from where the joint really is (its `joint_state`), not from where
    the integrator was left. While moving, the target never runs more than `max_lead` ahead of the
    joint's reported position, so a joint that lags (a stepper at its speed limit, a stalled servo)
    doesn't wind up and keep going after the stick is released.

    Subscriptions:
        axes:         GamepadAxes  — the Gamepad's normalized sticks.
        joint_state:  JointState   — the joint's Joint Controller `state` (its real position).

    Publications:
        trajectory:   JointTrajectoryPoint — the joint's target position.

    Parameters:
        joint_id:      The joint to move. The trajectory is addressed to it.
        axis:          Which stick axis: left_x, left_y, right_x or right_y.
        speed:         Full stick → this many rad/s (m/s for a slide).
        invert:        Reverse the direction.
        min_position / max_position:  Travel limits (bound to the joint's lower/upper limits).
        max_lead:      How far (rad) the target may run ahead of the reported position.
        rate_hz:       Integration rate.
    """

    def setup(self):
        self.joint_id: str = str(self.get_param("joint_id", "") or "")
        axis = str(self.get_param("axis", "left_x") or "left_x")
        self.axis: str = axis if axis in AXES else "left_x"
        self.speed: float = self._number("speed") if self._number("speed") is not None else 1.0
        self.invert: bool = bool(self.get_param("invert", False))
        self.min_position: Optional[float] = self._number("min_position")
        self.max_position: Optional[float] = self._number("max_position")
        lead = self._number("max_lead")
        self.max_lead: Optional[float] = lead if lead is not None and lead > 0 else None
        self.rate_hz: float = self._number("rate_hz") or 50.0
        self.dt: float = 1.0 / max(self.rate_hz, 1e-3)

        self._stick: float = 0.0
        self._position: Optional[float] = None  # the joint's last reported position
        self._target: Optional[float] = None  # None = idle: re-seed from _position on the next push
        self._warned = False

    def _number(self, key: str) -> Optional[float]:
        value = self.get_param(key, None)
        try:
            return None if value is None or value == "" else float(value)
        except (TypeError, ValueError):
            return None

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return
        if pin_id == "axes":
            if data.get("connected") is False:
                self._stick = 0.0
                return
            value = float(data.get(self.axis, 0.0) or 0.0)
            # Raw stick Y is positive pushed DOWN; up should be positive.
            if self.axis.endswith("_y"):
                value = -value
            self._stick = max(-1.0, min(1.0, -value if self.invert else value))
        elif pin_id == "joint_state":
            names = data.get("name") or []
            positions = data.get("position") or []
            if not positions:
                return
            # By name; an unnamed single-joint state is taken to be this joint's.
            index = names.index(self.joint_id) if self.joint_id in names else (0 if not names and len(positions) == 1 else -1)
            if index < 0 or index >= len(positions):
                return
            try:
                self._position = float(positions[index])
            except (TypeError, ValueError):
                return

    def _clamp(self, value: float) -> float:
        if self.min_position is not None and value < self.min_position:
            value = self.min_position
        if self.max_position is not None and value > self.max_position:
            value = self.max_position
        return value

    def poll(self):
        if not self.should_run(trigger_info={"pin_id": "__tick__"}):
            return
        if abs(self._stick) < 1e-6:
            self._target = None
            return
        if not self.joint_id:
            if not self._warned:
                self._warned = True
                self.log("Joint From Gamepad needs its joint; nothing sent.")
            return

        if self._target is None:
            # Start from where the joint is. Without feedback yet, start from 0 (the zero pose a
            # stepper counts from at power-on), clamped into the joint's travel.
            self._target = self._position if self._position is not None else 0.0

        target = self._target + self.speed * self._stick * self.dt
        if self.max_lead is not None and self._position is not None:
            target = max(self._position - self.max_lead, min(self._position + self.max_lead, target))
        target = self._clamp(target)
        self._target = target
        self.emit("trajectory", {"name": [self.joint_id], "positions": [target], "velocities": [], "effort": []})
