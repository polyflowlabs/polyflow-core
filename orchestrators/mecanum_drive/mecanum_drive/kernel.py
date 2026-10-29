import time

from common.polyflow_kernel import PolyflowKernel


_ZERO_TWIST = {
    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
}

_MOTORS = ("front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor")


class MecanumDriveKernel(PolyflowKernel):
    """
    Portable kinematics logic for a 4-wheel mecanum base.

    Converts (forward, sideways, turn) velocity commands into per-wheel speed
    commands. All inputs and outputs are plain dicts — no ROS or hardware
    dependencies.

    Two cmd_vel inputs with priority, as in Differential Drive:
        cmd_vel_teleop      — operator/gamepad input (the Gamepad in `6dof`
                              mode: left stick = linear.x / linear.y, right
                              stick X = angular.z). Wins whenever a message
                              has arrived in the last `teleop_timeout_s`.
        cmd_vel_automated   — autonomous planner input. Used when teleop is
                              stale (or has never published).

    Input pins (as dicts), ROS body frame (+x forward, +y left, +z up):
        cmd_vel_teleop / cmd_vel_automated
                            — {"linear": {"x", "y"}, "angular": {"z"}}

    Output pins (as dicts):
        front_left_motor, front_right_motor, back_left_motor, back_right_motor
                            — {"data": <normalized [-1, 1]>}

    The standard mecanum mix, for wheels whose rollers form an X seen from
    above (front-left and back-right one hand, front-right and back-left the
    other):
        front_left  = x - y - z      front_right = x + y + z
        back_left   = x + y - z      back_right  = x - y + z
    Every wheel's positive command drives the robot forward. A command that
    would exceed 1 scales ALL four down together, so the direction of travel
    is kept and only the speed drops.

    Inputs and outputs are normalized to [-1, 1]; the downstream motor
    controllers scale by their max_speed (the joint's max_velocity).

    Parameters:
        teleop_timeout_s:   Seconds without a teleop message before we fall
                            back to automated (default: 1.0).
    """

    def setup(self):
        self.teleop_timeout_s = float(self.get_param("teleop_timeout_s", 1.0))

        self._last_teleop_cmd = _ZERO_TWIST
        self._last_teleop_ts = 0.0
        self._last_automated_cmd = _ZERO_TWIST

    @staticmethod
    def mix(x: float, y: float, z: float) -> tuple:
        """(front_left, front_right, back_left, back_right), scaled into [-1, 1] together."""
        wheels = (x - y - z, x + y + z, x + y - z, x - y + z)
        peak = max(1.0, *(abs(w) for w in wheels))
        return tuple(w / peak for w in wheels)

    def _emit_motor_commands(self, wheels: tuple):
        for pin, value in zip(_MOTORS, wheels):
            self.emit(pin, {"data": value})

    def _select_command(self) -> dict:
        teleop_age = time.monotonic() - self._last_teleop_ts
        if self._last_teleop_ts > 0.0 and teleop_age < self.teleop_timeout_s:
            return self._last_teleop_cmd
        return self._last_automated_cmd

    @staticmethod
    def _axis(cmd: dict, part: str, key: str) -> float:
        try:
            return float(cmd.get(part, {}).get(key, 0.0) or 0.0)
        except (TypeError, ValueError, AttributeError):
            return 0.0

    def _emit_from_command(self, cmd: dict):
        x = self._axis(cmd, "linear", "x")
        y = self._axis(cmd, "linear", "y")
        z = self._axis(cmd, "angular", "z")
        self._emit_motor_commands(self.mix(x, y, z))

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "cmd_vel_teleop":
            self._last_teleop_cmd = data
            self._last_teleop_ts = time.monotonic()
        elif pin_id == "cmd_vel_automated":
            self._last_automated_cmd = data
        else:
            return

        self._emit_from_command(self._select_command())
