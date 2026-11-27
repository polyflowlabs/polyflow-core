from typing import Optional

from common.polyflow_kernel import PolyflowKernel


class GripperCommandKernel(PolyflowKernel):
    """
    Turns a button into a gripper position: press to close, press again to open (toggle), or hold to
    close (hold). Emits a JointTrajectoryPoint for the gripper's Joint Controller, so a gamepad can
    work a gripper without anyone hand-tuning positions.

    Parameters:
        joint_id:         The gripper's DRIVEN joint (its motor's — never a jaw that mimics it). The
                          trajectory is addressed to it, and the positions below are read off it.
        open_position:    Where "open" is, in the joint's units. Bound to the joint's lower limit.
        closed_position:  Where "closed" is. Bound to the joint's upper limit.
        mode:             "toggle" (each press flips) or "hold" (closed while held).
        start_closed:     The state a toggle starts from.
        invert:           Swap open and closed (a gripper whose limits run the other way).
    """

    def setup(self):
        self.joint_id: str = str(self.get_param("joint_id", "") or "")
        self.open_position: Optional[float] = self._number("open_position")
        self.closed_position: Optional[float] = self._number("closed_position")
        self.mode: str = str(self.get_param("mode", "toggle"))
        self.invert: bool = bool(self.get_param("invert", False))
        self.closed: bool = bool(self.get_param("start_closed", False))
        self._pressed: bool = False
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
        if pin_id != "button":
            return
        pressed = bool(data.get("data", False))

        if self.mode == "hold":
            closed = pressed
        else:
            # Toggle on the press, not the release, and not on a repeated "still pressed".
            closed = (not self.closed) if (pressed and not self._pressed) else self.closed
        self._pressed = pressed
        if closed == self.closed and self.mode != "hold":
            return
        self.closed = closed
        self._command()

    def _command(self):
        if not self.joint_id or self.open_position is None or self.closed_position is None:
            if not self._warned:
                self._warned = True
                self.log("Gripper Command needs its joint and that joint's travel limits (open/closed positions); nothing sent.")
            return
        closed = self.closed != self.invert
        position = self.closed_position if closed else self.open_position
        self.emit("trajectory", {"name": [self.joint_id], "positions": [position], "velocities": [], "effort": []})
        self.emit("closed", {"data": self.closed})
