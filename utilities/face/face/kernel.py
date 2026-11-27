import time
from typing import Callable, Dict, Optional

from common.polyflow_kernel import PolyflowKernel

EXPRESSIONS = ("neutral", "happy", "sad", "angry", "surprised", "sleepy", "off")

# A gamepad button → the expression it flashes.
BUTTON_EXPRESSIONS = {"a": "happy", "b": "surprised", "x": "sad", "y": "angry"}


def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if v < lo else hi if v > hi else v


class FaceKernel(PolyflowKernel):
    """
    A robot's face, from what it's doing. Emits a `DisplayCommand` (expression + gaze) for the display's
    adapter, which draws the eyes and animates them (blinks, easing) between commands.

    - Driving: the eyes look where the robot is turning, and a little up going forward (down in reverse).
    - Buttons: a happy, b surprised, x sad, y angry, each for `flash_s` after the press.
    - Idle: no driving for `idle_after_s` and the eyes go sleepy.
    - `set_expression` pin: set a standing expression ("" clears it back to automatic).

    Parameters:
        display_id:    The screen's placement on the robot.
        idle_after_s:  Seconds without driving before the face gets sleepy (0: never).
        flash_s:       How long a button's expression shows.
        look_gain:     How far the eyes follow a turn, 0–1.
        rate_hz:       How often the face is re-evaluated.
    """

    clock: Callable[[], float] = time.monotonic

    def setup(self):
        self.display_id: str = str(self.get_param("display_id", "") or "")
        self.idle_after_s = float(self.get_param("idle_after_s", 8.0))
        self.flash_s = float(self.get_param("flash_s", 2.0))
        self.look_gain = float(self.get_param("look_gain", 1.0))
        self.rate_hz = float(self.get_param("rate_hz", 10.0))

        now = self.clock()
        self._last_motion = now
        self._turn = 0.0
        self._forward = 0.0
        self._standing: Optional[str] = None
        self._flash: Optional[str] = None
        self._flash_until = 0.0
        self._buttons: Dict[str, bool] = {}
        self._sent: Optional[tuple] = None
        self._sent_at = -1e9
        self._warned = False

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return
        now = self.clock()
        if pin_id == "cmd_vel":
            lin = data.get("linear") or {}
            ang = data.get("angular") or {}
            self._forward = _clamp(float(lin.get("x", 0.0) or 0.0))
            self._turn = _clamp(float(ang.get("z", 0.0) or 0.0))
            if abs(self._forward) > 0.05 or abs(self._turn) > 0.05:
                self._last_motion = now
        elif pin_id == "buttons":
            for button, expression in BUTTON_EXPRESSIONS.items():
                pressed = bool(data.get(button, False))
                if pressed and not self._buttons.get(button, False):
                    self._flash, self._flash_until = expression, now + self.flash_s
                    self._last_motion = now  # a button press wakes it up
                self._buttons[button] = pressed
        elif pin_id == "set_expression":
            value = str(data.get("data", "") or "").strip().lower()
            self._standing = value if value in EXPRESSIONS else None
        self._update(now)

    def poll(self):
        if not self.should_run(trigger_info={"pin_id": "__tick__"}):
            return
        self._update(self.clock())

    # ── what to show ──

    def face(self, now: float) -> tuple:
        if self._flash and now < self._flash_until:
            expression = self._flash
        elif self._standing:
            expression = self._standing
        elif self.idle_after_s > 0 and now - self._last_motion >= self.idle_after_s:
            expression = "sleepy"
        else:
            expression = "neutral"
        # Facing the viewer, the robot's left is the viewer's right: a left turn (+z) looks to screen +x.
        gaze_x = _clamp(self._turn * self.look_gain)
        gaze_y = _clamp(-0.4 * self._forward)
        return expression, round(gaze_x, 2), round(gaze_y, 2)

    def _update(self, now: float):
        face = self.face(now)
        # On change, and once a second regardless, so a display that restarted catches up.
        if face == self._sent and now - self._sent_at < 1.0:
            return
        if not self.display_id:
            if not self._warned:
                self._warned = True
                self.log("Face has no display set (display_id); nothing sent.")
            return
        self._sent, self._sent_at = face, now
        expression, gaze_x, gaze_y = face
        self.emit("hw_command", {"display_id": self.display_id, "expression": expression, "gaze_x": gaze_x, "gaze_y": gaze_y})
        self.emit("expression", {"data": expression})
