"""
The display drivers: a 128x64 OLED showing the face, driven as a DEVICE (its hardware row is the
display's own placement, not a joint).

Consumes `DisplayCommand` (expression + gaze, from the Face node) and `HardwareLifecycle`. Between
commands it animates on its own, so a node only says what it wants:

- the gaze eases toward the commanded one, so the eyes glide rather than jump;
- the eyes blink every few seconds (not while sleepy or off).

Frames are drawn at `fps` and only pages that changed go over I2C, so an idle face costs almost nothing.
At the Pi's default 100 kHz I2C a full frame is ~100 ms; a blink touches a few pages.
"""

import random
import time
from typing import Any, Callable, Optional

from polyflow_adapter_sdk import HardwareAdapter, LEVEL_ERROR, LEVEL_OK, LIFECYCLE_DISABLE, LIFECYCLE_ENABLE

from . import face
from .oled import CONTROLLERS, I2CWriter, OledController, SMBusWriter, to_pages

BLINK_S = 0.16          # one blink, closed and open again
BLINK_EVERY_S = (2.5, 5.5)
GAZE_EASE = 0.35        # fraction of the way to the commanded gaze, per frame


class OledDisplayAdapter(HardwareAdapter):
    controller = "sh1106"
    default_address = 0x3C
    command_types = ["DisplayCommand", "HardwareLifecycle"]
    state_types: list = []

    bus_factory: Optional[Callable[[int], I2CWriter]] = None
    """Tests wire a fake bus; the robot opens /dev/i2c-<n> with smbus2."""
    clock: Callable[[], float] = time.monotonic
    rng: random.Random = random.Random()

    def configure(self) -> None:
        self.i2c_bus = int(self.params.get("i2c_bus", 1))
        address = self.params.get("i2c_address", self.default_address)
        self.address = int(address, 0) if isinstance(address, str) else int(address)
        self.fps = float(self.params.get("fps", 15.0))
        self.flip = bool(self.params.get("flip", False))
        self.contrast = self.params.get("contrast")

        self.expression = str(self.params.get("expression", "neutral"))
        self.target_gaze = (0.0, 0.0)
        self.gaze = (0.0, 0.0)
        self._bus: Any = None
        self._chip: Optional[OledController] = None
        self._poll: Any = None
        self._blink_at = 0.0
        self._next_blink()

    def start(self) -> None:
        try:
            self._bus = self.bus_factory(self.i2c_bus) if self.bus_factory else SMBusWriter(self.i2c_bus)
            chip = CONTROLLERS[self.controller](self._bus, self.address)
            chip.init()
            if self.contrast is not None:
                chip.contrast(int(self.contrast))
            self._chip = chip
            self.report_status(LEVEL_OK, "ready", f"{self.controller.upper()} at {self.address:#04x} on i2c-{self.i2c_bus}")
        except ImportError:
            self.report_status(LEVEL_ERROR, "smbus2_missing", "smbus2 is not installed")
            return
        except Exception as exc:
            self.report_status(
                LEVEL_ERROR,
                "display_not_found",
                f"No {self.controller.upper()} answered at {self.address:#04x} on i2c-{self.i2c_bus} ({exc}). Check its wiring, that I2C is "
                f"enabled, and the address (0x3C or 0x3D).",
            )
            return
        self.frame()
        if self.fps > 0:
            self._poll = self.schedule_poll(self.fps, self.frame)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "DisplayCommand":
            expression = str(data.get("expression") or "neutral")
            self.expression = expression if expression in face.EXPRESSIONS else "neutral"
            self.target_gaze = (float(data.get("gaze_x", 0.0)), float(data.get("gaze_y", 0.0)))
        elif command_type == "HardwareLifecycle":
            request = int(data.get("request", -1))
            if self._chip is not None and request in (LIFECYCLE_DISABLE, LIFECYCLE_ENABLE):
                self._chip.power(request == LIFECYCLE_ENABLE)

    # ── animation ──

    def _next_blink(self) -> None:
        lo, hi = BLINK_EVERY_S
        self._blink_at = self.clock() + lo + (hi - lo) * self.rng.random()

    def openness(self, now: float) -> float:
        """1 open … 0 shut: a quick close-and-open when a blink is due."""
        if self.expression in ("sleepy", "off"):
            return 1.0
        t = now - self._blink_at
        if t < 0:
            return 1.0
        if t >= BLINK_S:
            self._next_blink()
            return 1.0
        half = BLINK_S / 2.0
        return abs(t - half) / half

    def frame(self) -> int:
        """Draw one frame. Returns the pages sent (0 when nothing changed)."""
        if self._chip is None:
            return 0
        gx, gy = self.gaze
        tx, ty = self.target_gaze
        self.gaze = (gx + (tx - gx) * GAZE_EASE, gy + (ty - gy) * GAZE_EASE)
        rows = face.render(self.expression, self.gaze[0], self.gaze[1], self.openness(self.clock()))
        try:
            return self._chip.show(to_pages(rows, self.flip))
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "write_failed", str(exc))
            return 0

    def estop(self) -> None:
        # A face is harmless, and a robot that just stopped is a good time to still look alive.
        pass

    def shutdown(self) -> None:
        if self._poll is not None:
            try:
                self._poll.cancel()
            except Exception:
                pass
            self._poll = None
        if self._chip is not None:
            try:
                self._chip.power(False)
            except Exception:
                pass
        close = getattr(self._bus, "close", None)
        if close:
            close()


class SH1106DisplayAdapter(OledDisplayAdapter):
    driver_name = "sh1106_display"
    controller = "sh1106"
    default_address = 0x3C


class SSD1306DisplayAdapter(OledDisplayAdapter):
    driver_name = "ssd1306_display"
    controller = "ssd1306"
    default_address = 0x3D
