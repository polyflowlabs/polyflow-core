"""
The Raspberry Pi's own GPIO pins, as the three drivers in this adapter use them.

One `GpioBackend` per process, shared: every driver on the robot claims its pins from the same chip
handle, so two drivers handed the same pin is caught at start-up (`PinInUse`) instead of two drivers
fighting over one wire.

The real backend is **lgpio**, because it is the library that works on a Pi 5 (RPi.GPIO does not: the
Pi 5's header is on the RP1 chip). It does software-timed PWM (`tx_pwm`), servo pulses (`tx_servo`) and
queued pulse trains (`tx_pulse`) on any header pin, which is all these drivers need. Tests hand the
drivers a fake through `backend_factory`.
"""

from typing import Callable, Dict, Optional, Protocol


class PinInUse(RuntimeError):
    """A pin another driver on this robot already claimed."""


class GpioBackend(Protocol):
    def claim_output(self, pin: int, owner: str, level: int = 0) -> None: ...

    def write(self, pin: int, level: int) -> None: ...

    def pwm(self, pin: int, frequency_hz: float, duty_percent: float) -> None:
        """Continuous PWM; duty 0 holds the pin low."""

    def servo(self, pin: int, pulse_us: float, frequency_hz: float = 50.0) -> None:
        """Servo pulses; 0 stops them."""

    def pulses(self, pin: int, frequency_hz: float, count: int) -> None:
        """Queue `count` square pulses at `frequency_hz` after any already queued on the pin."""

    def busy(self, pin: int) -> bool:
        """Whether a pulse train is still playing (or queued) on the pin."""

    def stop(self, pin: int) -> None:
        """Cancel PWM / servo pulses / queued pulse trains on the pin and leave it low."""

    def release(self, pin: int) -> None: ...

    def close(self) -> None: ...


class LgpioBackend:
    """lgpio on the Pi's 40-pin header (gpiochip0 on a Pi 5 with a current kernel, gpiochip4 on older ones)."""

    def __init__(self, chip: Optional[int] = None) -> None:
        import lgpio  # imported here so the drivers and their tests need no lgpio

        self._lg = lgpio
        self._handle = self._open(chip)
        self._owners: Dict[int, str] = {}

    def _open(self, chip: Optional[int]) -> int:
        lg = self._lg
        if chip is not None:
            return lg.gpiochip_open(int(chip))
        # The header's chip is the one whose label is the SoC's pin controller (pinctrl-rp1 on a Pi 5,
        # pinctrl-bcm2711 on a Pi 4); its number moved between kernels.
        for n in range(0, 8):
            try:
                handle = lg.gpiochip_open(n)
            except Exception:
                continue
            try:
                info = lg.gpio_get_chip_info(handle)
                label = str(info[3] if len(info) > 3 else "")
            except Exception:
                label = ""
            if label.startswith("pinctrl-"):
                return handle
            lg.gpiochip_close(handle)
        raise RuntimeError("No GPIO chip with a pinctrl- label (is this a Raspberry Pi?)")

    def claim_output(self, pin: int, owner: str, level: int = 0) -> None:
        if pin in self._owners and self._owners[pin] != owner:
            raise PinInUse(f"GPIO{pin} is already used by '{self._owners[pin]}'")
        self._lg.gpio_claim_output(self._handle, pin, level)
        self._owners[pin] = owner

    def write(self, pin: int, level: int) -> None:
        self._lg.gpio_write(self._handle, pin, 1 if level else 0)

    def pwm(self, pin: int, frequency_hz: float, duty_percent: float) -> None:
        self._lg.tx_pwm(self._handle, pin, frequency_hz, max(0.0, min(100.0, duty_percent)))

    def servo(self, pin: int, pulse_us: float, frequency_hz: float = 50.0) -> None:
        self._lg.tx_servo(self._handle, pin, int(round(pulse_us)), int(round(frequency_hz)))

    def pulses(self, pin: int, frequency_hz: float, count: int) -> None:
        if count <= 0 or frequency_hz <= 0:
            return
        period_us = 1e6 / frequency_hz
        on = max(2, int(period_us / 2))  # StepStick drivers want ≥1–2 µs high
        off = max(2, int(period_us) - on)
        self._lg.tx_pulse(self._handle, pin, on, off, 0, int(count))

    def busy(self, pin: int) -> bool:
        return self._lg.tx_busy(self._handle, pin, self._lg.TX_PWM) == 1

    def stop(self, pin: int) -> None:
        self._lg.tx_pwm(self._handle, pin, 0, 0)
        self._lg.gpio_write(self._handle, pin, 0)

    def release(self, pin: int) -> None:
        if self._owners.pop(pin, None) is not None:
            try:
                self._lg.gpio_free(self._handle, pin)
            except Exception:
                pass

    def close(self) -> None:
        try:
            self._lg.gpiochip_close(self._handle)
        except Exception:
            pass


# ── one backend per process ─────────────────────────────────────────────────

_shared: Optional[GpioBackend] = None
_refs = 0


def acquire(factory: Optional[Callable[[], GpioBackend]] = None) -> GpioBackend:
    """The process's backend. A test's `factory` bypasses sharing (it hands back its own fake)."""
    global _shared, _refs
    if factory is not None:
        return factory()
    if _shared is None:
        _shared = LgpioBackend()
    _refs += 1
    return _shared


def release(backend: GpioBackend) -> None:
    global _shared, _refs
    if backend is not _shared:
        return
    _refs -= 1
    if _refs <= 0 and _shared is not None:
        _shared.close()
        _shared = None
        _refs = 0


def pin_param(params: dict, key: str, required: bool = True) -> Optional[int]:
    """A BCM pin number from hardware.json: 12, "12" or "GPIO12". Header pins are 0–27."""
    raw = params.get(key)
    if raw is None or raw == "":
        if required:
            raise ValueError(f"'{key}' is required")
        return None
    text = str(raw).strip().upper()
    if text.startswith("GPIO"):
        text = text[4:]
    pin = int(text)
    if not 0 <= pin <= 27:
        raise ValueError(f"'{key}' = {raw} is not a header GPIO (0–27)")
    return pin
