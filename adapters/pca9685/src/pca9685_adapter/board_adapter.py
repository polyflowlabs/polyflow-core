"""
PCA9685BoardAdapter — owns one PCA9685 (a 16-channel servo HAT) on an I²C bus.

The host for `pca9685_servo`: it opens the bus, sets the chip-wide PWM frequency once (writing PRE_SCALE
puts the whole chip to sleep, so no servo may do it), and hands the chip to every servo driver on it via
`get_resource()`. E-stop and shutdown turn every channel full off.
"""

from typing import Any, Callable, Optional

from polyflow_adapter_sdk import HardwareAdapter, LEVEL_ERROR, LEVEL_OK

from .chip import PCA9685, I2CBus


class PCA9685BoardAdapter(HardwareAdapter):
    driver_name = "pca9685_board"
    command_types: list = []
    state_types: list = []

    bus_factory: Optional[Callable[[int], I2CBus]] = None
    """Tests wire a fake bus; the robot opens /dev/i2c-<n> with smbus2."""

    def configure(self) -> None:
        self.i2c_bus = int(self.params.get("i2c_bus", 1))
        address = self.params.get("address", 0x40)
        self.address = int(address, 0) if isinstance(address, str) else int(address)  # "0x41" or 65
        self.frequency_hz = float(self.params.get("frequency_hz", 50.0))
        self._bus: Any = None
        self._chip: Optional[PCA9685] = None

    def start(self) -> None:
        try:
            if self.bus_factory is not None:
                self._bus = self.bus_factory(self.i2c_bus)
            else:
                from smbus2 import SMBus  # imported here so the chip code and the tests need no smbus2

                self._bus = SMBus(self.i2c_bus)
            chip = PCA9685(self._bus, self.address)
            chip.init(self.frequency_hz)
            self._chip = chip
            self.report_status(LEVEL_OK, "ready", f"PCA9685 at {self.address:#04x} on i2c-{self.i2c_bus}, {chip.frequency_hz:.1f} Hz")
        except ImportError:
            self.report_status(LEVEL_ERROR, "smbus2_missing", "smbus2 is not installed")
        except Exception as exc:
            self.report_status(
                LEVEL_ERROR,
                "chip_not_found",
                f"No PCA9685 answered at {self.address:#04x} on i2c-{self.i2c_bus} ({exc}). Check the HAT is seated, I²C is enabled, and its address jumpers.",
            )
            self._chip = None

    def get_resource(self) -> Any:
        return self._chip

    def estop(self) -> None:
        if self._chip is not None:
            try:
                self._chip.all_off()
            except Exception as exc:
                self.log(f"estop failed: {exc}")

    def shutdown(self) -> None:
        self.estop()
        bus, self._bus = self._bus, None
        if bus is not None and hasattr(bus, "close"):
            try:
                bus.close()
            except Exception:
                pass
