"""
The NXP PCA9685 16-channel, 12-bit PWM controller over I²C — the chip on the Adafruit 16-Channel
PWM/Servo HAT and its clones. Pure register access through an injectable bus; no ROS, no hardware.

From the NXP PCA9685 datasheet (Rev. 4): 25 MHz internal oscillator; PRE_SCALE = round(25 MHz /
(4096 × f)) − 1, writable only while MODE1.SLEEP is set; each channel n has LEDn_ON_L/H and
LEDn_OFF_L/H at 0x06 + 4n (12-bit on/off counts); bit 4 of LEDn_OFF_H is "full off".
"""

from typing import Protocol

OSC_HZ = 25_000_000
COUNTS = 4096

MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
ALL_LED_OFF_H = 0xFD
PRE_SCALE = 0xFE

MODE1_RESTART = 0x80
MODE1_AI = 0x20      # register auto-increment, so one block write sets ON and OFF
MODE1_SLEEP = 0x10
MODE1_ALLCALL = 0x01
MODE2_OUTDRV = 0x04  # totem-pole outputs, as servo signal lines want
FULL_OFF = 0x10      # bit 4 of LEDn_OFF_H / ALL_LED_OFF_H

PRESCALE_MIN, PRESCALE_MAX = 3, 255


class I2CBus(Protocol):
    """The slice of smbus2.SMBus this needs."""

    def write_byte_data(self, address: int, register: int, value: int) -> None: ...

    def read_byte_data(self, address: int, register: int) -> int: ...

    def write_i2c_block_data(self, address: int, register: int, data: list) -> None: ...


def prescale_for(frequency_hz: float) -> int:
    """PRE_SCALE for an output frequency (datasheet eq. 1), clamped to the chip's 3..255."""
    value = int(round(OSC_HZ / (COUNTS * float(frequency_hz)))) - 1
    return max(PRESCALE_MIN, min(PRESCALE_MAX, value))


def actual_frequency(prescale: int) -> float:
    """The frequency a PRE_SCALE really gives (it's quantised), which the pulse widths are timed against."""
    return OSC_HZ / (COUNTS * (prescale + 1))


class PCA9685:
    def __init__(self, bus: I2CBus, address: int = 0x40) -> None:
        self.bus = bus
        self.address = int(address)
        self.frequency_hz = 0.0

    def init(self, frequency_hz: float = 50.0) -> None:
        """Totem-pole outputs, all channels off, the PWM frequency set (asleep, as the chip requires), then
        awake with auto-increment."""
        self.bus.write_byte_data(self.address, MODE2, MODE2_OUTDRV)
        self.all_off()
        prescale = prescale_for(frequency_hz)
        self.bus.write_byte_data(self.address, MODE1, MODE1_SLEEP | MODE1_ALLCALL)
        self.bus.write_byte_data(self.address, PRE_SCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, MODE1_AI | MODE1_ALLCALL)
        self.bus.write_byte_data(self.address, MODE1, MODE1_RESTART | MODE1_AI | MODE1_ALLCALL)
        self.frequency_hz = actual_frequency(prescale)

    def set_counts(self, channel: int, on: int, off: int) -> None:
        if not 0 <= channel <= 15:
            raise ValueError(f"PCA9685 channel {channel} outside 0..15")
        on, off = int(on) & 0x0FFF, int(off) & 0x0FFF
        self.bus.write_i2c_block_data(self.address, LED0_ON_L + 4 * channel, [on & 0xFF, on >> 8, off & 0xFF, off >> 8])

    def set_pulse_us(self, channel: int, pulse_us: float) -> int:
        """A pulse `pulse_us` wide at the start of each period; returns the OFF count written."""
        if self.frequency_hz <= 0:
            raise RuntimeError("PCA9685 not initialised")
        counts = int(round(pulse_us * 1e-6 * self.frequency_hz * COUNTS))
        counts = max(0, min(COUNTS - 1, counts))
        self.set_counts(channel, 0, counts)
        return counts

    def off(self, channel: int) -> None:
        """Full off: no pulses, so a servo goes limp."""
        if not 0 <= channel <= 15:
            raise ValueError(f"PCA9685 channel {channel} outside 0..15")
        self.bus.write_i2c_block_data(self.address, LED0_ON_L + 4 * channel, [0, 0, 0, FULL_OFF])

    def all_off(self) -> None:
        self.bus.write_byte_data(self.address, ALL_LED_OFF_H, FULL_OFF)
