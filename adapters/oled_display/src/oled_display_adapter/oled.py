"""
128x64 monochrome OLED controllers over I2C: the SH1106 and the SSD1306.

Both take a control byte before each I2C write (0x00: what follows is commands; 0x40: display RAM) and
store the screen as 8 PAGES of 8 rows: one byte per column per page, bit 0 the page's top row. They
differ in how a frame gets into RAM:

- SSD1306 (datasheet rev 1.1, §10): horizontal addressing mode (0x20 0x00) with a column window
  (0x21 0 127) and page window (0x22 0 7), so a frame is one stream of 1024 bytes. Needs its charge
  pump on (0x8D 0x14).
- SH1106 (datasheet v2.2): no horizontal mode. Its RAM is 132 columns wide with the 128 visible ones
  starting at column 2, so each page is addressed on its own (0xB0+page, then column 2: 0x02 0x10) and
  written as 128 bytes. Uses its DC-DC (0xAD 0x8B) instead of a charge pump.

The bus is injected (`I2CWriter`), so the protocol is tested without hardware.
"""

from typing import List, Protocol, Sequence

WIDTH = 128
HEIGHT = 64
PAGES = HEIGHT // 8

CONTROL_COMMAND = 0x00
CONTROL_DATA = 0x40


class I2CWriter(Protocol):
    def write(self, address: int, data: Sequence[int]) -> None:
        """One I2C write transaction: `data` after the address byte."""


def to_pages(rows: List[List[bool]], flip: bool = False) -> List[bytes]:
    """Rows of booleans → 8 pages of 128 column bytes (bit 0 = the page's top row). `flip` turns the
    picture 180° for a screen mounted upside down."""
    pages = []
    for p in range(PAGES):
        page = bytearray(WIDTH)
        for x in range(WIDTH):
            b = 0
            for bit in range(8):
                y = p * 8 + bit
                on = rows[HEIGHT - 1 - y][WIDTH - 1 - x] if flip else rows[y][x]
                if on:
                    b |= 1 << bit
            page[x] = b
        pages.append(bytes(page))
    return pages


class OledController:
    """Common to both chips: commands, power, and sending only the pages that changed."""

    init_sequence: List[int] = []
    chunk = 128  # data bytes per I2C transaction (plus the control byte)

    def __init__(self, bus: I2CWriter, address: int) -> None:
        self.bus = bus
        self.address = address
        self._shown: List[bytes] = []

    def command(self, *cmds: int) -> None:
        self.bus.write(self.address, [CONTROL_COMMAND, *cmds])

    def data(self, payload: bytes) -> None:
        for i in range(0, len(payload), self.chunk):
            self.bus.write(self.address, [CONTROL_DATA, *payload[i : i + self.chunk]])

    def init(self) -> None:
        self.command(*self.init_sequence)
        self._shown = []
        self.show([bytes(WIDTH)] * PAGES)  # clear: RAM holds noise at power-on
        self.command(0xAF)  # display on

    def contrast(self, level: int) -> None:
        self.command(0x81, max(0, min(255, int(level))))

    def power(self, on: bool) -> None:
        self.command(0xAF if on else 0xAE)

    def show(self, pages: List[bytes]) -> int:
        """Write the pages that differ from what's on screen. Returns how many were sent."""
        sent = 0
        for p, page in enumerate(pages):
            if self._shown and self._shown[p] == page:
                continue
            self.write_page(p, page)
            sent += 1
        self._shown = list(pages)
        return sent

    def write_page(self, page: int, payload: bytes) -> None:
        raise NotImplementedError


class SSD1306(OledController):
    init_sequence = [
        0xAE,        # display off
        0xD5, 0x80,  # clock divide / oscillator
        0xA8, 0x3F,  # multiplex: 64 rows
        0xD3, 0x00,  # display offset 0
        0x40,        # start line 0
        0x8D, 0x14,  # charge pump on
        0x20, 0x00,  # horizontal addressing
        0xA1,        # segment remap: column 127 is SEG0 (so x runs left → right)
        0xC8,        # COM scan from the bottom (so y runs top → bottom)
        0xDA, 0x12,  # COM pins: alternative, for 128x64
        0x81, 0xCF,  # contrast
        0xD9, 0xF1,  # pre-charge
        0xDB, 0x40,  # VCOMH deselect
        0xA4,        # show RAM
        0xA6,        # not inverted
        0x2E,        # no scrolling
    ]

    def write_page(self, page: int, payload: bytes) -> None:
        self.command(0x21, 0, WIDTH - 1, 0x22, page, page)
        self.data(payload)


class SH1106(OledController):
    column_offset = 2  # the 128 visible columns of the 132-column RAM start here

    init_sequence = [
        0xAE,        # display off
        0xD5, 0x80,  # clock divide / oscillator
        0xA8, 0x3F,  # multiplex: 64 rows
        0xD3, 0x00,  # display offset 0
        0x40,        # start line 0
        0xAD, 0x8B,  # DC-DC on
        0xA1,        # segment remap
        0xC8,        # COM scan from the bottom
        0xDA, 0x12,  # COM pins
        0x81, 0x80,  # contrast
        0xD9, 0x22,  # pre-charge
        0xDB, 0x35,  # VCOM deselect
        0x32,        # pump voltage 8.0 V
        0xA4,        # show RAM
        0xA6,        # not inverted
    ]

    def write_page(self, page: int, payload: bytes) -> None:
        col = self.column_offset
        self.command(0xB0 + page, col & 0x0F, 0x10 | (col >> 4))
        self.data(payload)


CONTROLLERS = {"sh1106": SH1106, "ssd1306": SSD1306}


class SMBusWriter:
    """smbus2 as an `I2CWriter`: one raw I2C message per write, so a 128-byte page is one transaction
    (SMBus block writes stop at 32 bytes)."""

    def __init__(self, bus_number: int) -> None:
        from smbus2 import SMBus, i2c_msg  # imported here so the protocol and its tests need no smbus2

        self._bus = SMBus(bus_number)
        self._msg = i2c_msg

    def write(self, address: int, data: Sequence[int]) -> None:
        self._bus.i2c_rdwr(self._msg.write(address, list(data)))

    def close(self) -> None:
        self._bus.close()
