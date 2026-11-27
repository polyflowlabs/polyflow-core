"""The face renderer, both OLED controllers against fake chips that decode their I2C traffic into a screen,
and the display adapter's animation."""

import random
import unittest

from polyflow_adapter_sdk import LEVEL_ERROR, LIFECYCLE_DISABLE

from oled_display_adapter import face
from oled_display_adapter.display_adapter import BLINK_S, SH1106DisplayAdapter, SSD1306DisplayAdapter
from oled_display_adapter.oled import SH1106, SSD1306, to_pages


class FakeChip:
    """An OLED on the bus: decodes command and data writes into its RAM, the way each controller addresses
    it, so a test can read back exactly what would be on screen."""

    def __init__(self, kind, address):
        self.kind, self.address = kind, address
        self.ram = [bytearray(132) for _ in range(8)]
        self.page = self.col = 0
        self.window = (0, 127, 0, 7)  # SSD1306 horizontal mode
        self.on = False
        self.commands = []
        self.writes = 0

    def write(self, address, data):
        if address != self.address:
            raise OSError(121, "Remote I/O error")
        self.writes += 1
        control, payload = data[0], list(data[1:])
        if control == 0x00:
            self._commands(payload)
        elif control == 0x40:
            for b in payload:
                self.ram[self.page][self.col] = b
                self._advance()

    def _commands(self, cmds):
        self.commands += cmds
        i = 0
        while i < len(cmds):
            c = cmds[i]
            if c == 0xAF:
                self.on = True
            elif c == 0xAE:
                self.on = False
            if self.kind == "ssd1306" and c == 0x21:
                self.window = (cmds[i + 1], cmds[i + 2], self.window[2], self.window[3])
                self.col = cmds[i + 1]
                i += 2
            elif self.kind == "ssd1306" and c == 0x22:
                self.window = (self.window[0], self.window[1], cmds[i + 1], cmds[i + 2])
                self.page = cmds[i + 1]
                i += 2
            elif self.kind == "sh1106" and 0xB0 <= c <= 0xB7:
                self.page = c - 0xB0
            elif self.kind == "sh1106" and c <= 0x0F:
                self.col = (self.col & 0xF0) | c
            elif self.kind == "sh1106" and 0x10 <= c <= 0x1F:
                self.col = (self.col & 0x0F) | ((c & 0x0F) << 4)
            elif c in (0xD5, 0xA8, 0xD3, 0x8D, 0x20, 0xDA, 0x81, 0xD9, 0xDB, 0xAD):
                i += 1  # one argument byte
            i += 1

    def _advance(self):
        if self.kind == "sh1106":
            self.col += 1  # stays within the page
            return
        c0, c1, p0, p1 = self.window
        self.col += 1
        if self.col > c1:
            self.col = c0
            self.page = p0 if self.page >= p1 else self.page + 1

    def screen(self):
        """What's visible, as rows of booleans."""
        offset = 2 if self.kind == "sh1106" else 0
        return [[bool(self.ram[y // 8][x + offset] >> (y % 8) & 1) for x in range(128)] for y in range(64)]


class FakeHost:
    def __init__(self):
        self.statuses, self.polls = [], []

    def publish_state(self, *a):
        pass

    def report_status(self, target_id, level, code, message, values):
        self.statuses.append((level, code, message))

    def schedule(self, rate_hz, fn):
        self.polls.append(fn)

        class H:
            def cancel(self):
                pass

        return H()

    def log(self, message):
        pass


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def display(cls, kind, address, params=None):
    chip = FakeChip(kind, address)
    host = FakeHost()
    clock = Clock()
    d = cls(host, "face", params or {})
    d.bus_factory = lambda n: chip
    d.clock = clock
    d.rng = random.Random(1)
    d.configure()
    d.start()
    return d, chip, host, clock


class FaceTest(unittest.TestCase):
    def test_two_eyes_either_side_of_centre(self):
        rows = face.render()
        self.assertTrue(rows[32][40] and rows[32][88])
        self.assertFalse(rows[32][64])  # nothing between them
        self.assertFalse(any(rows[0]) or any(rows[63]))

    def test_gaze_moves_both_eyes(self):
        right = face.render("neutral", 1.0, 0.0)
        self.assertTrue(right[32][52] and right[32][100])
        self.assertFalse(right[32][30])

    def test_expressions_differ_and_off_is_dark(self):
        pics = {e: face.to_hex(face.render(e)) for e in face.EXPRESSIONS}
        self.assertEqual(len(set(pics.values())), len(face.EXPRESSIONS))
        self.assertFalse(any(any(r) for r in face.render("off")))

    def test_happy_is_an_arch_and_sleepy_a_half_lid(self):
        happy = face.render("happy")
        self.assertTrue(happy[20][40])  # top of the eye stays
        self.assertFalse(happy[36][40])  # its middle is cut away
        sleepy = face.render("sleepy")
        self.assertFalse(sleepy[22][40])
        self.assertTrue(sleepy[40][40])

    def test_a_shut_eye_is_a_thin_line(self):
        shut = face.render("neutral", 0.0, 0.0, 0.0)
        lit_rows = [y for y in range(64) if shut[y][40]]
        self.assertLessEqual(len(lit_rows), 3)

    def test_unknown_expression_is_neutral(self):
        self.assertEqual(face.render("bogus"), face.render("neutral"))


class ControllerTest(unittest.TestCase):
    def check(self, cls, kind):
        chip = FakeChip(kind, 0x3C)
        oled = cls(chip, 0x3C)
        oled.init()
        self.assertTrue(chip.on)
        rows = face.render("happy", 0.4, -0.2)
        oled.show(to_pages(rows))
        self.assertEqual(chip.screen(), rows)

    def test_sh1106_puts_the_face_on_screen(self):
        self.check(SH1106, "sh1106")

    def test_ssd1306_puts_the_face_on_screen(self):
        self.check(SSD1306, "ssd1306")

    def test_only_changed_pages_are_resent(self):
        chip = FakeChip("sh1106", 0x3C)
        oled = SH1106(chip, 0x3C)
        oled.init()
        self.assertEqual(oled.show(to_pages(face.render())), 4)  # the eyes span pages 2–5
        self.assertEqual(oled.show(to_pages(face.render())), 0)

    def test_flip_turns_it_round(self):
        rows = face.render("sad", 0.8, 0.3)
        chip = FakeChip("ssd1306", 0x3C)
        oled = SSD1306(chip, 0x3C)
        oled.init()
        oled.show(to_pages(rows, flip=True))
        self.assertEqual(chip.screen(), [list(reversed(r)) for r in reversed(rows)])


class AdapterTest(unittest.TestCase):
    def test_a_command_changes_the_face(self):
        d, chip, host, clock = display(SH1106DisplayAdapter, "sh1106", 0x3C)
        self.assertEqual(host.statuses[-1][1], "ready")
        d.on_command("DisplayCommand", {"display_id": "face", "expression": "surprised", "gaze_x": 0.0, "gaze_y": 0.0})
        d.frame()
        self.assertEqual(chip.screen(), face.render("surprised"))

    def test_gaze_eases_toward_the_command(self):
        d, chip, _, _ = display(SH1106DisplayAdapter, "sh1106", 0x3C)
        d.on_command("DisplayCommand", {"expression": "neutral", "gaze_x": 1.0, "gaze_y": 0.0})
        d.frame()
        self.assertGreater(d.gaze[0], 0.2)
        self.assertLess(d.gaze[0], 0.5)
        for _ in range(30):
            d.frame()
        self.assertAlmostEqual(d.gaze[0], 1.0, places=3)

    def test_it_blinks_on_its_own_and_not_when_sleepy(self):
        d, chip, _, clock = display(SSD1306DisplayAdapter, "ssd1306", 0x3D)
        clock.t = d._blink_at + BLINK_S / 2.0  # mid-blink
        d.frame()
        self.assertEqual(chip.screen(), face.render("neutral", 0.0, 0.0, 0.0))
        clock.t += BLINK_S  # blink over, next one scheduled
        self.assertEqual(d.openness(clock.t), 1.0)
        self.assertGreater(d._blink_at, clock.t)
        d.on_command("DisplayCommand", {"expression": "sleepy"})
        clock.t = d._blink_at + BLINK_S / 2.0
        self.assertEqual(d.openness(clock.t), 1.0)

    def test_an_unknown_expression_shows_neutral(self):
        d, chip, _, _ = display(SH1106DisplayAdapter, "sh1106", 0x3C)
        d.on_command("DisplayCommand", {"expression": "smug"})
        self.assertEqual(d.expression, "neutral")

    def test_wrong_address_is_reported(self):
        chip = FakeChip("sh1106", 0x3D)
        host = FakeHost()
        d = SH1106DisplayAdapter(host, "face", {})
        d.bus_factory = lambda n: chip
        d.configure()
        d.start()
        self.assertEqual(host.statuses[-1][:2], (LEVEL_ERROR, "display_not_found"))
        self.assertIn("0x3c", host.statuses[-1][2])
        self.assertEqual(d.frame(), 0)

    def test_disable_turns_the_screen_off(self):
        d, chip, _, _ = display(SH1106DisplayAdapter, "sh1106", 0x3C)
        d.on_command("HardwareLifecycle", {"request": LIFECYCLE_DISABLE})
        self.assertFalse(chip.on)

    def test_address_as_hex_string(self):
        d, chip, host, _ = display(SSD1306DisplayAdapter, "ssd1306", 0x3C, {"i2c_address": "0x3C"})
        self.assertEqual(host.statuses[-1][1], "ready")


if __name__ == "__main__":
    unittest.main()
