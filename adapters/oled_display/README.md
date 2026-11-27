# polyflow-oled-display-adapter

Polyflow hardware adapter for 128x64 monochrome I²C OLED screens, drawing a robot's **face**: two rounded
eyes that blink, look around and change expression.

| Driver | Screen | Default address |
|---|---|---|
| `sh1106_display` | SH1106, e.g. Waveshare 1.3" OLED (A) | `0x3C` |
| `ssd1306_display` | SSD1306, e.g. Adafruit 0.96" (STEMMA QT) | `0x3D` |

Each is a **device** driver: it runs the screen itself (no host board), and its hardware row is the
screen's own placement on the robot.

## What it shows

It consumes `DisplayCommand` (`display_id`, `expression`, `gaze_x`, `gaze_y`), which the **Face** node
sends, and animates on its own between commands. The gaze eases toward the commanded one, and the eyes
blink every few seconds (not when sleepy or off).

Expressions: `neutral`, `happy`, `sad`, `angry`, `surprised`, `sleepy`, `off`.

`face.py` is the one definition of the face. Studio's simulator draws the same pixels from a TypeScript port
that's tested against a fixture this file generates:

```
PYTHONPATH=adapters/polyflow_adapter_sdk/src:adapters/oled_display/src python3 -m oled_display_adapter.face > face-bitmaps.json
```

Regenerate it (and copy it to Studio's `libs/common/src/util/__fixtures__/face-bitmaps.json`) after any
change to the face.

## Wiring

On the Pi's header: VCC → 3.3 V (pin 1), GND → GND, SDA → GPIO2 (pin 3), SCL → GPIO3 (pin 5). On the
Waveshare board, set its BS resistors for I²C. Polyflow OS enables I²C bus 1.

## hardware.json example

```json
{ "drivers": [{ "id": "oled_1", "name": "sh1106_display", "params": { "i2c_address": 60 } }] }
```

## Tests

```
PYTHONPATH=adapters/polyflow_adapter_sdk/src:adapters/oled_display/src python3 -m unittest discover -s adapters/oled_display/tests -t adapters/oled_display
```

The tests drive both controllers against fake chips that decode their I²C traffic into a screen, per the
SH1106 and SSD1306 datasheets.
