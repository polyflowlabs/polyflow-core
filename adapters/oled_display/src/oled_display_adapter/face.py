"""
The face: two rounded eyes on a 128x64 monochrome screen, as a pure function of what to show.

    render(expression, gaze_x, gaze_y, openness) -> 64 rows of 128 booleans

This is the ONE definition of what the face looks like. Studio's simulator draws the same face from a
TypeScript port (`libs/common/src/util/faceBitmap.ts`), and a fixture generated from this file pins the
two together pixel for pixel, so what you see in the sim is what the robot will show. Keep the arithmetic
to plain float operations in a fixed order (no rounding, no library calls), which come out bit-identical
in Python and JavaScript, and regenerate the fixture after any change (`python3 -m oled_display_adapter.face`).
"""

from typing import List

WIDTH = 128
HEIGHT = 64

EXPRESSIONS = ("neutral", "happy", "sad", "angry", "surprised", "sleepy", "off")

# One eye, in pixels. The two sit either side of the centre line.
EYE_W = 28.0
EYE_H = 32.0
EYE_R = 9.0
EYE_SPACING = 48.0
# How far a full gaze (±1) moves the eyes.
GAZE_X = 12.0
GAZE_Y = 8.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _in_rounded_rect(px: float, py: float, cx: float, cy: float, w: float, h: float, r: float) -> bool:
    ax = abs(px - cx) - (w / 2.0 - r)
    ay = abs(py - cy) - (h / 2.0 - r)
    if ax <= 0.0 and ay <= 0.0:
        return True
    dx = ax if ax > 0.0 else 0.0
    dy = ay if ay > 0.0 else 0.0
    return dx * dx + dy * dy <= r * r


def render(expression: str = "neutral", gaze_x: float = 0.0, gaze_y: float = 0.0, openness: float = 1.0) -> List[List[bool]]:
    """The screen, row by row (row 0 at the top). Unknown expressions draw as neutral."""
    rows = [[False] * WIDTH for _ in range(HEIGHT)]
    if expression == "off":
        return rows

    w, h = EYE_W, EYE_H
    if expression == "surprised":
        w, h = EYE_W * 1.12, EYE_H * 1.15
    o = _clamp(openness, 0.0, 1.0)
    h = 2.0 + (h - 2.0) * o
    r = EYE_R
    if r > h / 2.0:
        r = h / 2.0
    if r > w / 2.0:
        r = w / 2.0

    gx = _clamp(gaze_x, -1.0, 1.0) * GAZE_X
    gy = _clamp(gaze_y, -1.0, 1.0) * GAZE_Y
    cy = HEIGHT / 2.0 + gy

    for side in (-1.0, 1.0):
        cx = WIDTH / 2.0 + side * EYE_SPACING / 2.0 + gx
        top = cy - h / 2.0
        x0 = int(cx - w / 2.0) - 1
        x1 = int(cx + w / 2.0) + 2
        y0 = int(top) - 1
        y1 = int(cy + h / 2.0) + 2
        for y in range(max(0, y0), min(HEIGHT, y1)):
            py = y + 0.5
            for x in range(max(0, x0), min(WIDTH, x1)):
                px = x + 0.5
                if not _in_rounded_rect(px, py, cx, cy, w, h, r):
                    continue
                if expression == "happy":
                    # An arch: a circle below the eye takes its lower half away.
                    ccy = cy + h * 0.38
                    rr = w * 0.66
                    if (px - cx) * (px - cx) + (py - ccy) * (py - ccy) <= rr * rr:
                        continue
                elif expression in ("sad", "angry"):
                    # A slanted lid: lower at the outer corner (sad) or the inner one (angry).
                    t = ((px - cx) * side + w / 2.0) / w
                    if expression == "angry":
                        t = 1.0 - t
                    if py < top + h * 0.5 * t:
                        continue
                elif expression == "sleepy":
                    if py < top + h * 0.5:
                        continue
                rows[y][x] = True
    return rows


def to_hex(rows: List[List[bool]]) -> str:
    """Row-major, MSB first, 8 pixels a byte: the fixture format Studio's test reads."""
    out = []
    for row in rows:
        for i in range(0, WIDTH, 8):
            b = 0
            for bit in range(8):
                if row[i + bit]:
                    b |= 0x80 >> bit
            out.append(f"{b:02x}")
    return "".join(out)


# The cases the TypeScript port is checked against.
FIXTURE_CASES = [
    {"expression": e, "gaze_x": gx, "gaze_y": gy, "openness": o}
    for e in EXPRESSIONS
    for (gx, gy, o) in ((0.0, 0.0, 1.0), (0.7, -0.4, 1.0), (-1.0, 0.6, 0.35), (0.25, 0.0, 0.0))
]


if __name__ == "__main__":  # regenerate the fixture: python3 -m oled_display_adapter.face > face-bitmaps.json
    import json

    print(json.dumps([{**c, "hex": to_hex(render(**c))} for c in FIXTURE_CASES]))
