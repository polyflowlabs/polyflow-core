#!/usr/bin/env python3
"""
Probe which motor sub-command the attached Hiwonder board actually honours.

Background: the adapter currently drives motors with MOTOR sub-cmd 0x04
(raw PWM, int16) and stops with sub-cmd 0x03 (bitmask). Neither exists in
Hiwonder's stock `ros_robot_controller_sdk.py` — the stock firmware only
implements:

    0x01  speed  motor_num:u8, [motor_id0:u8, speed:f32 LE] x n
    0x05  duty   motor_num:u8, [motor_id0:u8, duty:f32  LE] x n

(motor_id0 is 0-indexed on the wire: channel 1 -> 0.)

An unrecognised sub-cmd is dropped silently by the firmware, so a board
that speaks only 0x01/0x05 looks exactly like "commands sent, nothing
moves". This script sends each candidate in turn so you can see which one
actually spins the wheel.

Usage (on the robot, with the normal consumer stopped):
    sudo systemctl stop polyflow-system-manager
    python3 scripts/probe_motor_protocol.py --port /dev/ttyACM0 --motor 1

    # if you're unsure of the link speed, check it first:
    python3 scripts/probe_motor_protocol.py --port /dev/ttyACM0 --baud-scan

    # re-test just one candidate:
    python3 scripts/probe_motor_protocol.py --motor 1 --only duty
"""

import argparse
import collections
import importlib.util
import struct
import time
from pathlib import Path

# Load rrc.py standalone, skipping the SDK-dependent package __init__
# (same trick as test_stop.py / test_motor_feedback.py).
_RRC_PATH = Path(__file__).resolve().parents[1] / "src" / "hiwonder_rrc_adapter" / "rrc.py"
_spec = importlib.util.spec_from_file_location("rrc", _RRC_PATH)
rrc_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rrc_mod)
HiwonderRRC = rrc_mod.HiwonderRRC
Func = rrc_mod.Func


# --- Frame sniffer -----------------------------------------------------------

def attach_sniffer(rrc):
    """Wrap _dispatch so we can count every valid frame, including the
    SYS sub-commands rrc.py drops silently."""
    seen = collections.Counter()
    original = rrc._dispatch

    def spy(frame):
        func = frame[0]
        data = bytes(frame[2:])
        if func == int(Func.SYS) and data:
            seen[f"SYS.sub_{data[0]:#04x}"] += 1
        else:
            try:
                seen[Func(func).name] += 1
            except ValueError:
                seen[f"func_{func}"] += 1
        return original(frame)

    rrc._dispatch = spy
    return seen


def listen(rrc, seen, seconds, label):
    print(f"\n[{label}] listening {seconds:.0f}s for inbound frames ...")
    before = collections.Counter(seen)
    time.sleep(seconds)
    delta = collections.Counter(seen)
    delta.subtract(before)
    delta = {k: v for k, v in delta.items() if v > 0}
    if not delta:
        print("  (nothing received)")
    else:
        for name, count in sorted(delta.items()):
            print(f"  {name:<18} x{count}")
    return delta


# --- Candidate motor commands ------------------------------------------------
# Each sends one command frame for `motor` (1-indexed) and returns a label.
# Stock-firmware forms first, current-adapter form last.

def send_speed(rrc, motor, value):
    """MOTOR sub-cmd 0x01 — closed-loop speed, one f32 per motor."""
    data = bytearray([0x01, 1])
    data.extend(struct.pack("<Bf", motor - 1, float(value)))
    rrc._send(Func.MOTOR, list(data))


def send_duty(rrc, motor, value):
    """MOTOR sub-cmd 0x05 — open-loop duty, one f32 per motor."""
    data = bytearray([0x05, 1])
    data.extend(struct.pack("<Bf", motor - 1, float(value)))
    rrc._send(Func.MOTOR, list(data))


def send_raw_pwm(rrc, motor, value):
    """MOTOR sub-cmd 0x04 — raw PWM int16. What the adapter sends today."""
    rrc._send(Func.MOTOR, list(struct.pack("<BBh", 0x04, motor - 1, int(value))))


# The duty/speed scale factor differs between firmware builds, so sweep a
# few magnitudes per form rather than betting on one.
CANDIDATES = {
    "speed": ("MOTOR 0x01 speed (f32)", send_speed, [0.5, 2.0, 20.0, 100.0]),
    "duty": ("MOTOR 0x05 duty (f32)", send_duty, [0.3, 30.0, 100.0, 1000.0]),
    "rawpwm": ("MOTOR 0x04 raw PWM (i16) — current adapter path", send_raw_pwm, [400]),
}

# Stops, tried in order at the end of every trial.
def stop_all(rrc, motor):
    send_speed(rrc, motor, 0.0)
    send_duty(rrc, motor, 0.0)
    rrc._send(Func.MOTOR, [0x03, 0x0F])  # bitmask stop (current adapter path)


# --- Main --------------------------------------------------------------------

def baud_scan(port, seconds):
    """Open the port at each plausible speed and see which yields valid frames.

    A native-USB CDC board ignores the host baud setting entirely, so
    expect *both* to work if it enumerates as /dev/ttyACM*. On a real UART
    (/dev/ttyAMA0, /dev/ttyUSB0) only the correct one will decode.
    """
    print("=== Baud scan ===")
    for baud in (1_000_000, 115_200):
        rrc = HiwonderRRC(port=port, baudrate=baud)
        try:
            rrc.open()
        except Exception as exc:
            print(f"  {baud:>9} baud: open failed — {exc}")
            continue
        seen = attach_sniffer(rrc)
        time.sleep(seconds)
        total = sum(seen.values())
        detail = ", ".join(f"{k} x{v}" for k, v in sorted(seen.items())) or "nothing"
        print(f"  {baud:>9} baud: {total:>4} valid frames  ({detail})")
        rrc.close()
    print("\nUse the baud that decodes frames. If both do, it's native USB CDC")
    print("and the baud setting is irrelevant — the problem is elsewhere.")


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=1_000_000,
                   help="stock ros_robot_controller SDK uses 1000000 (default)")
    p.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4],
                   help="motor channel to spin (1-4)")
    p.add_argument("--seconds", type=float, default=2.0,
                   help="how long to hold each command before stopping")
    p.add_argument("--only", choices=sorted(CANDIDATES),
                   help="run just one candidate form")
    p.add_argument("--baud-scan", action="store_true",
                   help="only probe which baud rate decodes frames, then exit")
    args = p.parse_args()

    if args.baud_scan:
        baud_scan(args.port, 3.0)
        return

    rrc = HiwonderRRC(port=args.port, baudrate=args.baud)
    print(f"Opening {args.port} at {args.baud} baud ...")
    rrc.open()
    seen = attach_sniffer(rrc)

    try:
        # Phase 0 — is the link alive at all? If nothing arrives, the wrong
        # port/baud is the real problem and the motor results below are noise.
        inbound = listen(rrc, seen, 3.0, "LINK CHECK")
        if not inbound:
            print("\n!! No valid frames received. Before trusting anything below,")
            print("   re-run with --baud-scan and check nothing else holds the port")
            print("   (sudo systemctl stop polyflow-system-manager).")
        else:
            print("  -> link is good: the board is talking and CRC checks pass.")

        names = [args.only] if args.only else list(CANDIDATES)
        print(f"\nSpinning motor {args.motor}. Watch the wheel for each step.")
        print("Note down which ones move it.\n")

        for name in names:
            label, send, values = CANDIDATES[name]
            print(f"--- {name}: {label} ---")
            for value in values:
                print(f"  sending {name}={value} ... hold {args.seconds:.0f}s", flush=True)
                send(rrc, args.motor, value)
                time.sleep(args.seconds)
                stop_all(rrc, args.motor)
                time.sleep(0.7)
            print()

        print("========================================")
        print("If 'speed' and/or 'duty' moved the motor but 'rawpwm' did not,")
        print("the board runs stock Hiwonder firmware and the adapter needs to")
        print("switch off sub-cmd 0x04/0x03. Report which values moved it and")
        print("how fast — that sets the full-scale constant.")
        print("========================================")
    finally:
        print("\nStopping and closing port.")
        try:
            stop_all(rrc, args.motor)
        except Exception:
            pass
        rrc.close()


if __name__ == "__main__":
    main()
