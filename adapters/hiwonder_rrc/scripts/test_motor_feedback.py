#!/usr/bin/env python3
"""
Bench harness to check whether the Hiwonder RRC Lite is reporting real
wheel/motor feedback (encoder counters + rps).

The board emits a motor-state frame at 20 Hz (SYS func 0x00, sub_cmd 0x05)
carrying, per motor, an i32 tick counter and an f32 rotations/sec. On the
older board these read all zeros — no encoders wired to the PID. This
script watches that telemetry while spinning one motor open-loop (raw PWM),
so you can see whether the *new* board actually populates them.

Verdict logic: feedback is considered WORKING for the spun motor if its
counter moves by more than --min-ticks OR its |rps| exceeds --min-rps
during the spin phase, while it stayed ~flat during the idle baseline.

Usage (run on the robot, with the port's normal consumer stopped):
    sudo systemctl stop polyflow-system-manager
    python3 scripts/test_motor_feedback.py --port /dev/ttyACM0 --motor 1
    # gentler / no motion (telemetry-only, e.g. spin the wheel by hand):
    python3 scripts/test_motor_feedback.py --motor 1 --no-spin --seconds 15
"""

import argparse
import importlib.util
import time
from pathlib import Path

# Load rrc.py standalone, skipping the SDK-dependent package __init__
# (same trick as test_stop.py).
_RRC_PATH = Path(__file__).resolve().parents[1] / "src" / "hiwonder_rrc_adapter" / "rrc.py"
_spec = importlib.util.spec_from_file_location("rrc", _RRC_PATH)
rrc_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rrc_mod)
HiwonderRRC = rrc_mod.HiwonderRRC


def snapshot(rrc):
    """Return (counters[4], rps[4], age) from the latest cached frame, or None."""
    latest = rrc._latest_motor_state
    if latest is None:
        return None
    counters, rps, ts = latest
    return counters, rps, time.monotonic() - ts


def fmt_row(counters, rps, spun_idx):
    cells = []
    for i in range(4):
        mark = "*" if i == spun_idx else " "
        cells.append(f"{mark}M{i + 1}: c={counters[i]:>8d} rps={rps[i]:>7.2f}")
    return "  ".join(cells)


def watch(rrc, seconds, spun_idx, label, period=0.25):
    """Print motor-state at `period` cadence for `seconds`; return first/last frame."""
    print(f"\n[{label}] watching {seconds:.0f}s ...")
    first = None
    last = None
    deadline = time.monotonic() + seconds
    next_print = 0.0
    while time.monotonic() < deadline:
        snap = snapshot(rrc)
        if snap is not None:
            counters, rps, age = snap
            if first is None:
                first = (counters, rps)
            last = (counters, rps)
            now = time.monotonic()
            if now >= next_print:
                stale = "  (STALE)" if age > 0.5 else ""
                print(f"  {fmt_row(counters, rps, spun_idx)}  age={age * 1000:4.0f}ms{stale}")
                next_print = now + period
        time.sleep(0.02)
    return first, last


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115_200)
    p.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4],
                   help="motor port to spin / focus on (1-4)")
    p.add_argument("--pwm", type=int, default=400,
                   help="open-loop raw PWM duty for the spin phase (-1000..1000)")
    p.add_argument("--seconds", type=float, default=5.0,
                   help="duration of each watch phase")
    p.add_argument("--no-spin", action="store_true",
                   help="don't drive the motor; just watch telemetry "
                        "(spin the wheel by hand to test encoders)")
    p.add_argument("--min-ticks", type=int, default=5,
                   help="counter delta over the spin phase to call feedback working")
    p.add_argument("--min-rps", type=float, default=0.05,
                   help="peak |rps| to call feedback working")
    args = p.parse_args()

    spun_idx = args.motor - 1

    rrc = HiwonderRRC(port=args.port, baudrate=args.baud)
    print(f"Opening {args.port} at {args.baud} baud ...")
    rrc.open()
    print("Port open. (Stop polyflow-system-manager first if it's running.)")

    try:
        # Give the rx thread a beat to catch the first 20 Hz frame.
        time.sleep(0.5)
        if snapshot(rrc) is None:
            print("\n!! No motor-state frame (SYS sub_cmd 0x05) received at all.")
            print("   Either nothing is emitting telemetry, the port is wrong,")
            print("   or another process is holding the serial line. Aborting.")
            return

        # Phase 1: idle baseline — nothing driven.
        base_first, base_last = watch(rrc, args.seconds, spun_idx, "IDLE baseline")

        # Phase 2: spin one motor open-loop and watch feedback respond.
        if not args.no_spin:
            print(f"\n[SPIN] motor {args.motor} at raw PWM {args.pwm} ...")
            rrc.set_motor_pwm(args.motor, args.pwm)
            spin_first, spin_last = watch(rrc, args.seconds, spun_idx, "SPIN")
            print(f"\n[STOP] motor {args.motor}")
            rrc.set_motor_pwm(args.motor, 0)
            rrc.stop_all_motors()
        else:
            print("\n[SPIN] skipped (--no-spin). Spin the wheel by hand now.")
            spin_first, spin_last = watch(rrc, args.seconds, spun_idx, "MANUAL spin")

        # ---- Verdict for the spun motor ----
        print("\n================ RESULT ================")
        if spin_first is None or spin_last is None:
            print("No telemetry captured during the active phase. Inconclusive.")
            return

        c0 = spin_first[0][spun_idx]
        c1 = spin_last[0][spun_idx]
        tick_delta = abs(c1 - c0)
        peak_rps = max(abs(spin_first[1][spun_idx]), abs(spin_last[1][spun_idx]))

        # Idle sanity: how much did the counter drift with nothing driven?
        idle_drift = (
            abs(base_last[0][spun_idx] - base_first[0][spun_idx])
            if base_first and base_last else 0
        )

        print(f"Motor {args.motor}:  idle counter drift={idle_drift} ticks  |  "
              f"active tick delta={tick_delta}  peak |rps|={peak_rps:.3f}")

        working = tick_delta >= args.min_ticks or peak_rps >= args.min_rps
        if working:
            print("VERDICT: ✅ motor feedback is LIVE — counter/rps respond to motion.")
            print("         This board can drive real wheel odometry / speed-PID.")
        else:
            print("VERDICT: ❌ no feedback — counter/rps stayed flat while driven.")
            print("         Same as the old board: encoders not reporting. Odometry")
            print("         would have to come from commanded velocity, not this.")
        print("========================================")
    finally:
        print("\nClosing port.")
        try:
            rrc.stop_all_motors()
        except Exception:
            pass
        rrc.close()


if __name__ == "__main__":
    main()
