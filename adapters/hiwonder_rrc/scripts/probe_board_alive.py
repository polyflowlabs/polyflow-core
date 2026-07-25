#!/usr/bin/env python3
"""
Triage: is the host->board command path working at all, and is the motor
power rail live?

`probe_motor_protocol.py` came back with no motion on any sub-command form,
including the two the stock Hiwonder firmware implements (0x01 speed,
0x05 duty). That rules out "wrong motor opcode" as the whole story and
leaves three very different faults, which this script separates:

  A. Nothing is getting to the board (wrong port/baud, or the TX line is
     dead while RX happens to work).
  B. Commands land fine but the motor supply rail is unpowered — the STM32
     runs off USB VBUS, so the board enumerates, talks, and reports happily
     while the H-bridges have nothing to drive with.
  C. Commands land, rail is powered, and the fault is genuinely in the
     motor command path.

The discriminator is the buzzer/LED/RGB. Those use non-motor function
codes (2, 1, 11) whose wire format in rrc.py already matches the vendor
SDK byte-for-byte, and they don't depend on the motor rail. So:

  buzzer beeps  -> host->board TX works; fault is B or C
  buzzer silent -> fault is A; stop debugging motors and fix the link

Battery voltage (SYS sub-cmd 0x04) then splits B from C: a board with a
dead motor rail typically reports ~0 V or nothing at all, while a healthy
one reports pack voltage in the 7-14 V range this board expects.

Usage (on the robot):
    sudo systemctl stop polyflow-system-manager
    PP=$(ls -d /nix/store/*polyflow-system-manager-env/lib/python3*/site-packages | head -1)
    sudo env PYTHONPATH="$PP" python3 scripts/probe_board_alive.py --port /dev/ttyACM0
"""

import argparse
import collections
import importlib.util
import time
from pathlib import Path

_RRC_PATH = Path(__file__).resolve().parents[1] / "src" / "hiwonder_rrc_adapter" / "rrc.py"
_spec = importlib.util.spec_from_file_location("rrc", _RRC_PATH)
rrc_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rrc_mod)
HiwonderRRC = rrc_mod.HiwonderRRC
Func = rrc_mod.Func


def attach_sniffer(rrc):
    """Count every CRC-valid frame, including SYS sub-commands rrc.py drops."""
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


def ask(prompt):
    try:
        return input(f"{prompt} [y/N] ").strip().lower().startswith("y")
    except EOFError:
        print("(no tty — assuming no)")
        return False


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=1_000_000)
    p.add_argument("--listen", type=float, default=4.0,
                   help="seconds to listen during the RX census")
    p.add_argument("--no-prompt", action="store_true",
                   help="skip the interactive did-you-hear-it questions")
    args = p.parse_args()

    rrc = HiwonderRRC(port=args.port, baudrate=args.baud)
    print(f"Opening {args.port} at {args.baud} baud ...\n")
    rrc.open()
    seen = attach_sniffer(rrc)

    results = {}

    try:
        # ---- 1. RX census -------------------------------------------------
        print("=" * 60)
        print(f"1. RX CENSUS — listening {args.listen:.0f}s")
        print("=" * 60)
        time.sleep(args.listen)
        if not seen:
            print("  NOTHING RECEIVED.")
            print("  The board is not talking at this port/baud. Everything")
            print("  below will be meaningless — fix this first (--baud-scan")
            print("  in probe_motor_protocol.py).")
        else:
            for name, count in sorted(seen.items()):
                print(f"  {name:<18} x{count}")
        results["rx"] = dict(seen)

        # ---- 2. Battery / motor rail --------------------------------------
        print()
        print("=" * 60)
        print("2. BATTERY VOLTAGE — is the motor rail powered?")
        print("=" * 60)
        voltage = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            mv = rrc.get_battery()
            if mv is not None:
                voltage = mv / 1000.0
                break
            time.sleep(0.05)

        if voltage is None:
            print("  No battery frame (SYS sub-cmd 0x04) seen.")
            print("  -> Can't confirm the rail either way from telemetry.")
        else:
            print(f"  Reported pack voltage: {voltage:.2f} V")
            if voltage < 6.0:
                print("  -> BELOW the board's 7-14V input range. The motor rail")
                print("     is almost certainly unpowered: the STM32 is running")
                print("     off USB VBUS. Check the DC barrel input and the")
                print("     board's power switch. Motors cannot move like this,")
                print("     no matter which sub-command you send.")
            else:
                print("  -> Rail looks powered. Motor supply is not the problem.")
        results["voltage"] = voltage

        # ---- 3. TX proof via non-motor peripherals ------------------------
        print()
        print("=" * 60)
        print("3. TX PROOF — buzzer / LED / RGB (no motor rail involved)")
        print("=" * 60)
        print("  These use rrc.py wire formats that already match the vendor")
        print("  SDK exactly. If they work, host->board commands are fine.\n")

        print("  Beeping buzzer (2400 Hz, 3 x 200ms) ...", flush=True)
        rrc.set_buzzer(2400, 200, 200, 3)
        time.sleep(2.0)

        print("  Blinking onboard LED 1 (5 x 200ms) ...", flush=True)
        rrc.set_led(1, 200, 200, 5)
        time.sleep(2.5)

        print("  Setting RGB LEDs 1 and 2 to blue ...", flush=True)
        rrc.set_rgb([(1, 0, 0, 255), (2, 0, 0, 255)])
        time.sleep(1.5)
        print("  Setting RGB LEDs 1 and 2 to red ...", flush=True)
        rrc.set_rgb([(1, 255, 0, 0), (2, 255, 0, 0)])
        time.sleep(1.5)
        rrc.set_rgb([(1, 0, 0, 0), (2, 0, 0, 0)])

        if not args.no_prompt:
            results["buzzer"] = ask("\n  Did the buzzer beep?")
            results["led"] = ask("  Did the onboard LED blink?")
            results["rgb"] = ask("  Did the RGB LEDs change colour?")

        # ---- Verdict ------------------------------------------------------
        print()
        print("=" * 60)
        print("VERDICT")
        print("=" * 60)

        rx_ok = bool(seen)
        tx_ok = any(results.get(k) for k in ("buzzer", "led", "rgb"))

        if not rx_ok and not tx_ok:
            print("  Link is dead in both directions. Wrong port or baud, or")
            print("  something else holds the device. Run --baud-scan and")
            print("  confirm the Type-C cable is on UART1, not a power-only port.")
        elif rx_ok and not tx_ok and not args.no_prompt:
            print("  RX works, TX does not: the board talks to you but ignores")
            print("  everything you send. That is a one-directional link fault")
            print("  (cable, or a bridge chip whose TX baud disagrees), NOT a")
            print("  motor protocol problem. Motor opcodes are irrelevant until")
            print("  this is fixed.")
        elif tx_ok and results.get("voltage") is not None and results["voltage"] < 6.0:
            print("  Commands reach the board, but the motor rail is unpowered.")
            print("  Power the board from its 7-14V DC input and re-run")
            print("  probe_motor_protocol.py.")
        elif tx_ok:
            print("  Commands reach the board and the rail looks powered, yet no")
            print("  motor sub-command moves anything. Next suspects: motor")
            print("  wiring/connectors on the new board, a per-channel enable,")
            print("  or firmware that differs from stock. Report this output.")
        else:
            print("  Inconclusive — re-run without --no-prompt so the buzzer/LED")
            print("  answers can be used.")
        print("=" * 60)

    finally:
        print("\nClosing port.")
        try:
            rrc._send(Func.MOTOR, [0x03, 0x0F])
        except Exception:
            pass
        rrc.close()


if __name__ == "__main__":
    main()
