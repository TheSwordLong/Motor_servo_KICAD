#!/usr/bin/env python3
"""
RS-485 link probe for the GD32F303VET6 board.

Opens the USB-RS485 adapter, sends a known line, and reports whether the board
replied. In RS485_TEST_MODE_RX the board never transmits, so "no reply" is
expected there -- the point of this script is to give the board something to
receive, then you check g_rs485_rx_total in the debugger.

Usage:
    python tools/probe_rs485.py --port COM6
    python tools/probe_rs485.py --port COM6 --text "HELLO RS485" --repeat 5
    python tools/probe_rs485.py --port COM6 --listen-only
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.stderr.write("pyserial is required: pip install pyserial\n")
    raise SystemExit(2)


def main():
    ap = argparse.ArgumentParser(description="RS-485 link probe")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM6")
    ap.add_argument("--baud", type=int, default=9600, help="baud rate (default 9600)")
    ap.add_argument("--text", default="HELLO RS485", help="line to send")
    ap.add_argument("--repeat", type=int, default=1, help="how many times to send")
    ap.add_argument("--gap", type=float, default=0.5, help="seconds between sends")
    ap.add_argument("--listen-only", action="store_true",
                    help="do not send, only listen for board output")
    args = ap.parse_args()

    try:
        port = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        sys.stderr.write(f"cannot open {args.port}: {exc}\n")
        sys.stderr.write("Is another program (serial terminal) holding the port?\n")
        return 2

    with port:
        time.sleep(0.2)
        port.reset_input_buffer()

        if args.listen_only:
            print(f"listening on {args.port} @ {args.baud} for 5 s ...")
            end = time.time() + 5.0
            total = bytearray()
            while time.time() < end:
                chunk = port.read(256)
                if chunk:
                    total += chunk
            print(f"received {len(total)} bytes: {bytes(total)!r}")
            return 0

        for index in range(args.repeat):
            payload = f"{args.text}\r\n".encode("ascii")
            port.write(payload)
            port.flush()
            print(f"[{index + 1}/{args.repeat}] sent: {payload!r}")

            time.sleep(0.4)
            reply = port.read(256)
            if reply:
                print(f"           reply: {reply!r}")
            else:
                print(f"           reply: <none>")

            if index + 1 < args.repeat:
                time.sleep(args.gap)

    print()
    print("Notes:")
    print("  * In RS485_TEST_MODE_RX the board never transmits, so <none> is normal.")
    print("    Check g_rs485_rx_total / g_rs485_line in the debugger instead.")
    print("  * If a reply equals what you sent, the adapter has local echo enabled")
    print("    and the result is not a real board response.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
