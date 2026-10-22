#!/usr/bin/env python3
"""
RS-485 host test tool for the GD32F303VET6 control board.

Requires Python 3.7+ and pyserial:  pip install pyserial

Board firmware: BSP/board_config.h selects the test mode.
  RS485_TEST_MODE_A = 1 : the board emits "RS485_TEST,SEQ=000001\\r\\n" every second
  RS485_TEST_MODE_B = 1 : the board answers PING / ECHO / STATUS (default)

Examples
--------
Mode A, just listen:
    python test_rs485.py --port COM7 --mode listen

Mode B, 1000 PING round trips at 9600:
    python test_rs485.py --port COM7 --baud 9600 --mode ping --count 1000

Mode B, one ECHO:
    python test_rs485.py --port COM7 --mode echo --text "hello world"

Mode B, read the board counters:
    python test_rs485.py --port COM7 --mode status

IMPORTANT
    Use a USB-RS485 adapter wired to CN4 (pin 1 = B, pin 2 = A). Never connect a
    plain USB-TTL adapter directly to A/B. Make sure the adapter's local echo is
    disabled, otherwise it will be mistaken for a board reply.
"""

import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.stderr.write("pyserial is required: pip install pyserial\n")
    raise SystemExit(2)


def open_port(args):
    return serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=args.timeout,
        write_timeout=args.timeout,
    )


def read_line(port, timeout):
    """Read one CRLF/LF terminated line. Returns bytes without the terminator."""
    deadline = time.monotonic() + timeout
    buf = bytearray()

    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        if chunk == b"\n":
            return bytes(buf).rstrip(b"\r")
        buf += chunk

    return None


def round_trip(port, request, timeout):
    """Send one request and wait for one reply line. Returns (reply, elapsed_ms)."""
    port.reset_input_buffer()
    start = time.monotonic()
    port.write(request)
    port.flush()
    reply = read_line(port, timeout)
    elapsed_ms = (time.monotonic() - start) * 1000.0
    return reply, elapsed_ms


def mode_listen(port, args):
    if args.touch:
        # Board firmware mode A2_TRIGGER stays silent until the host sends the
        # first byte; anything works, the content is discarded by the board.
        port.write(b"\x00")
        port.flush()
        print(f"Sent the 1-byte trigger ({args.touch!r}); waiting for the stream...")
    else:
        print("Listening. The board should emit RS485_TEST,SEQ=... every second.")
        print("(If the board runs mode A2_TRIGGER it stays silent until it gets one byte;")
        print(" pass --touch to send that trigger.)")
    print("Press Ctrl+C to stop.\n")
    count = 0
    last_seq = None
    try:
        while True:
            line = read_line(port, 3.0)
            if line is None:
                print("  <timeout: no frame within 3 s>")
                continue
            count += 1
            text = line.decode("ascii", errors="replace")

            ok = text.startswith("RS485_TEST,SEQ=")
            seq_ok = True
            if ok:
                try:
                    seq = int(text.split("=", 1)[1])
                    if last_seq is not None and seq != last_seq + 1:
                        seq_ok = False
                    last_seq = seq
                except ValueError:
                    seq_ok = False

            flag = "OK" if (ok and seq_ok) else "BAD"
            print(f"  [{flag}] {text}")
    except KeyboardInterrupt:
        print(f"\nReceived {count} frames.")


def mode_ping(port, args):
    ok = 0
    timeout = 0
    mismatch = 0
    times = []

    print(f"PING x{args.count} at {args.baud} baud")
    for index in range(1, args.count + 1):
        reply, ms = round_trip(port, b"PING\r\n", args.timeout)

        if reply is None:
            timeout += 1
            status = "TIMEOUT"
        elif reply != b"PONG":
            mismatch += 1
            status = f"MISMATCH({reply!r})"
        else:
            ok += 1
            times.append(ms)
            status = f"OK {ms:.1f} ms"

        if index % 50 == 0 or status != "OK" and not status.startswith("OK "):
            print(f"  [{index}/{args.count}] {status}")

    print("\n--- PING summary ---")
    print(f"  success   : {ok}")
    print(f"  timeout   : {timeout}")
    print(f"  mismatch  : {mismatch}")
    if times:
        print(f"  avg RTT   : {statistics.mean(times):.1f} ms")
        print(f"  max RTT   : {max(times):.1f} ms")
    return 0 if (ok == args.count) else 1


def mode_echo(port, args):
    payload = args.text
    request = f"ECHO {payload}\r\n".encode("ascii")
    reply, ms = round_trip(port, request, args.timeout)
    expected = payload.encode("ascii")

    print(f"  sent    : {request!r}")
    print(f"  received: {reply!r}  ({ms:.1f} ms)")

    if reply is None:
        print("  result  : TIMEOUT")
        return 1
    if reply != expected:
        print(f"  result  : MISMATCH (expected {expected!r})")
        return 1

    print("  result  : OK")
    return 0


def mode_status(port, args):
    reply, ms = round_trip(port, b"STATUS\r\n", args.timeout)

    if reply is None:
        print("STATUS: TIMEOUT")
        return 1

    text = reply.decode("ascii", errors="replace")
    print(f"  raw ({ms:.1f} ms): {text}")

    if not text.startswith("STATUS,"):
        print("  result: UNEXPECTED FORMAT")
        return 1

    fields = {}
    for item in text.split(",")[1:]:
        if "=" in item:
            key, value = item.split("=", 1)
            fields[key] = value

    for key in ("BAUD", "CLOCK_HZ", "RX_BYTES", "TX_BYTES",
                "RX_OVERFLOW", "UART_ERRORS", "TX_TIMEOUTS"):
        print(f"    {key:<12} = {fields.get(key, '<missing>')}")

    print("  result: OK")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="RS-485 host test tool for the GD32F303VET6 board",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", required=True, help="serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=9600,
                        help="baud rate (default 9600; the isolator is a 150 kbps part)")
    parser.add_argument("--mode", default="ping",
                        choices=["listen", "ping", "echo", "status"],
                        help="test mode (default ping)")
    parser.add_argument("--count", type=int, default=1000,
                        help="number of PING round trips (default 1000)")
    parser.add_argument("--text", default="hello", help="payload for --mode echo")
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="reply timeout in seconds (default 1.0)")
    parser.add_argument("--touch", action="store_true",
                        help="--mode listen only: send one trigger byte first "
                             "(firmware mode A2_TRIGGER starts streaming only "
                             "after the host's first byte)")
    args = parser.parse_args()

    try:
        port = open_port(args)
    except serial.SerialException as exc:
        sys.stderr.write(f"Cannot open {args.port}: {exc}\n")
        return 2

    with port:
        time.sleep(0.1)
        if args.mode == "listen":
            return mode_listen(port, args)
        if args.mode == "ping":
            return mode_ping(port, args)
        if args.mode == "echo":
            return mode_echo(port, args)
        return mode_status(port, args)


if __name__ == "__main__":
    raise SystemExit(main())
