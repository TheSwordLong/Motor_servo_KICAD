"""Robot board client. No serial/CAN connection is opened on import.

Dependencies: pyserial for Modbus, python-can for CAN; install only the selected
transport. CAN adapter is supplied by python-can's interface/channel options.
"""
from __future__ import annotations
import argparse
import json
import struct
import time
from pathlib import Path

ROLES = {"unassigned": 0, "head": 1, "left_upper": 2, "right_upper": 3,
         "left_forearm": 4, "right_forearm": 5, "left_palm": 6, "right_palm": 7}
COMMANDS = {"config": 1, "zero": 2, "move": 3, "stop": 4, "stop-all": 5,
            "brake": 6, "direction": 7, "save": 8, "clear-fault": 9,
            "selftest": 10, "selftest-all": 11}
SELFTEST_STATES = ("untested", "queued", "running", "settling", "passed",
                   "failed", "cancelled", "skipped")
SELFTEST_REASONS = ("none", "no-pulses", "too-few-pulses", "capture-error",
                    "other-channel-activity", "stop-timeout", "cancelled",
                    "storage-error", "position-overflow", "return-timeout", "return-error")
SELFTEST_PHASES = ("not-started", "outbound", "outbound-settling", "return", "return-settling", "finished")


def selftest_result(values: list[int], detail: list[int] | None = None) -> dict:
    state, reason, ph, pl, powered, th, tl, sequence = values
    result = {"state": state, "state_name": SELFTEST_STATES[state] if state < len(SELFTEST_STATES) else "unknown",
            "reason": reason, "reason_name": SELFTEST_REASONS[reason] if reason < len(SELFTEST_REASONS) else "unknown",
            "pulses": (ph << 16) | pl, "powered_ms": powered,
            "completed_uptime_ms": (th << 16) | tl, "sequence": sequence}
    if detail is not None:
        phase, return_ms, oh, ol, rh, rl, eh, el = detail
        result.update(phase=phase, phase_name=SELFTEST_PHASES[phase] if phase < len(SELFTEST_PHASES) else "unknown",
                      return_ms=return_ms, outbound_pulses=(oh << 16) | ol,
                      return_pulses=(rh << 16) | rl, residual_pulses=signed32([eh, el]))
    return result


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


class Modbus:
    def __init__(self, port: str, address: int):
        import serial
        self.address = address
        self.serial = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.5)

    def exchange(self, pdu: bytes) -> bytes:
        body = bytes([self.address]) + pdu
        frame = body + struct.pack("<H", crc16(body))
        # Retries use identical complete command bytes and the same nonzero seq.
        for _ in range(3):
            time.sleep(0.004)
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            response = bytearray()
            deadline = time.monotonic() + 1.5  # Flash rollover can delay reply.
            while time.monotonic() < deadline:
                response += self.serial.read(256 - len(response))
                if len(response) < 3:
                    continue
                size = 5 if response[1] & 0x80 else (5 + response[2] if response[1] == 3 else 8)
                if len(response) >= size:
                    break
            if len(response) < 5 or crc16(response) or response[0] != self.address:
                continue
            if response[1] == (pdu[0] | 0x80):
                raise RuntimeError(f"Modbus exception {response[2]} (2=address,3=value/seq,4=state/storage,6=busy)")
            if response[1] != pdu[0]:
                continue
            if pdu[0] in (6, 16) and response[2:6] != pdu[1:5]:
                continue
            return bytes(response[1:-2])
        raise TimeoutError("No valid response; operation may have completed. Do not create a new movement request blindly.")

    def read(self, address: int, count: int) -> list[int]:
        data = self.exchange(struct.pack(">BHH", 3, address, count))
        if data[1] != count * 2:
            raise RuntimeError("Unexpected register count")
        return list(struct.unpack(f">{count}H", data[2:]))

    def write(self, address: int, values: list[int]):
        if len(values) == 1:
            self.exchange(struct.pack(">BHH", 6, address, values[0]))
        else:
            data = struct.pack(f">{len(values)}H", *values)
            self.exchange(struct.pack(">BHHB", 16, address, len(values), len(data)) + data)

    def command(self, seq: int, op: int, ch: int, arg: int):
        if not 1 <= seq <= 65535:
            raise ValueError("Retrying client requires a nonzero sequence (1..65535)")
        if self.read(0x0F, 1) != [2]:
            raise RuntimeError("Board does not support direct commands (revision 2); update firmware/client together")
        arg &= 0xFFFFFFFF
        self.write(0x30, [seq, op, ch, arg >> 16, arg & 0xFFFF])

    def close(self):
        self.serial.close()


class Can:
    def __init__(self, interface: str, channel: str, node: int):
        import can
        self.can = can
        self.bus = can.Bus(interface=interface, channel=channel, bitrate=500000)
        self.node = node
        self.tag = 0

    def exchange(self, data: bytes) -> bytes:
        for _ in range(3):
            self.bus.send(self.can.Message(arbitration_id=0x600 + self.node,
                                          is_extended_id=False, data=data), timeout=0.5)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                msg = self.bus.recv(max(0, deadline - time.monotonic()))
                if msg is None:
                    break
                p = bytes(msg.data)
                if (msg.arbitration_id != 0x580 + self.node or msg.is_extended_id or
                        msg.is_remote_frame or msg.is_error_frame or len(p) != 8 or
                        p[0] != (data[0] | 0x80) or p[1] != data[1]):
                    continue
                if data[0] == 3 and p[3] != data[2]:
                    continue
                if p[2]:
                    raise RuntimeError(f"Board result {p[2]} (1=value,2=busy,3=unready,4=storage,5=sequence,6=address)")
                return p
        raise TimeoutError("CAN response missing; retry the SAME command sequence, never an automatic new movement.")

    def read(self, address: int, count: int) -> list[int]:
        result = []
        for offset in range(0, count, 2):
            n = min(2, count - offset)
            self.tag = (self.tag + 1) & 255
            p = self.exchange(struct.pack(">BBHB3x", 1, self.tag, address + offset, n))
            if p[3] != n:
                raise RuntimeError("Unexpected CAN read count")
            result.extend(struct.unpack(">2H", p[4:])[:n])
        return result

    def write(self, address: int, values: list[int]):
        for offset, value in enumerate(values):
            self.tag = (self.tag + 1) & 255
            self.exchange(struct.pack(">BBHBHB", 2, self.tag, address + offset, 0, value, 0))

    def command(self, seq: int, op: int, ch: int, arg: int):
        if not 1 <= seq <= 65535:
            raise ValueError("Retrying client requires a nonzero sequence (1..65535)")
        self.exchange(struct.pack(">BHBBhB", 3, seq, op, ch, arg, 0))

    def close(self):
        self.bus.shutdown()


def identity(client) -> str:
    return "".join(f"{v:04X}" for v in client.read(8, 6))


def signed32(values: list[int]) -> int:
    return struct.unpack(">i", struct.pack(">HH", *values))[0]


def validate_inventory(config: dict):
    """Validate whole robot before any register writes; boards keyed by UID."""
    joints, nodes = set(), set()
    boards = config.get("boards", {})
    if not boards:
        raise ValueError("boards must map full 24-hex UID strings to configuration")
    for uid, board in boards.items():
        if len(uid) != 24 or uid != uid.upper() or any(c not in "0123456789ABCDEF" for c in uid):
            raise ValueError(f"Invalid full UID: {uid}")
        if board.get("protocol", 0) not in (0, 1):
            raise ValueError("protocol: 0=Modbus, 1=CAN")
        if not 1 <= board.get("address", 1) <= 247 or not 1 <= board.get("node", 1) <= 127:
            raise ValueError("Invalid address/node")
        if not 20000 <= board.get("pwm_hz", 25000) <= 30000:
            raise ValueError("pwm_hz must be 20000..30000")
        if board.get("protocol", 0) == 1:
            node = board.get("node", 1)
            if node in nodes:
                raise ValueError(f"Duplicate CAN node {node}")
            nodes.add(node)
        channels = board.get("channels", [])
        if len(channels) != 4:
            raise ValueError("Exactly four fixed channels required per board")
        for channel in channels:
            role = channel.get("role", "unassigned")
            if role not in ROLES or (role != "unassigned" and role in joints):
                raise ValueError(f"Unknown or duplicate robot joint: {role}")
            if role != "unassigned":
                joints.add(role)
            if channel.get("gear", 0) not in (0, 82, 290, 516):
                raise ValueError("gear must be 82, 290 or 516 (0 only for unconfigured channels)")
            if channel.get("enabled", False) and (not channel.get("gear") or role == "unassigned"):
                raise ValueError("Enabled channel needs gear and joint")
            if not 400 <= channel.get("rpm", 8000) <= 8000:
                raise ValueError("Pre-gear cruise rpm must be 400..8000")
            if channel.get("inverted", 0) not in (0, 1):
                raise ValueError("inverted must be 0 or 1")


def configure(client, inventory: dict, uid: str, sequence: int):
    validate_inventory(inventory)
    if uid not in inventory["boards"]:
        raise ValueError("Connected UID is not bound in inventory; replacement boards require explicit rebind")
    board = inventory["boards"][uid]
    client.write(0x20, [board.get("protocol", 0), board.get("address", 1),
                        board.get("node", 1), board.get("pwm_hz", 25000)])
    for i, ch in enumerate(board["channels"]):
        client.write(0x108 + i * 0x40, [int(ch.get("enabled", False)), ROLES[ch.get("role", "unassigned")],
                                       ch.get("gear", 0), ch.get("inverted", 0), ch.get("rpm", 8000)])
    client.command(sequence, COMMANDS["config"], 0, 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=["modbus", "can"], default="modbus")
    parser.add_argument("--port", help="CH340 COM port")
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--interface", default="slcan")
    parser.add_argument("--can-channel", default="COM5")
    parser.add_argument("--node", type=int, default=1)
    parser.add_argument("--uid", help="Required full board UID for every write/movement")
    parser.add_argument("--seq", type=int, help="Nonzero sequence (1..65535) for replaying an uncertain operation")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("status")
    cfg = sub.add_parser("config"); cfg.add_argument("inventory", type=Path)
    for name in ("zero", "move", "stop", "brake", "direction", "clear-fault"):
        p = sub.add_parser(name); p.add_argument("motor", type=int, choices=range(1, 5))
        if name == "move":
            p.add_argument("angle", type=float, help="relative output-shaft degrees, -360..360")
        elif name in ("brake", "direction"):
            p.add_argument("value", type=int, choices=(0, 1))
    sub.add_parser("stop-all"); sub.add_parser("save")
    probe = sub.add_parser("selftest", help="Brief response probe; does not prove physical connection")
    probe.add_argument("motor", choices=("1", "2", "3", "4", "all"))
    probe.add_argument("--direction", choices=("positive", "negative"), default="positive")
    args = parser.parse_args()
    if args.transport == "modbus" and not args.port:
        parser.error("--port is required for Modbus")
    if args.action != "status" and not args.uid:
        parser.error("--uid is required; first use status to identify the connected board")
    client = Modbus(args.port, args.address) if args.transport == "modbus" else Can(args.interface, args.can_channel, args.node)
    try:
        uid = identity(client)
        if args.uid and uid != args.uid.upper():
            raise ValueError(f"UID mismatch: connected {uid}")
        sequence = args.seq if args.seq is not None else (((client.read(0x36, 1)[0] + 1) & 0xFFFF) or 1)
        if not 1 <= sequence <= 65535:
            raise ValueError("Sequence must be 1..65535; zero is reserved for manual, non-retrying commands")
        if args.action == "status":
            print(json.dumps({"uid": uid, "board": client.read(0, 8),
                              "last_sequence_result": client.read(0x36, 2)}, indent=2))
            for i in range(4):
                base = 0x100 + i * 0x40
                print(json.dumps({"motor": i + 1, "config": client.read(base, 5),
                                  "state_fault_valid_brake_dir_duty_rpm": client.read(base + 16, 7),
                                  "angle_deg": signed32(client.read(base + 24, 2)) / 10,
                                  "signed_pulses": signed32(client.read(base + 26, 2)),
                                  "last_selftest": selftest_result(client.read(base + 0x20, 8),
                                                                  client.read(base + 0x28, 8))}, indent=2))
        else:
            # Print the request identity before sending: retain this for retries.
            print(f"UID={uid} seq={sequence} action={args.action}", flush=True)
            if args.action == "config":
                configure(client, json.loads(args.inventory.read_text(encoding="utf-8")), uid, sequence)
            elif args.action == "selftest":
                batch = args.motor == "all"
                client.command(sequence, COMMANDS["selftest-all" if batch else "selftest"],
                               0 if batch else int(args.motor) - 1, int(args.direction == "negative"))
            else:
                arg = round(args.angle * 10) if args.action == "move" else getattr(args, "value", 0)
                if args.action == "move" and not -3600 <= arg <= 3600:
                    raise ValueError("Angle must be -360..360 degrees")
                client.command(sequence, COMMANDS[args.action], getattr(args, "motor", 1) - 1, arg)
            print("Accepted. Motion completion and saved status must be polled separately.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
