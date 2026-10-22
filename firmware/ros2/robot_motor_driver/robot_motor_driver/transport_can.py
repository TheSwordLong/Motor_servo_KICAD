"""CAN transport: frame codec plus a deliberately honest refusal path.

Status: the codec below is unit tested against the frame layout implemented by
the firmware and by tools/robot_host.py, but **CAN has not been exercised on
hardware** in this round. The remote host does not even have python-can
installed (verified: `ModuleNotFoundError: No module named 'can'`), so
`CanTransport.open()` raises TransportNotAvailable with an actionable message
instead of pretending to work.

Frame layout (BSP/robot_protocol.c:44-54, tools/robot_host.py:108-159):
  * 500 kbit/s, standard 11 bit frames only, DLC always 8;
  * request id 0x600 + node, response id 0x580 + node;
  * response byte 0 is the request type OR 0x80, byte 1 echoes the rolling tag
    for reads/writes, and byte 2 carries the RAW business result 0..6 - there is
    no Modbus exception mapping on CAN;
  * a command argument is int16 on CAN (int32 on Modbus), which still covers the
    +/-3600 deci-degree MOVE range.
"""

import struct
from typing import List, Optional, Sequence, Tuple

from . import protocol as proto
from .transport_modbus import TransportError, TransportNotAvailable

CAN_BITRATE = 500000
CAN_DLC = 8
CAN_REQUEST_ID_BASE = 0x600
CAN_RESPONSE_ID_BASE = 0x580
CAN_NODE_MIN = proto.NODE_MIN
CAN_NODE_MAX = proto.NODE_MAX

TYPE_READ = 1
TYPE_WRITE = 2
TYPE_COMMAND = 3
RESPONSE_FLAG = 0x80

#: CAN carries the command argument as int16.
CAN_ARGUMENT_MIN = -32768
CAN_ARGUMENT_MAX = 32767


def request_id(node):
    # type: (int) -> int
    return CAN_REQUEST_ID_BASE + proto.validate_node(node)


def response_id(node):
    # type: (int) -> int
    return CAN_RESPONSE_ID_BASE + proto.validate_node(node)


def encode_read(tag, address, count):
    # type: (int, int, int) -> bytes
    """One CAN read request. The firmware answers at most 2 registers per frame."""
    if not 1 <= count <= 2:
        raise ValueError("CAN reads at most 2 registers per frame, got %r" % (count,))
    return struct.pack(">BBHB3x", TYPE_READ, tag & 0xFF, address & 0xFFFF, count)


def encode_write(tag, address, value):
    # type: (int, int, int) -> bytes
    """One CAN write request: exactly one register per frame."""
    return struct.pack(">BBHBHB", TYPE_WRITE, tag & 0xFF, address & 0xFFFF, 0,
                       int(value) & 0xFFFF, 0)


def encode_command(sequence, opcode, channel, argument):
    # type: (int, int, int, int) -> bytes
    """One CAN command frame. Sequence must be non-zero, exactly like Modbus."""
    values = proto.command_values(sequence, opcode, channel, argument)
    if not 1 <= values[0] <= proto.SEQUENCE_MAX:
        raise ValueError("sequence must be 1..65535, got %r" % (sequence,))
    arg = int(argument)
    if not CAN_ARGUMENT_MIN <= arg <= CAN_ARGUMENT_MAX:
        raise ValueError("CAN command argument must fit int16 (%d..%d), got %d"
                         % (CAN_ARGUMENT_MIN, CAN_ARGUMENT_MAX, arg))
    return struct.pack(">BHBBhB", TYPE_COMMAND, values[0], values[1], channel & 0xFF, arg, 0)


def decode_response(payload, request, tag=None):
    # type: (bytes, bytes, Optional[int]) -> Tuple[int, bytes]
    """Validate one CAN response and return (business result, payload).

    Raises FrameError when the frame does not belong to this request. The
    business result is the raw firmware code, NOT a Modbus exception code.
    """
    if len(payload) != CAN_DLC:
        raise proto.FrameError("CAN DLC must be 8, got %d" % len(payload))
    if payload[0] != (request[0] | RESPONSE_FLAG):
        raise proto.FrameError("CAN response type 0x%02X does not match request 0x%02X"
                               % (payload[0], request[0]))
    if request[0] in (TYPE_READ, TYPE_WRITE) and tag is not None and payload[1] != (tag & 0xFF):
        raise proto.FrameError("CAN tag mismatch: %d != %d" % (payload[1], tag & 0xFF))
    if request[0] == TYPE_COMMAND and payload[3] != request[2]:
        # A command response echoes the low byte of the sequence in byte 3, which is
        # the only way to tell two command replies apart on a shared bus.
        raise proto.FrameError("CAN command response echoes seq low 0x%02X, expected 0x%02X"
                               % (payload[3], request[2]))
    result = int(payload[2])
    if not 0 <= result <= int(proto.BusinessResult.ADDRESS):
        raise proto.FrameError("CAN result %d outside 0..6" % result)
    return result, payload


def decode_read_registers(payload, count):
    # type: (bytes, int) -> List[int]
    """Registers from a CAN read response; byte 3 is the register count."""
    if payload[3] != count:
        raise proto.FrameError("CAN read returned %d registers, expected %d" % (payload[3], count))
    return list(struct.unpack(">2H", payload[4:8]))[:count]


class CanTransport(object):
    """CAN master. Refuses to open until python-can is present AND the caller
    explicitly accepts that this path has no hardware evidence yet."""

    def __init__(self, interface, channel, node=1, bitrate=CAN_BITRATE,
                 response_timeout=proto.DEFAULT_RESPONSE_TIMEOUT,
                 require_hardware_evidence=True, can_module=None):
        # type: (...) -> None
        self.interface = interface
        self.channel = channel
        self.node = proto.validate_node(node)
        self.bitrate = int(bitrate)
        self.response_timeout = float(response_timeout)
        self.require_hardware_evidence = bool(require_hardware_evidence)
        self._can = can_module
        self._bus = None
        self._tag = 0
        self.transactions = 0

    def open(self):
        # type: () -> bool
        if self.require_hardware_evidence:
            raise TransportNotAvailable(
                "CAN has not been validated on hardware in this round and python-can "
                "is not installed on the target host. Refusing to open a CAN transport "
                "that would report success without evidence. Install python-can, then "
                "pass require_hardware_evidence=False once a bench test exists.")
        if self._can is None:
            try:
                import can as can_module
            except ImportError as error:
                raise TransportNotAvailable(
                    "python-can is not installed (%s); the Modbus RTU transport is the "
                    "supported path on this host" % error)
            self._can = can_module
        self._bus = self._can.Bus(interface=self.interface, channel=self.channel,
                                  bitrate=self.bitrate)
        return True

    def close(self):
        # type: () -> None
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    @property
    def is_open(self):
        # type: () -> bool
        return self._bus is not None

    def _next_tag(self):
        # type: () -> int
        self._tag = (self._tag + 1) & 0xFF
        return self._tag

    def _exchange(self, payload):
        # type: (bytes) -> bytes
        if self._bus is None:
            raise TransportError("CAN transport is not open")
        self.transactions += 1
        self._bus.send(self._can.Message(arbitration_id=request_id(self.node),
                                         is_extended_id=False, data=payload), timeout=0.5)
        import time

        deadline = time.monotonic() + self.response_timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise proto.ModbusTimeout("CAN response missing; retry the SAME command "
                                          "sequence, never an automatic new movement")
            message = self._bus.recv(remaining)
            if message is None:
                raise proto.ModbusTimeout("CAN receive returned None")
            if (message.arbitration_id != response_id(self.node) or message.is_extended_id
                    or getattr(message, "is_remote_frame", False)
                    or getattr(message, "is_error_frame", False)):
                continue
            return bytes(message.data)

    def read_registers(self, address, count, retries=None):
        # type: (int, int, Optional[int]) -> List[int]
        """Chunked read: at most 2 registers per CAN frame."""
        result = []    # type: List[int]
        offset = 0
        while offset < count:
            chunk = min(2, count - offset)
            tag = self._next_tag()
            request = encode_read(tag, address + offset, chunk)
            payload = decode_response(self._exchange(request), request, tag)[1]
            result.extend(decode_read_registers(payload, chunk))
            offset += chunk
        return result

    def write_registers(self, address, values, retries=None):
        # type: (int, Sequence[int], Optional[int]) -> None
        for offset, value in enumerate(values):
            tag = self._next_tag()
            request = encode_write(tag, address + offset, value)
            self._exchange(request)

    def write_command(self, sequence, opcode, channel, argument, retries=None):
        # type: (int, int, int, int, Optional[int]) -> None
        request = encode_command(sequence, opcode, channel, argument)
        payload = self._exchange(request)
        result, _ = decode_response(payload, request)
        if result:
            # CAN reports the raw business result, so there is no ambiguity to resolve.
            raise proto.ModbusExceptionResponse(
                proto.RESULT_TO_EXCEPTION.get(result, 0x04),
                "CAN business result %d" % result)

    def statistics(self):
        # type: () -> dict
        return {"interface": self.interface, "channel": self.channel, "node": self.node,
                "bitrate": self.bitrate, "open": self.is_open, "transactions": self.transactions,
                "hardware_evidence": not self.require_hardware_evidence}
