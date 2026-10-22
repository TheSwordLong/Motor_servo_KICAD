"""GD32F303 robot motor board protocol: framing, registers, enums, unit math.

Pure Python 3.8, no ROS and no serial import, so it can be unit tested anywhere.
Every constant here was extracted from firmware source; the evidence table lives
in docs/protocol_contract.md. Nothing in this module opens a device.

Firmware facts that shape this module:
  * registers are 16 bit big endian on the wire, CRC is little endian;
  * a 32 bit quantity occupies two registers with the HIGH word at the lower
    address (BSP/robot.c:508, 533);
  * MOVE arguments are int32 in 0.1 deg on the OUTPUT shaft, limited to
    +/-3600, and the client must not multiply by the gear ratio again;
  * the command block 0x0030..0x0034 only accepts one atomic FC16 write with
    address 0x0030 and count 5 (BSP/robot.c:573-577).
"""

import math
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------
# Errors
# --------------------------------------------------------------------------


class ProtocolError(Exception):
    """Base class for every protocol level failure."""


class ModbusExceptionResponse(ProtocolError):
    """The board answered with a Modbus exception frame.

    The exception code is COARSER than the firmware business result: both
    R_VALUE and R_SEQUENCE map to 0x03, and both R_UNREADY and R_STORAGE map to
    0x04 (BSP/robot_protocol.c:11-12). Callers that need the exact business
    result must read registers 0x0036/0x0037.
    """

    def __init__(self, code, context=""):
        self.code = int(code)
        self.context = context
        name = ModbusException.name_of(self.code)
        hint = MODBUS_EXCEPTION_HINTS.get(self.code, "")
        super(ModbusExceptionResponse, self).__init__(
            "modbus exception 0x%02X (%s)%s%s"
            % (self.code, name, (" " + hint) if hint else "",
               (": " + context) if context else "")
        )


class ModbusTimeout(ProtocolError):
    """No valid reply inside the bounded transaction window.

    The board stays completely silent on a CRC or address mismatch
    (BSP/robot_protocol.c:16), and a gap longer than 750 us inside one frame
    invalidates the whole ADU (BSP/robot_rtu.c:9-15), so a timeout carries no
    information about whether the command was executed. Callers must reconcile
    through 0x0036/0x0037 instead of blindly resending with a new sequence.
    """


class FrameError(ProtocolError):
    """A reply failed structural validation (length, CRC, echo, address)."""


class ReadOnlyViolation(ProtocolError):
    """A write was attempted while the driver is in read-only mode."""


class IdentityError(ProtocolError):
    """UID or command protocol revision does not match the expected board."""


# --------------------------------------------------------------------------
# CRC and framing
# --------------------------------------------------------------------------


def crc16(data):
    # type: (bytes) -> int
    """Standard Modbus CRC-16 (reflected polynomial 0xA001, init 0xFFFF).

    Bit-for-bit identical to the firmware implementation (BSP/robot_protocol.c:3-8)
    and to tools/robot_host.py:40-46.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF


def append_crc(body):
    # type: (bytes) -> bytes
    """Return body plus the two CRC bytes, low byte first."""
    return body + struct.pack("<H", crc16(body))


def frame_crc_ok(frame):
    # type: (bytes) -> bool
    """True when the CRC over the WHOLE frame (including its own two bytes) is zero."""
    return len(frame) >= 4 and crc16(frame) == 0


def build_adu(address, pdu):
    # type: (int, bytes) -> bytes
    """Build one complete RTU application data unit: address + pdu + crc."""
    if not 0 <= address <= 247:
        raise ValueError("modbus address must be 0..247, got %r" % (address,))
    return append_crc(bytes([address]) + pdu)


def expected_response_length(first_bytes):
    # type: (Sequence[int]) -> Optional[int]
    """Total ADU length implied by the first bytes of a response, or None if unknown.

    Mirrors the length logic of tools/robot_host.py:69 and the firmware encoder:
    exception frames are 5 bytes, FC3 is 5 + byte count, FC6/FC16 echo 8 bytes.
    """
    if len(first_bytes) < 2:
        return None
    func = first_bytes[1]
    if func & 0x80:
        return 5
    if func == 3:
        if len(first_bytes) < 3:
            return None
        return 5 + first_bytes[2]
    if func in (6, 16):
        return 8
    return None


MAX_ADU_BYTES = 256  # longer ADUs are discarded whole by the firmware


# --------------------------------------------------------------------------
# Register map
# --------------------------------------------------------------------------

# Board level, read only (BSP/robot.c:497-516)
REG_SCHEMA = 0x0000          # flash image schema, constant 1
REG_PROTOCOL = 0x0001        # 0 = Modbus, 1 = CAN (active transport)
REG_ADDRESS = 0x0002         # active modbus slave address 1..247
REG_NODE = 0x0003            # active CAN node 1..127
REG_PWM_HZ = 0x0004          # active pwm frequency 20000..30000
REG_DIRTY = 0x0005
REG_STORAGE_ERROR = 0x0006
REG_PENDING_SAVE = 0x0007
REG_UID = 0x0008             # 6 registers = 3 x 32 bit words, high word first
REG_HAVE_SEQ = 0x000E
REG_COMMAND_PROTOCOL = 0x000F  # constant 2 for the single frame command protocol

# Board level, writable staging area (BSP/robot.c:511-512, 566, 581-584)
REG_STAGED_BOARD = 0x0020    # 4 registers: protocol, address, node, pwm_hz

# Command shadow block: one atomic FC16 write of exactly 5 registers
REG_COMMAND = 0x0030         # seq, op, ch, arg_hi, arg_lo
REG_COMMAND_COUNT = 5
REG_COMMAND_TOMBSTONE = 0x0035  # reads 0, writes are rejected (legacy A55A commit)
LEGACY_COMMIT_MAGIC = 0xA55A    # never sent by this driver

REG_LAST_SEQUENCE = 0x0036
REG_LAST_RESULT = 0x0037

# Channel block offsets relative to channel_base(ch)
OFF_CONFIG_APPLIED = 0x00    # 5 regs: enabled, role, gear, inverted, rpm (read only)
OFF_CONFIG_STAGED = 0x08     # 5 regs, same layout (writable)
OFF_STATUS = 0x10            # 7 regs: state, fault, valid, brake, direction, duty, rpm
OFF_POSITION = 0x18          # 6 regs: angle i32, fg count i32, target angle i32
OFF_SELFTEST = 0x20          # 8 regs: state, reason, pulses hi/lo, powered ms, done hi/lo, seq
OFF_SELFTEST_DETAIL = 0x28   # 8 regs: phase, return ms, out hi/lo, ret hi/lo, residual hi/lo

CHANNEL_COUNT = 4
CHANNEL_STRIDE = 0x40
CHANNEL_BASE_FIRST = 0x0100

REG_CAPABILITIES = 0x0010
CAP_MANUAL_ADJUST = 1
CAP_FAULT4_RECOVERY = 2
OFF_MANUAL_ADJUST = 0x30
FAULT4_RECOVERY_MS = 5000
COMMAND_PROTOCOL_REVISION = 2
FLASH_SCHEMA = 1


def channel_base(channel):
    # type: (int) -> int
    """Protocol channel base address. channel is 0..3 (NOT the 1..4 YAML numbering)."""
    if not 0 <= channel < CHANNEL_COUNT:
        raise ValueError("protocol channel must be 0..3, got %r" % (channel,))
    return CHANNEL_BASE_FIRST + CHANNEL_STRIDE * channel


def yaml_channel_to_protocol(number):
    # type: (int) -> int
    """Convert the 1..4 channel numbering used in YAML to the protocol 0..3 numbering."""
    if not 1 <= number <= CHANNEL_COUNT:
        raise ValueError("yaml channel must be 1..4, got %r" % (number,))
    return number - 1


# --------------------------------------------------------------------------
# Enumerations (values are the firmware's, BSP/robot.h:27-37)
# --------------------------------------------------------------------------


class Opcode(IntEnum):
    CONFIG = 1
    ZERO = 2
    MOVE = 3
    STOP = 4
    STOP_ALL = 5
    BRAKE = 6
    DIRECTION = 7
    SAVE = 8
    CLEAR_FAULT = 9
    SELFTEST = 10
    SELFTEST_ALL = 11
    MANUAL_ADJUST = 12


#: Opcodes whose argument is a 0.1 deg output shaft angle.
ANGLE_OPCODES = frozenset([int(Opcode.MOVE)])

#: Opcodes that act on the whole board and ignore the channel field.
BOARD_OPCODES = frozenset([int(Opcode.CONFIG), int(Opcode.STOP_ALL),
                           int(Opcode.SAVE), int(Opcode.SELFTEST_ALL)])

#: Opcodes that require the WHOLE board to be quiet for 500 ms first
#: (BSP/robot.c:167, 382, 387, 420, 424, 430).
QUIET_REQUIRED_OPCODES = frozenset([
    int(Opcode.CONFIG), int(Opcode.ZERO), int(Opcode.SAVE),
    int(Opcode.DIRECTION), int(Opcode.CLEAR_FAULT),
    int(Opcode.SELFTEST), int(Opcode.SELFTEST_ALL),
])


class BusinessResult(IntEnum):
    OK = 0
    VALUE = 1
    BUSY = 2
    UNREADY = 3
    STORAGE = 4
    SEQUENCE = 5
    ADDRESS = 6


class MotionState(IntEnum):
    UNREADY = 0
    IDLE = 1
    RUNNING = 2
    DECEL = 3
    SETTLING = 4
    DONE = 5
    FAULT = 6
    SELFTEST = 7
    MANUAL = 8


#: States in which the mechanism may still be moving.
BUSY_STATES = frozenset([int(MotionState.RUNNING), int(MotionState.DECEL),
                         int(MotionState.SETTLING), int(MotionState.SELFTEST), int(MotionState.MANUAL)])


class FaultCode(IntEnum):
    NONE = 0
    FEEDBACK = 1
    TIMEOUT = 2
    CAPTURE = 3
    UNEXPECTED_FG = 4
    POSITION = 5
    STORAGE = 6
    SELFTEST = 7


class SelftestState(IntEnum):
    UNTESTED = 0
    QUEUED = 1
    RUNNING = 2
    SETTLING = 3
    PASSED = 4
    FAILED = 5
    CANCELLED = 6
    SKIPPED = 7


#: Self test outcomes that mean the probe is over, whether it passed or not.
SELFTEST_FINAL_STATES = frozenset([int(SelftestState.PASSED), int(SelftestState.FAILED),
                                   int(SelftestState.CANCELLED), int(SelftestState.SKIPPED)])


class SelftestReason(IntEnum):
    NONE = 0
    NO_PULSES = 1
    TOO_FEW_PULSES = 2
    CAPTURE_ERROR = 3
    OTHER_ACTIVITY = 4
    STOP_TIMEOUT = 5
    CANCELLED = 6
    STORAGE_ERROR = 7
    POSITION_OVERFLOW = 8
    RETURN_TIMEOUT = 9
    RETURN_ERROR = 10  # reserved for legacy records; no firmware path produces it


class SelftestPhase(IntEnum):
    NONE = 0
    OUTBOUND = 1
    OUTBOUND_SETTLING = 2
    RETURN = 3
    RETURN_SETTLING = 4
    FINISHED = 5


class Role(IntEnum):
    UNASSIGNED = 0
    HEAD = 1
    LEFT_UPPER = 2
    RIGHT_UPPER = 3
    LEFT_FOREARM = 4
    RIGHT_FOREARM = 5
    LEFT_PALM = 6
    RIGHT_PALM = 7


class ModbusException(IntEnum):
    ILLEGAL_FUNCTION = 0x01
    ILLEGAL_DATA_ADDRESS = 0x02
    ILLEGAL_DATA_VALUE = 0x03
    SLAVE_DEVICE_FAILURE = 0x04
    SLAVE_DEVICE_BUSY = 0x06

    @classmethod
    def name_of(cls, code):
        try:
            return cls(code).name.lower()
        except ValueError:
            return "unknown"


#: What each exception code can actually mean on this board. Kept at module level
#: because a plain dict inside an IntEnum body would be treated as a member.
MODBUS_EXCEPTION_HINTS = {
    0x01: "function code not supported",
    0x02: "illegal data address: unmapped/read-only register, or a command "
          "block write that was not FC16 address 0x0030 count 5",
    0x03: "illegal data value OR sequence conflict: unknown opcode, argument "
          "out of range, channel >= 4, replayed sequence with different "
          "parameters, or a sequence outside the half-range window",
    0x04: "not ready OR storage error: interlock not satisfied (channel not "
          "enabled, gear 0, fault present, board stop fault mask) or flash "
          "storage failure",
    0x06: "busy: board or channel not quiet, motion in progress, probe "
          "running, or the board stop input is asserted",
}


#: Business results that the exception code alone cannot distinguish. Reading
#: 0x0036/0x0037 is the only way to tell them apart.
AMBIGUOUS_EXCEPTIONS = {
    int(ModbusException.ILLEGAL_DATA_VALUE): (BusinessResult.VALUE, BusinessResult.SEQUENCE),
    int(ModbusException.SLAVE_DEVICE_FAILURE): (BusinessResult.UNREADY, BusinessResult.STORAGE),
}

#: Reverse map used for the simulated board and for tests.
RESULT_TO_EXCEPTION = {
    int(BusinessResult.ADDRESS): int(ModbusException.ILLEGAL_DATA_ADDRESS),
    int(BusinessResult.VALUE): int(ModbusException.ILLEGAL_DATA_VALUE),
    int(BusinessResult.SEQUENCE): int(ModbusException.ILLEGAL_DATA_VALUE),
    int(BusinessResult.UNREADY): int(ModbusException.SLAVE_DEVICE_FAILURE),
    int(BusinessResult.STORAGE): int(ModbusException.SLAVE_DEVICE_FAILURE),
    int(BusinessResult.BUSY): int(ModbusException.SLAVE_DEVICE_BUSY),
}


def _enum_name(enum_class, value, fallback):
    # type: (object, object, str) -> str
    try:
        return enum_class(int(value)).name.lower()
    except (ValueError, TypeError):
        return fallback


def opcode_name(value):
    # type: (int) -> str
    return _enum_name(Opcode, value, "opcode%d" % int(value))


def state_name(value):
    # type: (int) -> str
    return _enum_name(MotionState, value, "state%d" % int(value))


def fault_name(value):
    # type: (int) -> str
    return _enum_name(FaultCode, value, "fault%d" % int(value))


def business_result_name(value):
    # type: (int) -> str
    return _enum_name(BusinessResult, value, "result%d" % int(value))


def selftest_state_name(value):
    # type: (int) -> str
    return _enum_name(SelftestState, value, "selftest%d" % int(value))


def selftest_reason_name(value):
    # type: (int) -> str
    return _enum_name(SelftestReason, value, "reason%d" % int(value))


def selftest_phase_name(value):
    # type: (int) -> str
    return _enum_name(SelftestPhase, value, "phase%d" % int(value))


def role_name(value):
    # type: (int) -> str
    return _enum_name(Role, value, "role%d" % int(value))


# --------------------------------------------------------------------------
# Firmware limits and timing constants
# --------------------------------------------------------------------------

GEAR_RATIOS = (0, 82, 290, 516)     # BSP/robot.c:18; 0 only for unconfigured channels
RPM_MIN = 400                       # BSP/robot.c:17, pre-gear cruise speed
RPM_MAX = 8000
PWM_HZ_DEFAULT = 25000              # BSP/robot.c:8
PWM_HZ_MIN = 20000                  # BSP/robot.c:14
PWM_HZ_MAX = 30000
ADDRESS_MIN = 1
ADDRESS_MAX = 247
NODE_MIN = 1
NODE_MAX = 127

FG_PULSES_PER_MOTOR_REV = 6         # single wire FG, rising edges per motor turn
MAX_MOVE_DECI_DEG = 3600            # +/-360.0 deg per MOVE, BSP/robot.c:441
DUTY_MAX_PERMILLE = 950             # LOW-level per mille, BSP/robot.h:20

CONTROL_MS = 10                     # BSP/robot.h:16
FEEDBACK_TIMEOUT_MS = 1000          # BSP/robot.h:15
QUIET_MS = 500                      # BSP/robot.h:12
TOLERANCE_DECI_DEG = 50             # BSP/robot.h:11, 5.0 deg output shaft
CRAWL_RPM = 400                     # BSP/robot.h:18
DECEL_DECI_DEG = 300                # BSP/robot.h:19, last 30.0 deg ramp to crawl
SELFTEST_DUTY_MAX = 100             # BSP/robot.h:22
SELFTEST_DUTY_STEP = 10             # BSP/robot.h:23, per control period
SELFTEST_POWER_MS = 2000            # BSP/robot.h:24, per direction
SELFTEST_TARGET_PULSES = 3          # BSP/robot.h:25
SELFTEST_STOP_TIMEOUT_MS = 2000     # BSP/robot.h:26
UNEXPECTED_FG_DDEG = 100  # Output-shaft cumulative activity: strictly >10 degrees.

def unexpected_fg_budget(gear):
    """Applied gear only. UINT32_MAX disables fault 4 for unconfigured gears."""
    return (UNEXPECTED_FG_DDEG * 6 * int(gear) // 3600
            if gear in (516, 290, 82) else 0xFFFFFFFF)


#: The default build compiles the hardware e-stop OUT: BUTTON_ESTOP_ENABLE is 0
#: (BSP/board_config.h:111), so the board's stop_input stays 0 and the interlock
#: in BSP/robot.c:367-368 never fires. Software STOP is therefore the only stop
#: path on a default-build board, and it is not a safety function.
ESTOP_COMPILED_IN = False

#: FaultCode.STORAGE (6) has no assignment anywhere in the firmware; storage
#: failures surface only through register 0x0006 and business result R_STORAGE.
#: A client must not wait for B+0x11 == 6.
FAULT_STORAGE_UNREACHABLE = True

#: CONFIG applies pwm_hz immediately (BSP/robot.c:406) but protocol, address and
#: node are only assigned in robot_init (BSP/robot.c:69-70), so changing them
#: needs a power cycle. Reading 0x0001..0x0003 after a CONFIG that changed them
#: therefore still returns the OLD active values while 0x0020..0x0022 hold the
#: new staged ones; that divergence is expected, not a failure.
CONFIG_APPLIES_IMMEDIATELY = ("pwm_hz",)
CONFIG_NEEDS_REBOOT = ("protocol", "address", "node")

#: RTU framing thresholds in microseconds (BSP/robot_rtu.c:9-15). A gap above
#: RX_GAP_US inside one frame discards the whole ADU silently, and a frame above
#: RX_END_US of silence is treated as complete.
TX_GAP_US = 750
RX_END_US = 1750

#: Client side transaction window. The firmware writes flash when it ACCEPTS a
#: MOVE or SELFTEST, so a reply can legitimately take hundreds of milliseconds;
#: tools/robot_host.py:64 uses 1.5 s for the same reason.
DEFAULT_RESPONSE_TIMEOUT = 1.5
DEFAULT_INTER_FRAME_DELAY = 0.004

#: Sequence handling. The board keeps a 16 entry ring cache (BSP/robot.h:91) and
#: rejects any sequence whose distance from the last one is >= 0x8000.
SEQUENCE_CACHE_ENTRIES = 16
SEQUENCE_WINDOW = 0x8000
SEQUENCE_MAX = 0xFFFF


def motion_timeout_ms(distance_deci_deg, gear, rpm):
    # type: (int, int, int) -> int
    """Firmware monitoring deadline for one MOVE, in milliseconds.

    `10000 + 4 * (distance * gear * 60000 / (3600 * rpm))` (BSP/robot.c:462-463).
    This is a watchdog, NOT a promised arrival time.
    """
    distance = abs(int(distance_deci_deg))
    if gear <= 0 or rpm <= 0:
        return 10000
    return 10000 + 4 * int(distance * gear * 60000 // (3600 * rpm))


def pulses_per_output_rev(gear):
    # type: (int) -> int
    """FG counts per output shaft revolution: gear * 6."""
    return int(gear) * FG_PULSES_PER_MOTOR_REV if gear else 0


# --------------------------------------------------------------------------
# Integer helpers (C semantics matter here)
# --------------------------------------------------------------------------


def trunc_div(numerator, denominator):
    # type: (int, int) -> int
    """Integer division truncating toward zero, like C99, not Python's floor."""
    if denominator == 0:
        raise ZeroDivisionError("trunc_div by zero")
    quotient = abs(numerator) // abs(denominator)
    return quotient if (numerator < 0) == (denominator < 0) else -quotient


def round_half_away_from_zero(value):
    # type: (float) -> int
    """Symmetric rounding: 0.5 -> 1, -0.5 -> -1, 2.5 -> 3, -2.5 -> -3.

    Python's built-in round() is banker's rounding, which would make +0.05 deg
    and -0.05 deg quantise asymmetrically, so it is deliberately not used.
    """
    if value != value or value in (float("inf"), float("-inf")):
        raise ValueError("cannot round NaN or infinity")
    magnitude = math.floor(abs(value) + 0.5)
    return int(magnitude) if value >= 0 else -int(magnitude)


def pack_u32(value):
    # type: (int) -> Tuple[int, int]
    """Split an unsigned 32 bit value into (high word, low word)."""
    value = int(value) & 0xFFFFFFFF
    return (value >> 16) & 0xFFFF, value & 0xFFFF


def unpack_u32(high, low):
    # type: (int, int) -> int
    return ((int(high) & 0xFFFF) << 16) | (int(low) & 0xFFFF)


def pack_i32(value):
    # type: (int) -> Tuple[int, int]
    """Split a signed 32 bit value into (high word, low word), two's complement."""
    return pack_u32(int(value))


def unpack_i32(high, low):
    # type: (int, int) -> int
    return struct.unpack(">i", struct.pack(">HH", int(high) & 0xFFFF, int(low) & 0xFFFF))[0]


# --------------------------------------------------------------------------
# Angle and pulse conversion
# --------------------------------------------------------------------------


def rad_to_deci_deg(angle_rad):
    # type: (float) -> int
    """ROS radians -> firmware 0.1 deg output shaft argument, symmetric rounding.

    The gear ratio is NOT applied here: the firmware argument is already an
    output shaft angle (BSP/robot.c:441, 450).
    """
    if angle_rad != angle_rad or angle_rad in (float("inf"), float("-inf")):
        raise ValueError("angle_rad must be finite, got %r" % (angle_rad,))
    return round_half_away_from_zero(angle_rad * 1800.0 / math.pi)


def deci_deg_to_rad(value_deci_deg):
    # type: (int) -> float
    """Firmware 0.1 deg -> radians."""
    return int(value_deci_deg) * math.pi / 1800.0


def degrees_to_deci_deg(angle_deg):
    # type: (float) -> int
    if not math.isfinite(angle_deg):
        raise ValueError("angle_deg must be finite, got %r" % (angle_deg,))
    return round_half_away_from_zero(angle_deg * 10.0)


def deci_deg_to_degrees(value_deci_deg):
    """Firmware 0.1 degree -> output-shaft degrees."""
    return int(value_deci_deg) / 10.0


def deci_deg_to_steps(value_deci_deg, gear):
    # type: (int, int) -> int
    """Firmware step count for a MOVE argument: (arg*gear*6 +/- 1800)/3600.

    Matches BSP/robot.c:450 exactly, including the sign dependent bias and the
    truncation toward zero. arg = 0 yields 0 steps (the -1800 bias truncates to 0).
    """
    arg = int(value_deci_deg)
    bias = 1800 if arg > 0 else -1800
    return trunc_div(arg * int(gear) * FG_PULSES_PER_MOTOR_REV + bias, 3600)


def steps_to_deci_deg(steps, gear):
    # type: (int, int) -> float
    """Output shaft angle in 0.1 deg for a signed FG count, truncated toward zero.

    Matches BSP/robot.c:85 (`pulses*3600/(gear*6)`); gear 0 yields 0. Returned as
    float so callers can divide by 10 without losing the truncation semantics.
    """
    if not gear:
        return 0.0
    return float(trunc_div(int(steps) * 3600, int(gear) * FG_PULSES_PER_MOTOR_REV))


def steps_to_rad(steps, gear):
    # type: (int, int) -> float
    return deci_deg_to_rad(steps_to_deci_deg(steps, gear))


def quantises_to_zero(value_deci_deg, gear):
    # type: (int, int) -> bool
    """True when a MOVE argument produces zero steps at this gear ratio.

    This is NOT the same as `value_deci_deg == 0`. At gear 82 an argument of
    0.1 deg yields 0 steps, and the firmware still walks the channel through
    RUNNING and SETTLING to DONE without the output shaft moving at all. A
    client that only checks for a zero argument would report a completed motion
    that never happened, so the driver answers NO_OP before sending anything.
    """
    return deci_deg_to_steps(value_deci_deg, gear) == 0


def tolerance_pulses(gear):
    # type: (int) -> int
    """First FG error that exceeds the firmware's arrival tolerance.

    BSP/robot.c:354 fails with F_POSITION when `error*3600 > 50*6*gear`, i.e.
    when error > gear/12. That gives 44 pulses at gear 516, 25 at 290 and 7 at 82.
    """
    if not gear:
        return 0
    return int(gear) // 12 + 1


# --------------------------------------------------------------------------
# PDU construction
# --------------------------------------------------------------------------


def read_pdu(address, count):
    # type: (int, int) -> bytes
    """FC 0x03 read holding registers. Firmware accepts count 1..125."""
    if not 1 <= count <= 125:
        raise ValueError("read count must be 1..125, got %r" % (count,))
    if not 0 <= address <= 0xFFFF or address + count > 0x10000:
        raise ValueError("read range 0x%04X+%d out of bounds" % (address, count))
    return struct.pack(">BHH", 3, address, count)


def write_single_pdu(address, value):
    # type: (int, int) -> bytes
    """FC 0x06 write one register. Never valid for the command block."""
    return struct.pack(">BHH", 6, address, value & 0xFFFF)


def write_multi_pdu(address, values):
    # type: (int, Sequence[int]) -> bytes
    """FC 0x10 write contiguous registers. Firmware accepts count 1..123."""
    count = len(values)
    if not 1 <= count <= 123:
        raise ValueError("write count must be 1..123, got %r" % (count,))
    payload = struct.pack(">%dH" % count, *[int(v) & 0xFFFF for v in values])
    return struct.pack(">BHHB", 16, address, count, len(payload)) + payload


def command_values(sequence, opcode, channel, argument):
    # type: (int, int, int, int) -> List[int]
    """The five register values of one command: seq, op, ch, arg_hi, arg_lo.

    sequence must be non-zero: zero asks the board to allocate one, which is not
    replay safe and is therefore refused here (BSP/robot.h:103).
    """
    if not 1 <= int(sequence) <= SEQUENCE_MAX:
        raise ValueError("sequence must be 1..65535 for a replayable command, got %r" % (sequence,))
    opcode = int(opcode)
    if not 1 <= opcode <= 12:
        raise ValueError("unknown opcode %r" % (opcode,))
    if not 0 <= int(channel) <= 0xFFFF:
        raise ValueError("channel out of range: %r" % (channel,))
    high, low = pack_i32(argument)
    return [int(sequence) & 0xFFFF, opcode, int(channel) & 0xFFFF, high, low]


def command_pdu(sequence, opcode, channel, argument):
    # type: (int, int, int, int) -> bytes
    """The only legal way to issue a command: FC16, address 0x0030, count 5."""
    return write_multi_pdu(REG_COMMAND, command_values(sequence, opcode, channel, argument))


# --------------------------------------------------------------------------
# Response decoding
# --------------------------------------------------------------------------


def parse_read_response(pdu, count):
    # type: (bytes, int) -> List[int]
    """Validate an FC3 response PDU (function byte onwards) and return the registers."""
    if len(pdu) < 2:
        raise FrameError("read response too short: %s" % pdu.hex())
    if pdu[0] != 3:
        raise FrameError("expected FC3, got 0x%02X" % pdu[0])
    byte_count = pdu[1]
    if byte_count != count * 2:
        raise FrameError("FC3 byte count %d does not match %d registers" % (byte_count, count))
    if len(pdu) < 2 + byte_count:
        raise FrameError("FC3 payload truncated: %d of %d bytes" % (len(pdu) - 2, byte_count))
    return list(struct.unpack(">%dH" % count, pdu[2:2 + byte_count]))


def validate_write_echo(pdu, request_pdu):
    # type: (bytes, bytes) -> None
    """FC6/FC16 responses echo address and count; a mismatch means a desync."""
    if pdu[0] != request_pdu[0]:
        raise FrameError("write echo function 0x%02X != request 0x%02X" % (pdu[0], request_pdu[0]))
    if pdu[1:5] != request_pdu[1:5]:
        raise FrameError("write echo %s != request %s" % (pdu[1:5].hex(), request_pdu[1:5].hex()))


@dataclass
class ChannelConfigValues(object):
    """Applied or staged configuration of one channel (5 registers)."""

    enabled: bool = False
    role: int = 0
    gear: int = 0
    inverted: int = 0
    rpm: int = RPM_MAX

    @classmethod
    def from_registers(cls, registers):
        # type: (Sequence[int]) -> "ChannelConfigValues"
        if len(registers) < 5:
            raise FrameError("channel config needs 5 registers, got %d" % len(registers))
        return cls(bool(registers[0]), int(registers[1]), int(registers[2]),
                   int(registers[3]), int(registers[4]))

    def to_registers(self):
        # type: () -> List[int]
        return [1 if self.enabled else 0, self.role & 0xFFFF, self.gear & 0xFFFF,
                self.inverted & 0xFFFF, self.rpm & 0xFFFF]


@dataclass
class ChannelStatus(object):
    """Registers base+0x10..0x16."""

    state: int = 0
    fault: int = 0
    valid: bool = False
    brake: bool = False
    direction: bool = False
    duty_permille: int = 0
    rpm: int = 0

    @classmethod
    def from_registers(cls, registers):
        # type: (Sequence[int]) -> "ChannelStatus"
        if len(registers) < 7:
            raise FrameError("channel status needs 7 registers, got %d" % len(registers))
        return cls(int(registers[0]), int(registers[1]), bool(registers[2]),
                   bool(registers[3]), bool(registers[4]), int(registers[5]),
                   int(registers[6]))

    @property
    def busy(self):
        # type: () -> bool
        return self.state in BUSY_STATES

    @property
    def state_name(self):
        # type: () -> str
        return state_name(self.state)

    @property
    def fault_name(self):
        # type: () -> str
        return fault_name(self.fault)


@dataclass
class ChannelPosition(object):
    """Registers base+0x18..0x1D: three int32, high word first."""

    angle_deci_deg: int = 0
    fg_count: int = 0
    target_deci_deg: int = 0

    @classmethod
    def from_registers(cls, registers):
        # type: (Sequence[int]) -> "ChannelPosition"
        if len(registers) < 6:
            raise FrameError("channel position needs 6 registers, got %d" % len(registers))
        return cls(unpack_i32(registers[0], registers[1]),
                   unpack_i32(registers[2], registers[3]),
                   unpack_i32(registers[4], registers[5]))


@dataclass
class SelftestRecord(object):
    """Registers base+0x20..0x2F. RAM only: a power cycle clears it."""

    state: int = 0
    reason: int = 0
    pulses: int = 0
    powered_ms: int = 0
    completed_uptime_ms: int = 0
    sequence: int = 0
    phase: int = 0
    return_ms: int = 0
    outbound_pulses: int = 0
    return_pulses: int = 0
    residual_pulses: int = 0

    @classmethod
    def from_registers(cls, summary, detail=None):
        # type: (Sequence[int], Optional[Sequence[int]]) -> "SelftestRecord"
        if len(summary) < 8:
            raise FrameError("selftest summary needs 8 registers, got %d" % len(summary))
        record = cls(int(summary[0]), int(summary[1]), unpack_u32(summary[2], summary[3]),
                     int(summary[4]), unpack_u32(summary[5], summary[6]), int(summary[7]))
        if detail is not None:
            if len(detail) < 8:
                raise FrameError("selftest detail needs 8 registers, got %d" % len(detail))
            record.phase = int(detail[0])
            record.return_ms = int(detail[1])
            record.outbound_pulses = unpack_u32(detail[2], detail[3])
            record.return_pulses = unpack_u32(detail[4], detail[5])
            record.residual_pulses = unpack_i32(detail[6], detail[7])
        return record

    @property
    def finished(self):
        # type: () -> bool
        return self.state in SELFTEST_FINAL_STATES


@dataclass
class BoardInfo(object):
    """Board level registers 0x0000..0x0007."""

    schema: int = 0
    protocol: int = 0
    address: int = 0
    node: int = 0
    pwm_hz: int = 0
    dirty: bool = False
    storage_error: bool = False
    pending_save: bool = False

    @classmethod
    def from_registers(cls, registers):
        # type: (Sequence[int]) -> "BoardInfo"
        if len(registers) < 8:
            raise FrameError("board info needs 8 registers, got %d" % len(registers))
        return cls(int(registers[0]), int(registers[1]), int(registers[2]), int(registers[3]),
                   int(registers[4]), bool(registers[5]), bool(registers[6]), bool(registers[7]))

    @property
    def save_clean(self):
        # type: () -> bool
        """True when nothing is pending: dirty, storage_error and pending_save all zero."""
        return not (self.dirty or self.storage_error or self.pending_save)


def uid_from_registers(registers):
    # type: (Sequence[int]) -> str
    """Six registers -> 24 uppercase hex characters, high word of each 32 bit word first."""
    if len(registers) < 6:
        raise FrameError("uid needs 6 registers, got %d" % len(registers))
    return "".join("%04X" % (int(v) & 0xFFFF) for v in registers[:6])


def validate_uid(uid):
    # type: (str) -> str
    """Normalise and validate a full 24 hex UID as used by tools/robot_host.py:177."""
    if not isinstance(uid, str):
        raise ValueError("uid must be a string, got %r" % (uid,))
    text = uid.strip().upper()
    if len(text) != 24 or any(c not in "0123456789ABCDEF" for c in text):
        raise ValueError("uid must be exactly 24 hex characters, got %r" % (uid,))
    return text


# --------------------------------------------------------------------------
# Validation of configuration values (firmware domains)
# --------------------------------------------------------------------------


def validate_gear(gear):
    # type: (int) -> int
    if int(gear) not in GEAR_RATIOS:
        raise ValueError("gear must be one of %s, got %r" % (list(GEAR_RATIOS), gear))
    return int(gear)


def validate_rpm(rpm):
    # type: (int) -> int
    if not RPM_MIN <= int(rpm) <= RPM_MAX:
        raise ValueError("pre-gear rpm must be %d..%d, got %r" % (RPM_MIN, RPM_MAX, rpm))
    return int(rpm)


def validate_pwm_hz(pwm_hz):
    # type: (int) -> int
    if not PWM_HZ_MIN <= int(pwm_hz) <= PWM_HZ_MAX:
        raise ValueError("pwm_hz must be %d..%d, got %r" % (PWM_HZ_MIN, PWM_HZ_MAX, pwm_hz))
    return int(pwm_hz)


def validate_address(address):
    # type: (int) -> int
    if not ADDRESS_MIN <= int(address) <= ADDRESS_MAX:
        raise ValueError("modbus address must be %d..%d, got %r" % (ADDRESS_MIN, ADDRESS_MAX, address))
    return int(address)


def validate_node(node):
    # type: (int) -> int
    if not NODE_MIN <= int(node) <= NODE_MAX:
        raise ValueError("can node must be %d..%d, got %r" % (NODE_MIN, NODE_MAX, node))
    return int(node)


def validate_move_argument(value_deci_deg):
    # type: (int) -> int
    """Firmware rejects |arg| > 3600 with R_VALUE (BSP/robot.c:441)."""
    value = int(value_deci_deg)
    if not -MAX_MOVE_DECI_DEG <= value <= MAX_MOVE_DECI_DEG:
        raise ValueError("MOVE argument must be -%d..%d (0.1 deg), got %d"
                         % (MAX_MOVE_DECI_DEG, MAX_MOVE_DECI_DEG, value))
    return value


def validate_boolean_flag(value, name):
    # type: (int, str) -> int
    if int(value) not in (0, 1):
        raise ValueError("%s must be 0 or 1, got %r" % (name, value))
    return int(value)


def validate_roles_unique(roles):
    # type: (Sequence[int]) -> None
    """Assigned roles must not repeat across the four channels (BSP/robot.c:19-20).

    Callers must pass the role of EVERY channel, disabled ones included: the
    firmware loop considers a channel as soon as its role is non-zero, so a
    disabled channel left holding a role still collides. Filtering by `enabled`
    here would accept an image the board rejects with R_VALUE.
    """
    seen = set()
    for role in roles:
        role = int(role)
        if not 0 <= role <= 7:
            raise ValueError("role must be 0..7, got %r" % (role,))
        if role == int(Role.UNASSIGNED):
            continue
        if role in seen:
            raise ValueError("duplicate role %d across channels" % role)
        seen.add(role)


#: Board-level values the firmware itself accepts (BSP/robot.c:13-14).
PROTOCOL_VALUES = (0, 1)


def validate_board_image(channels, board=None):
    # type: (Sequence[ChannelConfigValues], Optional[Sequence[int]]) -> List[str]
    """Reasons a staged configuration image would be rejected, [] when acceptable.

    A transcription of `robot_config_valid()` (BSP/robot.c:11-23) so the driver
    refuses locally what the board would refuse with R_VALUE, instead of learning it
    after a flash write. Two rules are easy to get wrong and are honoured here:
      * enabled/role/inverted/rpm/gear ranges apply to EVERY channel, disabled ones
        included (rpm 400..8000 and a legal gear even for a disabled channel);
      * role uniqueness includes disabled channels that still hold a non-zero role.
    """
    reasons = []
    if board is not None:
        protocol_value, address, node, pwm_hz = board
        if int(protocol_value) not in PROTOCOL_VALUES:
            reasons.append("protocol must be 0 or 1, got %r" % (protocol_value,))
        for check, value in ((validate_address, address), (validate_node, node),
                             (validate_pwm_hz, pwm_hz)):
            try:
                check(value)
            except ValueError as error:
                reasons.append(str(error))
    if len(channels) != CHANNEL_COUNT:
        reasons.append("the firmware validates exactly %d channels, got %d"
                       % (CHANNEL_COUNT, len(channels)))
    seen_roles = set()
    for index, values in enumerate(channels):
        enabled, role, gear = int(values.enabled), int(values.role), int(values.gear)
        inverted, rpm = int(values.inverted), int(values.rpm)
        if enabled not in (0, 1):
            reasons.append("channel %d: enabled must be 0 or 1, got %r" % (index, values.enabled))
        if not 0 <= role <= 7:
            reasons.append("channel %d: role must be 0..7, got %r" % (index, role))
        if inverted not in (0, 1):
            reasons.append("channel %d: inverted must be 0 or 1, got %r" % (index, inverted))
        if not RPM_MIN <= rpm <= RPM_MAX:
            reasons.append("channel %d: rpm must be %d..%d even when disabled, got %r"
                           % (index, RPM_MIN, RPM_MAX, rpm))
        if gear not in GEAR_RATIOS:
            reasons.append("channel %d: gear must be one of %s even when disabled, got %r"
                           % (index, list(GEAR_RATIOS), gear))
        if enabled and (not gear or not role):
            reasons.append("channel %d: an enabled channel needs a non-zero gear and role" % index)
        if role:
            if role in seen_roles:
                reasons.append("channel %d: role %d duplicates another channel (checked even when "
                               "the channel is disabled)" % (index, role))
            seen_roles.add(role)
    return reasons


def next_sequence(last_sequence):
    # type: (int) -> int
    """Sequence after the board's last one, wrapping past zero (BSP/robot.c:468-479)."""
    value = (int(last_sequence) + 1) & SEQUENCE_MAX
    return value if value else 1


def sequence_distance(candidate, last_sequence):
    # type: (int, int) -> int
    """Forward distance mod 65536; the board rejects >= 0x8000 (BSP/robot.c:484-485)."""
    return (int(candidate) - int(last_sequence)) & SEQUENCE_MAX


def sequence_acceptable(candidate, last_sequence):
    # type: (int, int) -> bool
    return sequence_distance(candidate, last_sequence) < SEQUENCE_WINDOW


__all__ = [name for name in dir() if not name.startswith("_")]
