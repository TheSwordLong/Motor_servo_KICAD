"""Board session: identity binding, sequence allocation, command reconciliation.

One BoardSession owns one transport (one serial port). It is deliberately free of
threads and of ROS imports so it can be unit tested with a fake transport.

Rules enforced here, all traceable to firmware behaviour:

* Identity first. The UID (0x0008..0x000D) must match the configured board and
  the command protocol revision (0x000F) must be 2 before ANY write is allowed.
  A mismatch is a hard error, not a warning.
* Non-zero sequences only, allocated board-wide from 0x0036 + 1 and wrapping
  past zero. Sequence 0 asks the board to allocate one, which is not replay
  safe, so it is refused (BSP/robot.h:103).
* A timeout is reconciled, never papered over. Because the board records every
  ACCEPTED request in 0x0036/0x0037, reading those two registers after a timeout
  tells us whether the command was taken. If that read also fails, the outcome is
  UNKNOWN: the session stops issuing new motions until an operator acknowledges.
* Retries reuse the SAME sequence and the SAME bytes. Resending an identical
  command is side-effect free thanks to the board's 16 entry dedupe cache
  (BSP/robot.c:480-483); inventing a new sequence is what causes double motion.
* Read-only means read-only. Every write is refused, including STOP, CONFIG,
  ZERO and CLEAR_FAULT. Stopping from software is a best-effort measure and must
  never be mistaken for the independent hardware e-stop.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from . import protocol as proto
from .protocol import (BoardInfo, ChannelConfigValues, ChannelPosition, ChannelStatus,
                       IdentityError, ModbusExceptionResponse, ModbusTimeout, Opcode,
                       ProtocolError, ReadOnlyViolation, SelftestRecord)

#: Driver level result codes. They start at 100 so they can never be confused
#: with a firmware business result (0..6) in the same uint16 field.
DRIVER_OK = 0
DRIVER_READONLY = 100
DRIVER_IDENTITY = 101
DRIVER_UNKNOWN = 102
DRIVER_TIMEOUT = 103
DRIVER_VALID_REQUIRED = 104
DRIVER_LIMIT = 105
DRIVER_NO_OP = 106
DRIVER_BUSY = 107
DRIVER_CONFIG_MISMATCH = 108
DRIVER_HALTED = 109
DRIVER_NOT_CONNECTED = 110
DRIVER_RPM_MISMATCH = 111
DRIVER_LIMITS_UNCONFIRMED = 112
DRIVER_STALE_POSITION = 113
DRIVER_BAD_REQUEST = 114

DRIVER_RESULT_NAMES = {
    DRIVER_OK: "ok",
    DRIVER_READONLY: "driver refused: read-only mode intercepts every write",
    DRIVER_IDENTITY: "driver refused: board identity not confirmed",
    DRIVER_UNKNOWN: "command outcome unknown; reconcile before any new motion",
    DRIVER_TIMEOUT: "no reply and reconciliation failed",
    DRIVER_VALID_REQUIRED: "absolute move needs a valid zero reference",
    DRIVER_LIMIT: "goal rejected by configured limits",
    DRIVER_NO_OP: "requested motion quantises to zero; nothing was sent",
    DRIVER_BUSY: "another motion owns this joint or the board",
    DRIVER_CONFIG_MISMATCH: "staged configuration did not read back as requested",
    DRIVER_HALTED: "driver halted after an unknown command outcome",
    DRIVER_NOT_CONNECTED: "transport is not connected",
    DRIVER_RPM_MISMATCH: ("requested motor_rpm differs from the rpm the board has stored for "
                          "this channel; nothing was sent"),
    DRIVER_LIMITS_UNCONFIRMED: ("mechanical limits are unconfirmed for this joint, so the driver "
                                "cannot verify where the motion ends"),
    DRIVER_STALE_POSITION: ("no fresh position could be read, so the driver refused to compute a "
                            "movement from old data"),
    DRIVER_BAD_REQUEST: "the driver was asked for something malformed; nothing was sent",
}


@dataclass
class CommandOutcome(object):
    """What we actually know about one command after sending it."""

    sequence: int = 0
    opcode: int = 0
    channel: int = 0
    argument: int = 0
    accepted: bool = False
    result: int = -1            # firmware business result 0..6, or a driver code >= 100
    unknown: bool = False       # outcome could not be established
    detail: str = ""
    request_id: str = ""        # logged identity, for retry and for the record
    sent_monotonic: float = 0.0
    sent_wall: float = 0.0
    reconciled: bool = False    # result came from 0x0036/0x0037 rather than the reply

    @property
    def result_name(self):
        # type: () -> str
        if self.result in DRIVER_RESULT_NAMES:
            return DRIVER_RESULT_NAMES[self.result]
        try:
            return proto.BusinessResult(self.result).name.lower()
        except ValueError:
            return "unknown(%d)" % self.result


@dataclass
class BoardSnapshot(object):
    """Everything the node publishes, captured in one polling pass."""

    monotonic: float = 0.0
    wall: float = 0.0
    info: Optional[BoardInfo] = None
    uid: str = ""
    command_protocol: int = 0
    have_sequence: bool = False
    last_sequence: int = 0
    last_result: int = 0
    configs: List[ChannelConfigValues] = field(default_factory=list)
    staged: List[ChannelConfigValues] = field(default_factory=list)
    statuses: List[ChannelStatus] = field(default_factory=list)
    positions: List[ChannelPosition] = field(default_factory=list)
    selftests: List[SelftestRecord] = field(default_factory=list)
    capabilities: int = 0
    manual_modes: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    communication_ok: bool = False
    error: str = ""

    def channel_status(self, channel):
        # type: (int) -> Optional[ChannelStatus]
        return self.statuses[channel] if 0 <= channel < len(self.statuses) else None

    def channel_position(self, channel):
        # type: (int) -> Optional[ChannelPosition]
        return self.positions[channel] if 0 <= channel < len(self.positions) else None

    @property
    def save_clean(self):
        # type: () -> bool
        return bool(self.info) and self.info.save_clean

    @property
    def busy_channels(self):
        # type: () -> List[int]
        return [index for index, status in enumerate(self.statuses) if status.busy]


class RequestLog(object):
    """Bounded log of every command identity, written BEFORE the frame is sent."""

    def __init__(self, capacity=256):
        # type: (int) -> None
        self._entries = []   # type: List[dict]
        self._capacity = int(capacity)
        self._lock = threading.Lock()

    def record(self, entry):
        # type: (dict) -> dict
        with self._lock:
            self._entries.append(entry)
            if len(self._entries) > self._capacity:
                del self._entries[:len(self._entries) - self._capacity]
        return entry

    def entries(self):
        # type: () -> List[dict]
        with self._lock:
            return list(self._entries)

    def find_by_sequence(self, sequence):
        # type: (int) -> List[dict]
        with self._lock:
            return [entry for entry in self._entries if entry.get("sequence") == sequence]

    def __len__(self):
        # type: () -> int
        with self._lock:
            return len(self._entries)


class BoardSession(object):
    """Stateful conversation with one board over one transport."""

    def __init__(self, transport, expected_uid=None, read_only=True,
                 require_protocol=proto.COMMAND_PROTOCOL_REVISION,
                 logger=None, clock=None, request_log=None,
                 command_retries=1):
        # type: (...) -> None
        self.capabilities = 0
        self.transport = transport
        self.expected_uid = proto.validate_uid(expected_uid) if expected_uid else None
        self.read_only = bool(read_only)
        self.require_protocol = int(require_protocol)
        self._log = logger or (lambda level, message: None)   # type: Callable[[str, str], None]
        self._clock = clock or time.monotonic
        self.request_log = request_log or RequestLog()
        self.command_retries = int(command_retries)
        self._lock = threading.RLock()
        self._uid = ""                       # type: str
        self._protocol = 0                   # type: int
        self._identified = False
        self._last_sequence = 0              # type: int
        self._have_sequence = False
        #: Local mirror of the board's 16 entry dedupe cache: seq -> (op, ch, arg, result)
        self._dedupe = {}                    # type: Dict[int, Tuple[int, int, int, int]]
        self._dedupe_order = []              # type: List[int]
        self.unknown_command_state = False
        self.unknown_detail = ""
        self.reconnects = 0

    # -- identity ----------------------------------------------------------

    @property
    def uid(self):
        # type: () -> str
        return self._uid

    @property
    def command_protocol(self):
        # type: () -> int
        return self._protocol

    @property
    def identified(self):
        # type: () -> bool
        return self._identified

    def identify(self):
        # type: () -> str
        """Read UID and command protocol revision, then bind or reject the board."""
        with self._lock:
            registers = self.transport.read_registers(proto.REG_SCHEMA, 16)
            info = BoardInfo.from_registers(registers[:8])
            uid = proto.uid_from_registers(registers[proto.REG_UID:proto.REG_UID + 6])
            have_seq = bool(registers[proto.REG_HAVE_SEQ])
            revision = int(registers[proto.REG_COMMAND_PROTOCOL])
            if self.expected_uid and uid != self.expected_uid:
                self._identified = False
                raise IdentityError(
                    "board UID %s does not match the configured %s; refusing every write"
                    % (uid, self.expected_uid))
            if revision != self.require_protocol:
                self._identified = False
                raise IdentityError(
                    "command protocol revision is %d, this driver requires %d; "
                    "firmware and client must be updated together" % (revision, self.require_protocol))
            try:
                self.capabilities = int(self.transport.read_registers(proto.REG_CAPABILITIES, 1)[0])
            except proto.ModbusExceptionResponse as error:
                if error.code != 2:
                    raise
                self.capabilities = 0
            self._uid = uid
            self._protocol = revision
            self._have_sequence = have_seq
            self._identified = True
            self._log("info", "board identified uid=%s protocol=%d schema=%d dirty=%s"
                      % (uid, revision, info.schema, info.dirty))
            return uid

    def ensure_identified(self):
        # type: () -> None
        if not self._identified:
            self.identify()

    # -- sequence handling -------------------------------------------------

    def read_last_sequence(self):
        # type: () -> Tuple[int, int]
        """(last_sequence, last_result) from 0x0036/0x0037."""
        values = self.transport.read_registers(proto.REG_LAST_SEQUENCE, 2)
        with self._lock:
            self._last_sequence = int(values[0])
            if self._last_sequence:
                self._have_sequence = True
        return int(values[0]), int(values[1])

    def allocate_sequence(self):
        # type: () -> int
        """Next non-zero sequence, skipping anything still in the dedupe window.

        The board rejects a sequence whose distance from its last one is >= 0x8000
        and returns R_SEQUENCE when a cached sequence is reused with different
        parameters, so a fresh sequence must be outside the local mirror of the
        board's 16 entry cache.
        """
        with self._lock:
            last_sequence, _ = self.read_last_sequence()
            candidate = proto.next_sequence(last_sequence)
            for _ in range(proto.SEQUENCE_CACHE_ENTRIES + 1):
                if candidate not in self._dedupe:
                    break
                candidate = proto.next_sequence(candidate)
            if not proto.sequence_acceptable(candidate, last_sequence):
                raise ProtocolError(
                    "cannot allocate a sequence inside the board window: last=%d candidate=%d"
                    % (last_sequence, candidate))
            self._last_sequence = candidate
            return candidate

    def _remember(self, sequence, opcode, channel, argument, result):
        # type: (int, int, int, int, int) -> None
        with self._lock:
            if sequence not in self._dedupe:
                self._dedupe_order.append(sequence)
            self._dedupe[sequence] = (int(opcode), int(channel), int(argument), int(result))
            while len(self._dedupe_order) > proto.SEQUENCE_CACHE_ENTRIES:
                oldest = self._dedupe_order.pop(0)
                self._dedupe.pop(oldest, None)

    def cached_result(self, sequence, opcode, channel, argument):
        # type: (int, int, int, int) -> Optional[int]
        """Result the board would replay for an identical command, or None."""
        with self._lock:
            entry = self._dedupe.get(int(sequence))
            if entry and entry[:3] == (int(opcode), int(channel), int(argument)):
                return entry[3]
        return None

    # -- write gating ------------------------------------------------------

    #: The only commands that may be attempted while the driver is halted after an
    #: UNKNOWN outcome. Stopping is how an operator regains control, so refusing it
    #: would make the halt strictly worse than the problem it prevents. BRAKE is
    #: deliberately NOT on this list even though the firmware treats arg=1 as a
    #: stop: the requirement is to let STOP/STOP_ALL through and nothing else.
    STOP_OPCODES = (int(Opcode.STOP), int(Opcode.STOP_ALL))

    def _guard_write(self, what, allow_while_unknown=False):
        # type: (str, bool) -> None
        if self.read_only:
            raise ReadOnlyViolation(
                "read-only mode refuses %s; every write is intercepted, including STOP. "
                "Enable motion explicitly (allow_motion:=true) or use the maintenance entry."
                % what)
        if self.unknown_command_state and not allow_while_unknown:
            raise ProtocolError(
                "driver halted: %s is refused while a previous command outcome is UNKNOWN "
                "(%s); acknowledge it first" % (what, self.unknown_detail))

    def _require_identity(self):
        # type: () -> None
        if not self._identified:
            raise IdentityError("board identity is not confirmed; refusing to write")

    def mark_unknown(self, detail):
        # type: (str) -> None
        """Halt motion scheduling after an irreconcilable outcome."""
        with self._lock:
            self.unknown_command_state = True
            self.unknown_detail = detail
        self._log("warn", "UNKNOWN command state: %s" % detail)

    def acknowledge_unknown(self):
        # type: () -> str
        """Operator acknowledgement. Does NOT resume anything automatically."""
        with self._lock:
            detail = self.unknown_detail
            self.unknown_command_state = False
            self.unknown_detail = ""
        self._log("warn", "operator acknowledged UNKNOWN state (%s); no motion was resumed" % detail)
        return detail

    # -- commands ----------------------------------------------------------

    def send_command(self, opcode, channel=0, argument=0, sequence=None,
                     ros_target_id="", retries=None, allow_while_unknown=False):
        # type: (int, int, int, Optional[int], str, Optional[int], bool) -> CommandOutcome
        """Send one single-frame command and return what is known about it.

        `allow_while_unknown` is honoured for STOP/STOP_ALL only; asking for it on
        any other opcode does not bypass the halt.
        """
        opcode = int(opcode)
        name = proto.opcode_name(opcode)
        with self._lock:
            permitted = bool(allow_while_unknown) and opcode in self.STOP_OPCODES
            if allow_while_unknown and not permitted and self.unknown_command_state:
                raise ProtocolError(
                    "driver halted: %s is not a stop command, so it stays refused while a "
                    "previous outcome is UNKNOWN (%s)" % (name, self.unknown_detail))
            self._guard_write("%s command" % name, allow_while_unknown=permitted)
            self._require_identity()
            if sequence is None:
                sequence = self.allocate_sequence()
            else:
                sequence = int(sequence)
                if not 1 <= sequence <= proto.SEQUENCE_MAX:
                    raise ValueError("sequence must be 1..65535, got %r" % (sequence,))
                last_sequence, _ = self.read_last_sequence()
                if not proto.sequence_acceptable(sequence, last_sequence):
                    raise ProtocolError(
                        "sequence %d is outside the board window (last %d)" % (sequence, last_sequence))
            outcome = CommandOutcome(sequence=sequence, opcode=opcode, channel=int(channel),
                                     argument=int(argument))
            outcome.sent_monotonic = self._clock()
            outcome.sent_wall = time.time()
            outcome.request_id = "uid=%s target=%s seq=%d op=%s ch=%d arg=%d" % (
                self._uid or "?", ros_target_id or "-", sequence, name, channel, argument)
            # The identity is logged BEFORE the frame leaves, so a lost reply can
            # still be retried with the same sequence.
            self.request_log.record({"sequence": sequence, "opcode": opcode, "channel": int(channel),
                                     "argument": int(argument), "uid": self._uid,
                                     "ros_target_id": ros_target_id, "monotonic": outcome.sent_monotonic,
                                     "wall": outcome.sent_wall, "request_id": outcome.request_id})
            self._log("info", "TX %s" % outcome.request_id)
            attempts = self.command_retries if retries is None else int(retries)
            try:
                self.transport.write_command(sequence, opcode, channel, argument, retries=attempts)
                outcome.accepted = True
                outcome.result = int(proto.BusinessResult.OK)
                outcome.detail = "accepted"
                self._remember(sequence, opcode, channel, argument, outcome.result)
                return outcome
            except ModbusExceptionResponse as error:
                # The exception code is ambiguous; read the real business result.
                outcome.accepted = False
                outcome.detail = str(error)
                return self._reconcile(outcome, error.code)
            except ModbusTimeout as error:
                outcome.detail = "timeout: %s" % error
                return self._reconcile(outcome, None)
            except ProtocolError as error:
                outcome.detail = "transport error: %s" % error
                return self._reconcile(outcome, None)

    def _reconcile(self, outcome, exception_code):
        # type: (CommandOutcome, Optional[int]) -> CommandOutcome
        """Establish the real outcome from 0x0036/0x0037, or declare UNKNOWN."""
        try:
            last_sequence, last_result = self.read_last_sequence()
        except ProtocolError as error:
            outcome.unknown = True
            outcome.result = DRIVER_UNKNOWN
            outcome.detail += ("; reconciliation failed (%s), so the outcome is UNKNOWN: the frame "
                               "left the wire and the board may or may not have executed it. Do not "
                               "report this as success or as failure until an operator reconciles."
                               % error)
            self.mark_unknown(outcome.detail)
            return outcome
        outcome.reconciled = True
        if last_sequence == outcome.sequence:
            mirrored = self._dedupe.get(outcome.sequence)
            if exception_code is not None and mirrored is not None \
                    and mirrored[:3] != (outcome.opcode, outcome.channel,
                                         outcome.argument & 0xFFFFFFFF):
                # This sequence was already used with DIFFERENT parameters, so the
                # board answered from its dedupe cache path and 0x0037 still holds the
                # EARLIER command's result. Reading it as ours would report somebody
                # else's outcome, so name the real one instead.
                outcome.result = int(proto.BusinessResult.SEQUENCE)
                outcome.accepted = False
                outcome.unknown = False
                outcome.detail += ("; sequence %d was already used with different parameters, so "
                                   "0x0036/0x0037 still describe that earlier request and this one "
                                   "was rejected as a sequence conflict" % outcome.sequence)
                self._log("warn", outcome.detail)
                return outcome
            # The board recorded this exact request, so the result is authoritative
            # even when the reply frame itself was lost.
            outcome.result = int(last_result)
            outcome.accepted = outcome.result == int(proto.BusinessResult.OK)
            outcome.unknown = False
            outcome.detail += "; reconciled: board last_seq=%d result=%d" % (last_sequence, last_result)
            self._remember(outcome.sequence, outcome.opcode, outcome.channel,
                           outcome.argument, outcome.result)
            self._log("warn", "reconciled seq=%d result=%d" % (last_sequence, last_result))
            return outcome
        if exception_code is not None:
            # The board updates 0x0036 BEFORE running the business logic but AFTER
            # rejecting a bad sequence (BSP/robot.c:480-494). That ordering is the
            # only reliable discriminator for the ambiguous exception codes:
            #   last_seq unchanged  -> the sequence itself was rejected, so the
            #                      command definitely did NOT execute;
            #   last_seq == our seq  -> handled above, 0x0037 holds the real result.
            candidates = proto.AMBIGUOUS_EXCEPTIONS.get(exception_code)
            if candidates and exception_code == int(proto.ModbusException.ILLEGAL_DATA_VALUE):
                outcome.result = int(proto.BusinessResult.SEQUENCE)
                outcome.accepted = False
                outcome.unknown = False
                outcome.detail += ("; exception 0x03 with last_seq=%d unchanged: the sequence was "
                                   "rejected and the command did not execute" % last_sequence)
                self._log("warn", outcome.detail)
                return outcome
            if candidates:
                # Exception 0x04 without a recorded sequence should not happen; treat
                # it as unknown rather than guessing between UNREADY and STORAGE.
                outcome.result = DRIVER_UNKNOWN
                outcome.unknown = True
                outcome.detail += ("; exception 0x%02X is ambiguous between %s and the board did "
                                   "not record this sequence, so the outcome is UNKNOWN"
                                   % (exception_code, [c.name for c in candidates]))
                self.mark_unknown(outcome.detail)
                return outcome
            outcome.result = DRIVER_TIMEOUT
            outcome.accepted = False
            outcome.unknown = False
            outcome.detail += ("; exception 0x%02X and the board did not record this sequence"
                               % exception_code)
            return outcome
        # No reply at all and the board never recorded the sequence: the command
        # was not accepted. It is safe to retry with the SAME sequence.
        outcome.result = DRIVER_TIMEOUT
        outcome.accepted = False
        outcome.unknown = False
        outcome.detail += ("; board last_seq=%d differs, so the command was not accepted"
                           % last_sequence)
        self._log("warn", outcome.detail)
        return outcome

    # -- reads -------------------------------------------------------------

    def read_board_info(self):
        # type: () -> BoardInfo
        return BoardInfo.from_registers(self.transport.read_registers(proto.REG_SCHEMA, 8))

    def read_save_flags(self):
        # type: () -> Tuple[bool, bool, bool]
        values = self.transport.read_registers(proto.REG_DIRTY, 3)
        return bool(values[0]), bool(values[1]), bool(values[2])

    def read_channel_config(self, channel):
        # type: (int) -> ChannelConfigValues
        base = proto.channel_base(channel)
        return ChannelConfigValues.from_registers(
            self.transport.read_registers(base + proto.OFF_CONFIG_APPLIED, 5))

    def read_channel_staged(self, channel):
        # type: (int) -> ChannelConfigValues
        base = proto.channel_base(channel)
        return ChannelConfigValues.from_registers(
            self.transport.read_registers(base + proto.OFF_CONFIG_STAGED, 5))

    def read_channel_status(self, channel):
        # type: (int) -> ChannelStatus
        base = proto.channel_base(channel)
        return ChannelStatus.from_registers(self.transport.read_registers(base + proto.OFF_STATUS, 7))

    def read_channel_position(self, channel):
        # type: (int) -> ChannelPosition
        base = proto.channel_base(channel)
        return ChannelPosition.from_registers(self.transport.read_registers(base + proto.OFF_POSITION, 6))

    def read_status_and_position(self, channel):
        # type: (int) -> Tuple[ChannelStatus, ChannelPosition]
        """Status and position of one channel.

        These are NOT one contiguous read: offset 0x17 between them is unmapped, so
        a 14 register request covering base+0x10..0x1D is rejected as an illegal
        address. Two transactions are required.
        """
        base = proto.channel_base(channel)
        status = ChannelStatus.from_registers(
            self.transport.read_registers(base + proto.OFF_STATUS, 7))
        position = ChannelPosition.from_registers(
            self.transport.read_registers(base + proto.OFF_POSITION, 6))
        return status, position

    def read_selftest(self, channel):
        # type: (int) -> SelftestRecord
        base = proto.channel_base(channel)
        values = self.transport.read_registers(base + proto.OFF_SELFTEST, 16)
        return SelftestRecord.from_registers(values[:8], values[8:])

    def poll(self, include_selftest=True, include_staged=False):
        # type: (bool, bool) -> BoardSnapshot
        """One full read-only pass over the board. Never writes anything."""
        snapshot = BoardSnapshot(monotonic=self._clock(), wall=time.time())
        snapshot.capabilities = self.capabilities
        try:
            head = self.transport.read_registers(proto.REG_SCHEMA, 16)
            snapshot.info = BoardInfo.from_registers(head[:8])
            snapshot.uid = proto.uid_from_registers(head[proto.REG_UID:proto.REG_UID + 6])
            snapshot.have_sequence = bool(head[proto.REG_HAVE_SEQ])
            snapshot.command_protocol = int(head[proto.REG_COMMAND_PROTOCOL])
            tail = self.transport.read_registers(proto.REG_LAST_SEQUENCE, 2)
            snapshot.last_sequence, snapshot.last_result = int(tail[0]), int(tail[1])
            for channel in range(proto.CHANNEL_COUNT):
                if self.capabilities & proto.CAP_MANUAL_ADJUST:
                    snapshot.manual_modes[channel] = int(self.transport.read_registers(
                        proto.channel_base(channel) + proto.OFF_MANUAL_ADJUST, 1)[0])
                base = proto.channel_base(channel)
                snapshot.configs.append(ChannelConfigValues.from_registers(
                    self.transport.read_registers(base + proto.OFF_CONFIG_APPLIED, 5)))
                status, position = self.read_status_and_position(channel)
                snapshot.statuses.append(status)
                snapshot.positions.append(position)
                if include_selftest:
                    snapshot.selftests.append(self.read_selftest(channel))
                if include_staged:
                    snapshot.staged.append(ChannelConfigValues.from_registers(
                        self.transport.read_registers(base + proto.OFF_CONFIG_STAGED, 5)))
            snapshot.communication_ok = True
            with self._lock:
                self._uid = snapshot.uid
                self._protocol = snapshot.command_protocol
                self._last_sequence = snapshot.last_sequence
        except ProtocolError as error:
            snapshot.communication_ok = False
            snapshot.error = "%s: %s" % (type(error).__name__, error)
            self._log("error", "poll failed: %s" % snapshot.error)
        except OSError as error:
            snapshot.communication_ok = False
            snapshot.error = "serial error: %s" % error
            self._log("error", "poll failed: %s" % snapshot.error)
        return snapshot

    # -- configuration staging --------------------------------------------

    def stage_board_config(self, protocol_value, address, node, pwm_hz):
        # type: (int, int, int, int) -> None
        """Write the staged board block 0x0020..0x0023. Applies only after CONFIG."""
        self._guard_write("board configuration staging")
        self._require_identity()
        values = [int(protocol_value), proto.validate_address(address), proto.validate_node(node),
                  proto.validate_pwm_hz(pwm_hz)]
        self.transport.write_registers(proto.REG_STAGED_BOARD, values)

    def stage_channel_config(self, channel, config):
        # type: (int, ChannelConfigValues) -> None
        """Write one channel's staged configuration (base+8..12)."""
        self._guard_write("channel %d configuration staging" % channel)
        self._require_identity()
        base = proto.channel_base(channel)
        proto.validate_gear(config.gear)
        proto.validate_rpm(config.rpm)
        proto.validate_boolean_flag(config.inverted, "inverted")
        if not 0 <= int(config.role) <= 7:
            raise ValueError("role must be 0..7, got %r" % (config.role,))
        self.transport.write_registers(base + proto.OFF_CONFIG_STAGED, config.to_registers())

    def commit_config(self, ros_target_id=""):
        # type: (str) -> CommandOutcome
        """Send the single CONFIG command that validates and flashes the staged area."""
        return self.send_command(Opcode.CONFIG, 0, 0, ros_target_id=ros_target_id)

    # -- lifecycle ---------------------------------------------------------

    def reconnect(self):
        # type: () -> None
        """Re-open the transport after a loss. Never resumes a previous motion."""
        with self._lock:
            self.reconnects += 1
            self._identified = False
            self._dedupe.clear()
            del self._dedupe_order[:]
            # A reconnect invalidates every assumption about in-flight commands:
            # the board may have rebooted, which also clears its dedupe cache and
            # its RAM self-test records.
            self.mark_unknown("transport was reconnected; the board may have rebooted and its "
                              "16 entry dedupe cache and RAM self-test records are gone")
            try:
                self.transport.close()
            except ProtocolError:
                pass
            self.transport.open()
        self._log("warn", "transport reconnected (%d); identity must be re-established and no "
                          "motion was resumed" % self.reconnects)
