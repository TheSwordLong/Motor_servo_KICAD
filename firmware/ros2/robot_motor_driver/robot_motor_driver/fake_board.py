"""Simulated board plus a pyserial-shaped stub, for hardware-free testing.

The fake speaks at the ADU level: it receives complete Modbus frames, validates
CRC and address exactly like the firmware (silently dropping anything invalid,
BSP/robot_protocol.c:16), and answers with real frames. That means the production
`ModbusRtuTransport` runs unchanged on top of it, so framing, resynchronisation,
CRC errors, retries, dedupe replay and the sequence window are all testable
without a board.

The simulated semantics follow docs/protocol_contract.md: command block only via
FC16 0x0030 count 5, 16 entry dedupe ring cache, rejected commands still consume
a sequence, board-wide quiet requirements, the dirty / storage_error /
pending_save lifecycle, and MOVE deriving direction from the sign of the argument
XOR the inverted flag.

Deliberate simplifications, so nobody mistakes them for firmware behaviour:
motion is time based rather than a PWM and FG simulation, the decel ramp is two
speed segments, self test pulses are generated instead of captured, and the fake
clock can auto-advance so tests never sleep. None of this is evidence about real
hardware.
"""

import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from . import protocol as proto
from .protocol import BusinessResult, ModbusException, Opcode

DEFAULT_UID = "C23B378115B092083830364A"

#: Default channel configuration matching the audited board: M1 left upper arm
#: 516, M2 head 516, M3 left forearm 290, M4 left wrist 82.
DEFAULT_CHANNELS = (
    {"enabled": True, "role": int(proto.Role.LEFT_UPPER), "gear": 516, "inverted": 0, "rpm": 8000},
    {"enabled": True, "role": int(proto.Role.HEAD), "gear": 516, "inverted": 0, "rpm": 8000},
    {"enabled": True, "role": int(proto.Role.LEFT_FOREARM), "gear": 290, "inverted": 0, "rpm": 8000},
    {"enabled": True, "role": int(proto.Role.LEFT_PALM), "gear": 82, "inverted": 0, "rpm": 8000},
)


class FakeClock(object):
    """Manually advanced monotonic clock.

    `auto_advance` makes every reading move time forward a little, so a transport
    that is waiting for a reply reaches its deadline instead of spinning forever
    on a frozen clock. Tests that need exact control pass auto_advance=0.
    """

    def __init__(self, start=1000.0, auto_advance=0.001):
        # type: (float, float) -> None
        self._now = float(start)
        self.auto_advance = float(auto_advance)
        self.reads = 0

    def __call__(self):
        # type: () -> float
        self.reads += 1
        if self.auto_advance:
            self._now += self.auto_advance
        return self._now

    def advance(self, seconds):
        # type: (float) -> float
        self._now += float(seconds)
        return self._now

    def set(self, value):
        # type: (float) -> None
        self._now = float(value)


@dataclass
class FakeChannel(object):
    """One simulated output channel."""

    index: int = 0
    enabled: bool = False
    role: int = 0
    gear: int = 0
    inverted: int = 0
    rpm: int = proto.RPM_MAX
    staged: Tuple[int, int, int, int, int] = (0, 0, 0, 0, proto.RPM_MAX)

    state: int = int(proto.MotionState.UNREADY)
    fault: int = 0
    valid: bool = False
    brake: bool = True
    direction: bool = False
    duty: int = 0
    measured_rpm: int = 0

    fg_count: int = 0
    target_deci_deg: int = 0

    unexpected_fg: int = 0
    manual: int = 0
    recovery_events: int = 0
    recovery_started: float = 0.0
    stop_fault: bool = False
    last_edge: float = 0.0

    motion_started: float = 0.0
    motion_duration: float = 0.0
    motion_start_count: int = 0
    motion_steps: int = 0
    motion_settle_until: float = 0.0
    settle_to_idle: bool = False

    probe_started: float = 0.0
    probe_direction: int = 0
    probe_state: int = int(proto.SelftestState.UNTESTED)
    probe_reason: int = 0
    probe_phase: int = int(proto.SelftestPhase.NONE)
    probe_pulses: int = 0
    probe_powered_ms: int = 0
    probe_return_ms: int = 0
    probe_outbound: int = 0
    probe_return_pulses: int = 0
    probe_residual: int = 0
    probe_completed_ms: int = 0
    probe_sequence: int = 0

    def applied_config(self):
        # type: () -> proto.ChannelConfigValues
        return proto.ChannelConfigValues(self.enabled, self.role, self.gear,
                                         self.inverted, self.rpm)

    def staged_config(self):
        # type: () -> proto.ChannelConfigValues
        return proto.ChannelConfigValues.from_registers(list(self.staged))

    @property
    def angle_deci_deg(self):
        # type: () -> int
        return int(proto.steps_to_deci_deg(self.fg_count, self.gear))


class FakeBoard(object):
    """Register and command level simulation of one board."""

    def __init__(self, uid=DEFAULT_UID, address=1, node=1, pwm_hz=proto.PWM_HZ_DEFAULT,
                 protocol_value=0, command_protocol=proto.COMMAND_PROTOCOL_REVISION,
                 clock=None, channels=None):
        # type: (...) -> None
        self.uid = proto.validate_uid(uid)
        self.address = int(address)
        self.node = int(node)
        self.pwm_hz = int(pwm_hz)
        self.protocol_value = int(protocol_value)
        self.command_protocol = int(command_protocol)
        self.schema = proto.FLASH_SCHEMA
        self._clock = clock or time.monotonic
        self._boot_time = self._clock()
        self._lock = threading.RLock()

        self.dirty = False
        self.storage_error = False
        self.pending_save = False
        self.have_seq = False
        self.last_seq = 0
        self.last_result = 0
        self.stop_input = False
        self.last_command_block = [0, 0, 0, 0, 0]
        self._current_sequence = 0

        self.channels = []   # type: List[FakeChannel]
        for index in range(proto.CHANNEL_COUNT):
            spec = dict(channels[index] if channels else DEFAULT_CHANNELS[index])
            channel = FakeChannel(index=index, enabled=bool(spec.get("enabled", False)),
                                  role=int(spec.get("role", 0)), gear=int(spec.get("gear", 0)),
                                  inverted=int(spec.get("inverted", 0)),
                                  rpm=int(spec.get("rpm", proto.RPM_MAX)))
            channel.staged = tuple(channel.applied_config().to_registers())
            channel.last_edge = self._boot_time
            self.channels.append(channel)

        self.staged_board = [self.protocol_value, self.address, self.node, self.pwm_hz]
        #: Identity changes staged by CONFIG; they only become active after reboot().
        self.pending_identity = None    # type: Optional[List[int]]

        self.cache = {}             # type: Dict[int, Tuple[int, int, int, int]]
        self.cache_order = []       # type: List[int]

        self.commands_received = 0
        self.commands_executed = 0
        self.commands_replayed = 0
        self.silent_drops = 0
        self.flash_writes = 0
        self.injected_faults = {}   # type: Dict[int, int]
        self.storage_failure = False
        self.selftest_pulses = proto.SELFTEST_TARGET_PULSES + 2
        #: Probe session state. The firmware energises ONE channel at a time and only
        #: starts the next queued channel after the current one settles
        #: (BSP/robot.c:294-296), so a parallel model here could never exercise batch
        #: completion, a mid-batch failure, or the dropped-queue case.
        self.probe_active = False
        self.probe_queue = []       # type: List[int]
        self.probe_current = -1
        self.probe_aborting = False
        self.probe_scan_all = False
        self.probe_direction = 0
        #: Test knob: shrinks the per-leg probe timing. Never changes phase order or
        #: the pass/fail rules, only how long each leg takes in simulated time.
        self.probe_time_scale = 1.0
        self.motion_duration_scale = 1.0
        self.allow_auto_seq = True

    # ------------------------------------------------------------------ time

    def _now(self):
        # type: () -> float
        return self._clock()

    def _uptime_ms(self):
        # type: () -> int
        return int((self._now() - self._boot_time) * 1000)

    # ---------------------------------------------------------- simulation

    def advance(self, seconds=0.0):
        # type: (float) -> None
        """Progress the simulation; with no argument just re-evaluate the state."""
        with self._lock:
            if seconds:
                if hasattr(self._clock, "advance"):
                    self._clock.advance(seconds)
                else:
                    time.sleep(seconds)
            self._advance()

    def _advance(self):
        # type: () -> None
        now = self._now()
        for channel in self.channels:
            if channel.fault == int(proto.FaultCode.UNEXPECTED_FG) and not channel.manual and channel.recovery_events and (now-channel.recovery_started)*1000 >= proto.FAULT4_RECOVERY_MS:
                channel.fault = 0; channel.valid = False; channel.brake = True; channel.duty = 0
                channel.state = int(proto.MotionState.UNREADY)
                channel.unexpected_fg = 0
                channel.recovery_events = 0; channel.recovery_started = 0.0
            if channel.manual == 2 and (now-channel.last_edge)*1000 >= proto.QUIET_MS:
                channel.manual = 0
                channel.state = int(proto.MotionState.FAULT if channel.fault else proto.MotionState.UNREADY)
                self.pending_save = True
                continue
            if channel.manual:
                continue
            if channel.state in (int(proto.MotionState.RUNNING), int(proto.MotionState.DECEL)):
                self._advance_motion(channel, now)
            elif channel.state == int(proto.MotionState.SETTLING):
                if now >= channel.motion_settle_until:
                    if channel.settle_to_idle:
                        channel.state = (int(proto.MotionState.IDLE) if channel.valid
                                         else int(proto.MotionState.UNREADY))
                    else:
                        channel.state = int(proto.MotionState.DONE)
                        self.pending_save = True
                    channel.settle_to_idle = False
                    channel.brake = True
                    channel.duty = 0
                    channel.measured_rpm = 0
                    channel.last_edge = now
            elif channel.state == int(proto.MotionState.SELFTEST):
                self._advance_probe(channel, now)
        self._background_save(now)

    def _advance_motion(self, channel, now):
        # type: (FakeChannel, float) -> None
        elapsed = now - channel.motion_started
        duration = max(1e-6, channel.motion_duration)
        progress = min(1.0, elapsed / duration)
        if progress < 0.85:
            channel.state = int(proto.MotionState.RUNNING)
            channel.measured_rpm = channel.rpm
            channel.duty = min(proto.DUTY_MAX_PERMILLE, 320 + int(progress * 300))
        elif progress < 1.0:
            channel.state = int(proto.MotionState.DECEL)
            channel.measured_rpm = proto.CRAWL_RPM
            channel.duty = 200
        else:
            channel.state = int(proto.MotionState.SETTLING)
            channel.motion_settle_until = now + proto.QUIET_MS / 1000.0
            channel.measured_rpm = 0
            channel.duty = 0
        channel.fg_count = channel.motion_start_count + int(channel.motion_steps * progress)
        channel.last_edge = now
        if progress >= 1.0:
            error = abs(channel.motion_steps - (channel.fg_count - channel.motion_start_count))
            if error * 3600 > proto.TOLERANCE_DECI_DEG * 6 * max(1, channel.gear):
                channel.fault = int(proto.FaultCode.POSITION)
                channel.state = int(proto.MotionState.FAULT)
                channel.brake = True
                channel.duty = 0

    def _advance_probe(self, channel, now):
        # type: (FakeChannel, float) -> None
        """Walk the probe through its phases, one phase per evaluation.

        The firmware runs this on a 10 ms control loop; the simulator advances one
        phase per call, which is why tests step the clock several times instead of
        making one big jump. Pulse counts are taken from `selftest_pulses` at the
        phase boundary so a test can force PASSED, NO_PULSES or TOO_FEW_PULSES.
        """
        elapsed_ms = int((now - channel.probe_started) * 1000)
        # probe_time_scale shortens the 2000 ms-per-leg firmware timing so a four
        # channel batch can be tested without waiting ten real seconds. It changes
        # durations only, never the order of phases or the pass/fail rules.
        scale = max(0.001, float(self.probe_time_scale))
        power_ms = max(4, int(proto.SELFTEST_POWER_MS / scale))
        quarter = max(1, power_ms // 4)
        sign = -1 if channel.probe_direction else 1
        if channel.probe_phase in (int(proto.SelftestPhase.NONE), int(proto.SelftestPhase.OUTBOUND)):
            channel.probe_phase = int(proto.SelftestPhase.OUTBOUND)
            channel.probe_pulses = min(self.selftest_pulses, elapsed_ms // max(1, quarter // 2))
            channel.probe_powered_ms = min(power_ms, elapsed_ms)
            channel.duty = min(proto.SELFTEST_DUTY_MAX,
                               proto.SELFTEST_DUTY_STEP * max(1, elapsed_ms // 10))
            channel.measured_rpm = channel.rpm
            channel.last_edge = now
            if elapsed_ms >= quarter * 2:
                channel.probe_pulses = self.selftest_pulses
                channel.probe_outbound = self.selftest_pulses
                channel.fg_count += self.selftest_pulses * sign
                channel.probe_phase = int(proto.SelftestPhase.OUTBOUND_SETTLING)
        elif channel.probe_phase == int(proto.SelftestPhase.OUTBOUND_SETTLING):
            channel.duty = 0
            channel.measured_rpm = 0
            if elapsed_ms >= quarter * 3:
                channel.probe_phase = int(proto.SelftestPhase.RETURN)
        elif channel.probe_phase == int(proto.SelftestPhase.RETURN):
            channel.probe_return_pulses = min(
                channel.probe_outbound,
                channel.probe_outbound * max(0, elapsed_ms - quarter * 3) // max(1, quarter))
            channel.probe_return_ms = max(0, elapsed_ms - quarter * 3)
            channel.duty = 120
            channel.last_edge = now
            if elapsed_ms >= power_ms:
                channel.probe_return_pulses = channel.probe_outbound
                channel.fg_count -= channel.probe_return_pulses * sign
                channel.probe_phase = int(proto.SelftestPhase.RETURN_SETTLING)
        else:
            channel.probe_phase = int(proto.SelftestPhase.FINISHED)
            channel.probe_residual = channel.probe_outbound - channel.probe_return_pulses
            channel.probe_completed_ms = self._uptime_ms()
            channel.duty = 0
            channel.measured_rpm = 0
            channel.brake = True
            channel.last_edge = now
            if channel.probe_outbound >= proto.SELFTEST_TARGET_PULSES:
                channel.probe_state = int(proto.SelftestState.PASSED)
                channel.probe_reason = int(proto.SelftestReason.NONE)
            elif channel.probe_outbound <= 0:
                channel.probe_state = int(proto.SelftestState.FAILED)
                channel.probe_reason = int(proto.SelftestReason.NO_PULSES)
            else:
                channel.probe_state = int(proto.SelftestState.FAILED)
                channel.probe_reason = int(proto.SelftestReason.TOO_FEW_PULSES)
            # BSP/robot.c:130-139: a non-passing result costs the zero reference, a
            # FAILED probe also sets F_SELFTEST, and the rest of the batch is dropped
            # EXCEPT for the "no motor / weak feedback" discovery case in a scan-all
            # batch, which is an expected finding rather than a safety abort.
            if channel.probe_state != int(proto.SelftestState.PASSED):
                channel.valid = False
                if channel.probe_state == int(proto.SelftestState.FAILED):
                    channel.fault = int(proto.FaultCode.SELFTEST)
                discovery_failure = (self.probe_scan_all
                                     and channel.probe_state == int(proto.SelftestState.FAILED)
                                     and channel.probe_reason in
                                     (int(proto.SelftestReason.NO_PULSES),
                                      int(proto.SelftestReason.TOO_FEW_PULSES)))
                if not discovery_failure:
                    self.probe_aborting = True
                    self._drop_probe_queue(channel.probe_reason)
            if channel.fault:
                channel.state = int(proto.MotionState.FAULT)
            else:
                channel.state = (int(proto.MotionState.IDLE) if channel.valid
                                 else int(proto.MotionState.UNREADY))
            self._finish_probe_step(now)

    def _background_save(self, now):
        # type: (float) -> None
        """Firmware retries a pending save once the whole board is quiet."""
        if any(c.manual for c in self.channels):
            return
        if not (self.pending_save or self.dirty):
            return
        if self.storage_error or not self.board_quiet(now):
            return
        self._persist()

    def _persist(self):
        # type: () -> bool
        """Flash write that clears dirty and pending_save, or sets storage_error."""
        self.flash_writes += 1
        if self.storage_failure:
            self.storage_error = True
            return False
        self.dirty = False
        self.pending_save = False
        self.storage_error = False
        return True

    def _prewrite(self):
        # type: () -> bool
        """Flash write of a DIRTY record, as the firmware does when it accepts a
        MOVE or a self test: dirty stays 1 so an interrupted motion is visible at
        the next boot."""
        self.flash_writes += 1
        if self.storage_failure:
            self.storage_error = True
            return False
        self.pending_save = False
        return True

    def board_quiet(self, now=None):
        # type: (Optional[float]) -> bool
        now = self._now() if now is None else now
        for channel in self.channels:
            if channel.state in proto.BUSY_STATES:
                return False
            if (now - channel.last_edge) * 1000.0 < proto.QUIET_MS:
                return False
        return True

    def channel_quiet(self, channel, now=None):
        # type: (FakeChannel, Optional[float]) -> bool
        now = self._now() if now is None else now
        if channel.state in proto.BUSY_STATES:
            return False
        return (now - channel.last_edge) * 1000.0 >= proto.QUIET_MS

    # --------------------------------------------------------------- faults

    def inject_fault(self, channel, fault):
        # type: (int, int) -> None
        """Force a fault code, e.g. F_UNEXPECTED after a joint was turned by hand."""
        with self._lock:
            self.channels[channel].fault = int(fault)
            if self.channels[channel].manual:
                self._stop_channel(self.channels[channel])
            else:
                self.channels[channel].state = int(proto.MotionState.FAULT)
            self.injected_faults[channel] = int(fault)

    def inject_unexpected_pulses(self, channel, count):
        # type: (int, int) -> None
        """Accumulate stray edges using the applied gear and >10 degree threshold."""
        with self._lock:
            target = self.channels[channel]
            target.last_edge = self._now()
            if target.manual or (target.fault and target.fault != int(proto.FaultCode.UNEXPECTED_FG)):
                return
            limit = proto.unexpected_fg_budget(target.gear)
            activity = min(0xFFFFFFFF, target.unexpected_fg + max(0, int(count)))
            if limit == 0xFFFFFFFF:
                target.unexpected_fg = activity
                return
            if target.fault == int(proto.FaultCode.UNEXPECTED_FG):
                events, target.unexpected_fg = divmod(activity, limit + 1)
                if events:
                    previous = target.recovery_events
                    if not previous or previous + events >= 3:
                        target.recovery_started = self._now()
                    target.recovery_events = 1 + ((previous + events - 1) % 2 if previous else (events - 1) % 2)
            elif activity > limit:
                target.fault = int(proto.FaultCode.UNEXPECTED_FG)
                target.state = int(proto.MotionState.FAULT)
                target.valid = False; target.brake = True; target.duty = 0
                target.unexpected_fg = 0; target.recovery_events = 1; target.recovery_started = self._now()
                self.pending_save = True
            else:
                target.unexpected_fg = activity

    def reboot(self, keep_flash=True):
        # type: (bool) -> None
        """Power cycle. A dirty image at boot costs every channel its zero reference."""
        with self._lock:
            dirty_at_boot = self.dirty
            if self.pending_identity:
                # CONFIG staged a new transport/address/node; robot_init picks them
                # up now, which is exactly why the change needed a power cycle.
                self.protocol_value, self.address, self.node = self.pending_identity
                self.pending_identity = None
            self.cache.clear()
            del self.cache_order[:]
            self._boot_time = self._now()
            for channel in self.channels:
                channel.state = int(proto.MotionState.UNREADY)
                channel.brake = True
                channel.duty = 0
                channel.measured_rpm = 0
                channel.unexpected_fg = 0
                channel.manual = 0
                channel.recovery_events = 0
                channel.recovery_started = 0.0
                channel.stop_fault = False
                channel.motion_steps = 0
                channel.settle_to_idle = False
                channel.probe_state = int(proto.SelftestState.UNTESTED)
                channel.probe_reason = 0
                channel.probe_phase = int(proto.SelftestPhase.NONE)
                channel.probe_pulses = 0
                channel.probe_outbound = 0
                channel.probe_return_pulses = 0
                channel.probe_residual = 0
                channel.probe_sequence = 0
                channel.probe_completed_ms = 0
                channel.last_edge = self._boot_time
                if dirty_at_boot or not keep_flash:
                    channel.valid = False
                    channel.fg_count = 0
                    channel.target_deci_deg = 0
                if not keep_flash:
                    channel.fault = 0
            self.dirty = bool(dirty_at_boot)
            self.pending_save = False
            self.injected_faults.clear()
            self.probe_active = False
            self.probe_queue = []
            self.probe_current = -1
            self.probe_aborting = False
            self.probe_scan_all = False

    # ------------------------------------------------------------ registers

    def read_registers(self, address, count):
        # type: (int, int) -> Tuple[int, List[int]]
        """Return (business result, values); a non-zero result means an exception frame."""
        with self._lock:
            self._advance()
            if not 1 <= count <= 125 or address + count > 0x10000:
                return int(BusinessResult.VALUE), []
            values = []   # type: List[int]
            for offset in range(count):
                ok, value = self._read_one(address + offset)
                if not ok:
                    return int(BusinessResult.ADDRESS), []
                values.append(value)
            return int(BusinessResult.OK), values

    def _read_one(self, address):
        # type: (int) -> Tuple[bool, int]
        if address == proto.REG_SCHEMA:
            return True, self.schema
        if address == proto.REG_PROTOCOL:
            return True, self.protocol_value
        if address == proto.REG_ADDRESS:
            return True, self.address
        if address == proto.REG_NODE:
            return True, self.node
        if address == proto.REG_PWM_HZ:
            return True, self.pwm_hz
        if address == proto.REG_DIRTY:
            return True, 1 if self.dirty else 0
        if address == proto.REG_STORAGE_ERROR:
            return True, 1 if self.storage_error else 0
        if address == proto.REG_PENDING_SAVE:
            return True, 1 if self.pending_save else 0
        if proto.REG_UID <= address < proto.REG_UID + 6:
            # The 96 bit UID is three 32 bit words, each stored high word first, so
            # the six registers read C23B 3781 15B0 9208 3830 364A for the audited
            # board (BSP/robot.c:508, BSP/robot_port.c:221-223).
            words = [int(self.uid[i:i + 8], 16) for i in range(0, 24, 8)]
            registers = []   # type: List[int]
            for word in words:
                high, low = proto.pack_u32(word)
                registers.extend([high, low])
            return True, registers[address - proto.REG_UID]
        if address == proto.REG_HAVE_SEQ:
            return True, 1 if self.have_seq else 0
        if address == proto.REG_CAPABILITIES:
            return True, 3
        if address == proto.REG_COMMAND_PROTOCOL:
            return True, self.command_protocol
        if proto.REG_STAGED_BOARD <= address < proto.REG_STAGED_BOARD + 4:
            return True, self.staged_board[address - proto.REG_STAGED_BOARD]
        if proto.REG_COMMAND <= address < proto.REG_COMMAND + proto.REG_COMMAND_COUNT:
            return True, self.last_command_block[address - proto.REG_COMMAND]
        if address == proto.REG_COMMAND_TOMBSTONE:
            return True, 0
        if address == proto.REG_LAST_SEQUENCE:
            return True, self.last_seq
        if address == proto.REG_LAST_RESULT:
            return True, self.last_result
        if proto.CHANNEL_BASE_FIRST <= address < proto.CHANNEL_BASE_FIRST + 4 * proto.CHANNEL_STRIDE:
            delta = address - proto.CHANNEL_BASE_FIRST
            index, offset = divmod(delta, proto.CHANNEL_STRIDE)
            return self._read_channel(self.channels[index], offset)
        return False, 0

    def _read_channel(self, channel, offset):
        # type: (FakeChannel, int) -> Tuple[bool, int]
        if offset == proto.OFF_MANUAL_ADJUST:
            return True, channel.manual
        applied = channel.applied_config().to_registers()
        staged = list(channel.staged)
        if 0 <= offset <= 4:
            return True, applied[offset]
        if 8 <= offset <= 12:
            return True, staged[offset - 8]
        if offset == 0x10:
            return True, channel.state
        if offset == 0x11:
            return True, channel.fault
        if offset == 0x12:
            return True, 1 if channel.valid else 0
        if offset == 0x13:
            return True, 1 if channel.brake else 0
        if offset == 0x14:
            return True, 1 if channel.direction else 0
        if offset == 0x15:
            return True, channel.duty
        if offset == 0x16:
            return True, channel.measured_rpm
        if 0x18 <= offset <= 0x1D:
            values = (proto.pack_i32(channel.angle_deci_deg)
                      + proto.pack_i32(channel.fg_count)
                      + proto.pack_i32(channel.target_deci_deg))
            return True, values[offset - 0x18]
        if 0x20 <= offset <= 0x27:
            values = (channel.probe_state, channel.probe_reason) \
                + proto.pack_u32(channel.probe_pulses) \
                + (channel.probe_powered_ms,) \
                + proto.pack_u32(channel.probe_completed_ms) \
                + (channel.probe_sequence,)
            return True, values[offset - 0x20]
        if 0x28 <= offset <= 0x2F:
            values = (channel.probe_phase, channel.probe_return_ms) \
                + proto.pack_u32(channel.probe_outbound) \
                + proto.pack_u32(channel.probe_return_pulses) \
                + proto.pack_i32(channel.probe_residual)
            return True, values[offset - 0x28]
        return False, 0

    def write_registers(self, address, values):
        # type: (int, List[int]) -> int
        """Business result of a register write. The command block is handled separately."""
        with self._lock:
            self._advance()
            count = len(values)
            if not 1 <= count <= 123:
                return int(BusinessResult.VALUE)
            if address == proto.REG_STAGED_BOARD and count <= 4:
                for offset, value in enumerate(values):
                    self.staged_board[offset] = int(value) & 0xFFFF
                return int(BusinessResult.OK)
            if proto.REG_COMMAND <= address <= proto.REG_COMMAND_TOMBSTONE:
                # Only the atomic FC16 block is legal. Everything else, including the
                # legacy A55A commit register, is rejected with no side effects.
                return int(BusinessResult.ADDRESS)
            if proto.CHANNEL_BASE_FIRST <= address < proto.CHANNEL_BASE_FIRST + 4 * proto.CHANNEL_STRIDE:
                delta = address - proto.CHANNEL_BASE_FIRST
                index, offset = divmod(delta, proto.CHANNEL_STRIDE)
                if offset == proto.OFF_CONFIG_STAGED and count <= 5:
                    staged = list(self.channels[index].staged)
                    for position, value in enumerate(values):
                        staged[position] = int(value) & 0xFFFF
                    self.channels[index].staged = tuple(staged)
                    return int(BusinessResult.OK)
                return int(BusinessResult.ADDRESS)
            return int(BusinessResult.ADDRESS)

    # ------------------------------------------------------------- commands

    def execute_command(self, sequence, opcode, channel, argument):
        # type: (int, int, int, int) -> int
        """Run one command block write and return the firmware business result."""
        with self._lock:
            self._advance()
            self.commands_received += 1
            argument = int(argument)
            self.last_command_block = [sequence & 0xFFFF, opcode & 0xFFFF, channel & 0xFFFF] \
                + list(proto.pack_i32(argument))

            if sequence == 0:
                if not self.allow_auto_seq:
                    return int(BusinessResult.SEQUENCE)
                sequence = self._auto_sequence()

            cached = self.cache.get(sequence)
            if cached is not None:
                if cached[:3] == (opcode, channel, argument & 0xFFFFFFFF):
                    # Replay of an identical request: cached result, zero side effects.
                    # The board returns before touching 0x0036/0x0037 (BSP/robot.c:480-483),
                    # so those still describe whatever ran last.
                    self.commands_replayed += 1
                    return cached[3]
                # Same sequence, different parameters: also returned before the
                # last_seq/last_result update and before anything is cached
                # (BSP/robot.c:481). That is exactly what lets a client tell
                # "sequence rejected, nothing executed" from "executed but refused".
                return int(BusinessResult.SEQUENCE)

            if self.have_seq and not proto.sequence_acceptable(sequence, self.last_seq):
                # Outside the half-range window: refused with no side effects at all.
                return int(BusinessResult.SEQUENCE)

            self._current_sequence = sequence
            result = self._dispatch(opcode, channel, argument)

            # Every accepted request consumes a sequence and is cached, including the
            # ones the business logic rejected.
            self.have_seq = True
            self.last_seq, self.last_result = sequence, result
            self._cache(sequence, opcode, channel, argument, result)
            return result

    def _auto_sequence(self):
        # type: () -> int
        candidate = proto.next_sequence(self.last_seq)
        for _ in range(proto.SEQUENCE_CACHE_ENTRIES):
            if candidate not in self.cache:
                break
            candidate = proto.next_sequence(candidate)
        return candidate

    def _cache(self, sequence, opcode, channel, argument, result):
        # type: (int, int, int, int, int) -> None
        if sequence not in self.cache:
            self.cache_order.append(sequence)
        self.cache[sequence] = (int(opcode), int(channel), int(argument) & 0xFFFFFFFF, int(result))
        while len(self.cache_order) > proto.SEQUENCE_CACHE_ENTRIES:
            oldest = self.cache_order.pop(0)
            self.cache.pop(oldest, None)

    def _dispatch(self, opcode, channel, argument):
        # type: (int, int, int) -> int
        if opcode == int(Opcode.STOP_ALL):
            self._stop_all()
            self.commands_executed += 1
            return int(BusinessResult.OK)
        if opcode == int(Opcode.MANUAL_ADJUST):
            if not 0 <= channel < proto.CHANNEL_COUNT:
                return int(BusinessResult.VALUE)
            result = self._op_manual_adjust(channel, argument)
            if result == int(BusinessResult.OK): self.commands_executed += 1
            return result
        if any(c.manual for c in self.channels) and opcode != int(Opcode.STOP) and not (opcode == int(Opcode.BRAKE) and argument == 1):
            return int(BusinessResult.BUSY)
        if not 1 <= opcode <= 11:
            return int(BusinessResult.VALUE)
        board_level = opcode in (int(Opcode.CONFIG), int(Opcode.SAVE), int(Opcode.SELFTEST_ALL))
        if not board_level and not 0 <= channel < proto.CHANNEL_COUNT:
            return int(BusinessResult.VALUE)
        handler = {
            int(Opcode.CONFIG): self._op_config,
            int(Opcode.ZERO): self._op_zero,
            int(Opcode.MOVE): self._op_move,
            int(Opcode.STOP): self._op_stop,
            int(Opcode.BRAKE): self._op_brake,
            int(Opcode.DIRECTION): self._op_direction,
            int(Opcode.SAVE): self._op_save,
            int(Opcode.CLEAR_FAULT): self._op_clear_fault,
            int(Opcode.SELFTEST): self._op_selftest,
            int(Opcode.SELFTEST_ALL): self._op_selftest_all,
        }[opcode]
        result = handler(channel, argument)
        if result == int(BusinessResult.OK):
            self.commands_executed += 1
        return result

    def _op_manual_adjust(self, channel, argument):
        target = self.channels[channel]
        if argument not in (0, 1): return int(BusinessResult.VALUE)
        if not argument:
            if target.manual == 1: self._stop_channel(target)
            return int(BusinessResult.OK)
        if target.manual == 1: return int(BusinessResult.OK)
        if self.stop_input or self.probe_active: return int(BusinessResult.BUSY)
        if any(c.stop_fault for c in self.channels): return int(BusinessResult.UNREADY)
        if self.storage_error: return int(BusinessResult.STORAGE)
        if not target.enabled or not target.gear or target.fault not in (0, int(proto.FaultCode.UNEXPECTED_FG)):
            return int(BusinessResult.UNREADY)
        now = self._now()
        if any(c.manual == 2 or (not c.manual and c.state in proto.BUSY_STATES) or (now-c.last_edge)*1000 < proto.QUIET_MS for c in self.channels):
            return int(BusinessResult.BUSY)
        if not any(c.manual for c in self.channels) and not self._prewrite(): return int(BusinessResult.STORAGE)
        self.dirty = True; target.valid = False; target.manual = 1
        target.fault = 0; target.unexpected_fg = 0; target.recovery_events = 0
        target.state = int(proto.MotionState.MANUAL); target.duty = 0; target.brake = False
        return int(BusinessResult.OK)

    def _op_config(self, channel, argument):
        # type: (int, int) -> int
        if not self.board_quiet():
            return int(BusinessResult.BUSY)
        staged_board = list(self.staged_board)
        if not proto.ADDRESS_MIN <= staged_board[1] <= proto.ADDRESS_MAX:
            return int(BusinessResult.VALUE)
        if not proto.NODE_MIN <= staged_board[2] <= proto.NODE_MAX:
            return int(BusinessResult.VALUE)
        if not proto.PWM_HZ_MIN <= staged_board[3] <= proto.PWM_HZ_MAX:
            return int(BusinessResult.VALUE)
        roles = []
        parsed = []
        for target in self.channels:
            values = target.staged_config()
            if values.gear not in proto.GEAR_RATIOS:
                return int(BusinessResult.VALUE)
            if values.inverted not in (0, 1):
                return int(BusinessResult.VALUE)
            if values.enabled:
                if not values.gear or not values.role:
                    return int(BusinessResult.VALUE)
                if not proto.RPM_MIN <= values.rpm <= proto.RPM_MAX:
                    return int(BusinessResult.VALUE)
            roles.append(values.role)
            parsed.append(values)
        try:
            proto.validate_roles_unique(roles)
        except ValueError:
            return int(BusinessResult.VALUE)
        # Only pwm_hz takes effect immediately (BSP/robot.c:406). protocol, address
        # and node are assigned once in robot_init (BSP/robot.c:69-70), so a CONFIG
        # that changes them leaves the ACTIVE registers at their old values until a
        # power cycle. Reading 0x0001..0x0003 right after such a CONFIG therefore
        # disagrees with 0x0020..0x0022 by design.
        self.pwm_hz = staged_board[3]
        self.pending_identity = [staged_board[0], staged_board[1], staged_board[2]]
        for target, values in zip(self.channels, parsed):
            changed = (values.gear != target.gear or values.role != target.role
                       or values.inverted != target.inverted)
            target.enabled, target.role = values.enabled, values.role
            target.gear, target.inverted, target.rpm = values.gear, values.inverted, values.rpm
            if changed:
                # Changing gear, role or inverted invalidates the stored zero and the
                # recorded self test for that channel.
                target.valid = False
                target.probe_state = int(proto.SelftestState.UNTESTED)
                target.probe_reason = 0
        self.dirty = True
        return int(BusinessResult.OK) if self._persist() else int(BusinessResult.STORAGE)

    def _op_zero(self, channel, argument):
        # type: (int, int) -> int
        target = self.channels[channel]
        if not self.board_quiet():
            return int(BusinessResult.BUSY)
        if not target.gear:
            return int(BusinessResult.UNREADY)
        target.fg_count = 0
        target.target_deci_deg = 0
        target.valid = True
        target.unexpected_fg = 0
        target.recovery_events = 0
        target.stop_fault = False
        target.fault = 0
        target.state = int(proto.MotionState.IDLE)
        # ZERO produces no FG edges, so it must not restart the board's quiet timer:
        # a CONFIG or MOVE right after a legitimate zero is not "busy".
        self.dirty = True
        return int(BusinessResult.OK) if self._persist() else int(BusinessResult.STORAGE)

    def _op_move(self, channel, argument):
        # type: (int, int) -> int
        target = self.channels[channel]
        if self.stop_input:
            return int(BusinessResult.BUSY)
        if not -proto.MAX_MOVE_DECI_DEG <= argument <= proto.MAX_MOVE_DECI_DEG:
            return int(BusinessResult.VALUE)
        if not target.enabled or not target.gear:
            return int(BusinessResult.UNREADY)
        if self.storage_error:
            return int(BusinessResult.STORAGE)
        if target.fault or target.stop_fault:
            return int(BusinessResult.UNREADY)
        if target.state in proto.BUSY_STATES or not self.board_quiet():
            return int(BusinessResult.BUSY)
        steps = proto.deci_deg_to_steps(argument, target.gear)
        now = self._now()
        if not self.dirty:
            # The firmware pre-writes a dirty record when it ACCEPTS a move, so an
            # interrupted motion is detectable at the next boot.
            self.dirty = True
            if not self._prewrite():
                return int(BusinessResult.STORAGE)
        target.target_deci_deg = target.angle_deci_deg + argument
        target.motion_start_count = target.fg_count
        target.motion_steps = steps
        target.motion_started = now
        target.settle_to_idle = False
        pulses_per_second = max(1.0, target.rpm / 10.0)
        target.motion_duration = (abs(steps) / pulses_per_second) * self.motion_duration_scale
        # MOVE derives the direction output itself; an explicit DIRECTION command is
        # overwritten here (BSP/robot.c:458).
        target.direction = bool((argument < 0) ^ bool(target.inverted))
        target.brake = False
        target.state = int(proto.MotionState.RUNNING)
        target.last_edge = now
        return int(BusinessResult.OK)

    def _op_stop(self, channel, argument):
        # type: (int, int) -> int
        self._stop_channel(self.channels[channel])
        return int(BusinessResult.OK)

    def _stop_all(self):
        # type: () -> None
        for target in self.channels:
            self._stop_channel(target)

    def _stop_channel(self, target):
        # type: (FakeChannel) -> None
        now = self._now()
        if target.manual:
            if target.manual == 1: target.last_edge = now
            target.manual = 2; target.valid = False; target.duty = 0; target.brake = True
            target.state = int(proto.MotionState.SETTLING)
            return
        was_moving = target.state in proto.BUSY_STATES
        if self.probe_active and self.probe_current == target.index:
            # Cancelling the channel that is being probed cancels the whole batch:
            # the firmware drops the remaining queue (BSP/robot.c:137-138, 146).
            target.probe_state = int(proto.SelftestState.CANCELLED)
            target.probe_reason = int(proto.SelftestReason.CANCELLED)
            target.probe_phase = int(proto.SelftestPhase.FINISHED)
            target.probe_completed_ms = self._uptime_ms()
            target.valid = False
            self.probe_aborting = True
            self._drop_probe_queue(int(proto.SelftestReason.CANCELLED))
            self.probe_active = False
            self.probe_current = -1
        elif target.state == int(proto.MotionState.SELFTEST):
            target.probe_state = int(proto.SelftestState.CANCELLED)
            target.probe_reason = int(proto.SelftestReason.CANCELLED)
            target.probe_phase = int(proto.SelftestPhase.FINISHED)
        if was_moving:
            target.motion_settle_until = now + proto.QUIET_MS / 1000.0
            target.settle_to_idle = True
            target.state = int(proto.MotionState.SETTLING)
        else:
            target.state = (int(proto.MotionState.IDLE) if target.valid
                            else int(proto.MotionState.UNREADY))
        target.duty = 0
        target.measured_rpm = 0
        target.brake = True
        target.last_edge = now

    def _op_brake(self, channel, argument):
        # type: (int, int) -> int
        target = self.channels[channel]
        if argument not in (0, 1):
            return int(BusinessResult.VALUE)
        if argument == 1:
            self._stop_channel(target)
            target.brake = True
            return int(BusinessResult.OK)
        if self.stop_input or target.stop_fault:
            return int(BusinessResult.BUSY)
        if target.state in proto.BUSY_STATES or target.fault:
            return int(BusinessResult.BUSY)
        target.brake = False
        return int(BusinessResult.OK)

    def _op_direction(self, channel, argument):
        # type: (int, int) -> int
        target = self.channels[channel]
        if argument not in (0, 1):
            return int(BusinessResult.VALUE)
        if self.stop_input or target.stop_fault:
            return int(BusinessResult.BUSY)
        if not self.channel_quiet(target):
            return int(BusinessResult.BUSY)
        target.direction = bool(argument)
        return int(BusinessResult.OK)

    def _op_save(self, channel, argument):
        # type: (int, int) -> int
        if not self.board_quiet():
            return int(BusinessResult.BUSY)
        self.dirty = True
        return int(BusinessResult.OK) if self._persist() else int(BusinessResult.STORAGE)

    def _op_clear_fault(self, channel, argument):
        # type: (int, int) -> int
        target = self.channels[channel]
        if not self.board_quiet():
            return int(BusinessResult.BUSY)
        target.fault = 0
        target.unexpected_fg = 0
        target.recovery_events = 0
        target.stop_fault = False
        # CLEAR_FAULT does NOT restore valid: a lost zero still needs SetZero.
        if target.state == int(proto.MotionState.FAULT):
            target.state = (int(proto.MotionState.IDLE) if target.valid
                            else int(proto.MotionState.UNREADY))
        self.injected_faults.pop(channel, None)
        return int(BusinessResult.OK)

    def _op_selftest(self, channel, argument):
        # type: (int, int) -> int
        if argument not in (0, 1):
            return int(BusinessResult.VALUE)
        return self._start_probe([self.channels[channel]], argument, scan_all=False)

    def _op_selftest_all(self, channel, argument):
        # type: (int, int) -> int
        if argument not in (0, 1):
            return int(BusinessResult.VALUE)
        # The channel field is ignored for the batch form and may be out of range.
        return self._start_probe(list(self.channels), argument, scan_all=True)

    def _start_probe(self, targets, argument, scan_all=False):
        # type: (List[FakeChannel], int, bool) -> int
        """Mirror BSP/robot.c:163-191 plus the sequential queue at 294-296."""
        if self.stop_input:
            return int(BusinessResult.BUSY)
        if not self.board_quiet():
            return int(BusinessResult.BUSY)
        if self.storage_error:
            return int(BusinessResult.STORAGE)
        mask = set()
        for target in self.channels:
            if target.fault:
                # Any faulted channel blocks the whole board's self test.
                return int(BusinessResult.UNREADY)
            if scan_all or (targets and target is targets[0] and target.enabled and target.gear):
                mask.add(target.index)
        if not mask:
            return int(BusinessResult.UNREADY)
        now = self._now()
        sequence = self._current_sequence
        reset_scope = list(range(proto.CHANNEL_COUNT)) if scan_all else [t.index for t in targets]
        for index in reset_scope:
            record = self.channels[index]
            record.probe_state = (int(proto.SelftestState.QUEUED) if index in mask
                                  else int(proto.SelftestState.SKIPPED))
            record.probe_reason = 0
            record.probe_phase = int(proto.SelftestPhase.NONE)
            record.probe_pulses = 0
            record.probe_powered_ms = 0
            record.probe_return_ms = 0
            record.probe_outbound = 0
            record.probe_return_pulses = 0
            record.probe_residual = 0
            record.probe_sequence = sequence
            if record.probe_state == int(proto.SelftestState.SKIPPED):
                record.probe_completed_ms = self._uptime_ms()
        self.probe_scan_all = bool(scan_all)
        self.probe_aborting = False
        self.probe_direction = int(argument)
        self.probe_queue = sorted(mask)
        self.dirty = True
        if not self._prewrite():
            # Firmware marks every selected channel failed on a storage error and moves
            # nothing (BSP/robot.c:185-190).
            for index in mask:
                record = self.channels[index]
                record.probe_state = int(proto.SelftestState.FAILED)
                record.probe_reason = int(proto.SelftestReason.STORAGE_ERROR)
                record.probe_completed_ms = self._uptime_ms()
            self.probe_queue = []
            self.probe_active = False
            return int(BusinessResult.STORAGE)
        self.probe_active = True
        self._begin_probe(self.probe_queue.pop(0), now)
        return int(BusinessResult.OK)

    def _begin_probe(self, index, now):
        # type: (int, float) -> None
        target = self.channels[index]
        self.probe_current = index
        target.probe_started = now
        target.probe_direction = self.probe_direction
        target.probe_state = int(proto.SelftestState.RUNNING)
        target.probe_phase = int(proto.SelftestPhase.OUTBOUND)
        target.probe_pulses = 0
        target.probe_powered_ms = 0
        target.probe_return_ms = 0
        target.probe_outbound = 0
        target.probe_return_pulses = 0
        target.state = int(proto.MotionState.SELFTEST)
        target.brake = False
        target.direction = bool(self.probe_direction ^ bool(target.inverted))
        target.last_edge = now

    def _drop_probe_queue(self, reason):
        # type: (int) -> None
        """BSP/robot.c:100-107: queued-but-never-energised channels become UNTESTED.

        That is why a batch must not be judged by "every channel reached a final
        state": UNTESTED is not final, and reading it as a pass would be wrong.
        """
        for index in list(self.probe_queue):
            record = self.channels[index]
            record.probe_state = int(proto.SelftestState.UNTESTED)
            record.probe_reason = int(reason)
            record.probe_completed_ms = self._uptime_ms()
        self.probe_queue = []

    def _finish_probe_step(self, now):
        # type: (float) -> None
        if self.probe_aborting or not self.probe_queue:
            self.probe_active = False
            self.probe_current = -1
            self.pending_save = True
        else:
            self._begin_probe(self.probe_queue.pop(0), now)

    # ------------------------------------------------------------------ ADU

    def handle_adu(self, frame):
        # type: (bytes) -> Optional[bytes]
        """Process one complete ADU. Returns the reply ADU, or None for silence."""
        with self._lock:
            if len(frame) < 4 or not proto.frame_crc_ok(frame):
                self.silent_drops += 1
                return None
            if frame[0] != self.address:
                self.silent_drops += 1
                return None
            pdu = frame[1:-2]
            func = pdu[0]
            if func == 3:
                if len(pdu) != 5:
                    return self._exception(func, ModbusException.ILLEGAL_DATA_VALUE)
                address = (pdu[1] << 8) | pdu[2]
                count = (pdu[3] << 8) | pdu[4]
                result, values = self.read_registers(address, count)
                if result:
                    return self._exception(func, proto.RESULT_TO_EXCEPTION.get(
                        result, ModbusException.ILLEGAL_DATA_ADDRESS))
                payload = bytes(bytearray([3, len(values) * 2])) \
                    + struct.pack(">%dH" % len(values), *values)
                return proto.build_adu(self.address, payload)
            if func == 6:
                if len(pdu) != 5:
                    return self._exception(func, ModbusException.ILLEGAL_DATA_VALUE)
                address = (pdu[1] << 8) | pdu[2]
                value = (pdu[3] << 8) | pdu[4]
                result = self.write_registers(address, [value])
                if result:
                    return self._exception(func, proto.RESULT_TO_EXCEPTION.get(
                        result, ModbusException.ILLEGAL_DATA_ADDRESS))
                return proto.build_adu(self.address, pdu)
            if func == 16:
                if len(pdu) < 6 or pdu[5] != len(pdu) - 6:
                    return self._exception(func, ModbusException.ILLEGAL_DATA_VALUE)
                address = (pdu[1] << 8) | pdu[2]
                count = (pdu[3] << 8) | pdu[4]
                if count * 2 != pdu[5]:
                    return self._exception(func, ModbusException.ILLEGAL_DATA_VALUE)
                raw = pdu[6:]
                values = [(raw[i] << 8) | raw[i + 1] for i in range(0, len(raw), 2)]
                if address == proto.REG_COMMAND and count == proto.REG_COMMAND_COUNT:
                    argument = proto.unpack_i32(values[3], values[4])
                    result = self.execute_command(values[0], values[1], values[2], argument)
                    if result:
                        return self._exception(func, proto.RESULT_TO_EXCEPTION.get(
                            result, ModbusException.SLAVE_DEVICE_FAILURE))
                    # Success echoes the first six bytes of the REQUEST ADU (slave,
                    # function, address, quantity) plus CRC: eight bytes in total,
                    # with no byte-count field (BSP/robot_protocol.c:35).
                    return proto.build_adu(self.address, pdu[:5])
                result = self.write_registers(address, values)
                if result:
                    return self._exception(func, proto.RESULT_TO_EXCEPTION.get(
                        result, ModbusException.ILLEGAL_DATA_ADDRESS))
                return proto.build_adu(self.address, pdu[:5])
            return self._exception(func, ModbusException.ILLEGAL_FUNCTION)

    def _exception(self, func, code):
        # type: (int, int) -> bytes
        return proto.build_adu(self.address, bytes(bytearray([func | 0x80, int(code)])))


def complete_request_length(buffer):
    # type: (bytes) -> Optional[int]
    """Length of a complete request ADU implied by the bytes so far, or None."""
    if len(buffer) < 2:
        return None
    func = buffer[1]
    if func in (3, 6):
        return 8
    if func == 16:
        if len(buffer) < 7:
            return None
        return 9 + buffer[6]
    return None


def _is_command_frame(frame):
    # type: (bytes) -> bool
    """True for the single-frame command: FC16, address 0x0030, count 5."""
    return (len(frame) >= 6 and frame[1] == 16
            and ((frame[2] << 8) | frame[3]) == proto.REG_COMMAND
            and ((frame[4] << 8) | frame[5]) == proto.REG_COMMAND_COUNT)


class SerialStub(object):
    """pyserial-shaped loopback around a FakeBoard, with fault injection.

    Covers the awkward cases a real bus produces: a reply that arrives one byte at
    a time, a corrupted CRC, a vanished reply, leftover garbage from a previous
    transaction, and a request split across several write calls.
    """

    def __init__(self, board, clock=None):
        # type: (FakeBoard, Optional[Callable[[], float]]) -> None
        self.board = board
        self._clock = clock or time.monotonic
        self._tx = bytearray()
        self._rx = bytearray()
        self.is_open = True
        self.port = "fake://board"
        #: injection knobs
        self.drop_responses = 0
        self.drop_command_responses = 0   # drop only FC16 command-block replies
        self.corrupt_crc = 0
        self.fragment_size = 0          # 0 = deliver the whole reply in one read
        self.preload_garbage = b""
        self.leave_stale_half_frame = False
        self.response_delay = 0.0
        #: observed traffic
        self.writes = []      # type: List[bytes]
        self._previous_response = b""
        self._response_deadline = None   # type: Optional[float]

    def write(self, data):
        # type: (bytes) -> int
        if not self.is_open:
            raise RuntimeError("port is closed")
        self.writes.append(bytes(data))
        self._tx.extend(data)
        needed = complete_request_length(bytes(self._tx))
        if needed is None or len(self._tx) < needed:
            return len(data)   # request still arriving; wait for the rest
        frame = bytes(self._tx[:needed])
        del self._tx[:needed]
        response = self.board.handle_adu(frame)
        if self.leave_stale_half_frame and self._previous_response:
            # A stale half frame, exactly what a previous timeout leaves behind.
            stale = self._previous_response[:max(1, len(self._previous_response) // 2)]
            self._rx.extend(stale)
        if response is None:
            self._previous_response = b""
            return len(data)
        if self.drop_command_responses > 0 and _is_command_frame(frame):
            # Drop only the reply to a command block, so a test can simulate "the
            # board took it but the answer never came back" without also losing the
            # reads that surround it.
            self.drop_command_responses -= 1
            self._previous_response = response
            return len(data)
        if self.drop_responses > 0:
            self.drop_responses -= 1
            self._previous_response = response
            return len(data)
        if self.corrupt_crc > 0:
            self.corrupt_crc -= 1
            corrupted = bytearray(response)
            corrupted[-1] ^= 0xFF
            response = bytes(corrupted)
        if self.preload_garbage:
            self._rx.extend(self.preload_garbage)
            self.preload_garbage = b""
        if self.response_delay:
            self._response_deadline = self._clock() + self.response_delay
        self._previous_response = response
        self._rx.extend(response)
        return len(data)

    def read(self, count=1):
        # type: (int) -> bytes
        if not self.is_open:
            raise RuntimeError("port is closed")
        if self._response_deadline is not None:
            if self._clock() < self._response_deadline:
                return b""
            self._response_deadline = None
        if not self._rx:
            return b""
        chunk = self.fragment_size if self.fragment_size else len(self._rx)
        take = max(1, min(count, chunk, len(self._rx)))
        data = bytes(self._rx[:take])
        del self._rx[:take]
        return data

    def reset_input_buffer(self):
        # type: () -> None
        self._rx = bytearray()

    def flush(self):
        # type: () -> None
        return None

    def close(self):
        # type: () -> None
        self.is_open = False

    @property
    def in_waiting(self):
        # type: () -> int
        return len(self._rx)


def make_fake_stack(uid=DEFAULT_UID, read_only=False, expected_uid=None, clock=None,
                    channels=None, identify=True, response_timeout=0.25, **board_kwargs):
    # type: (...) -> Tuple[object, FakeBoard, SerialStub, object, FakeClock]
    """Build (transport, board, stub, session, clock) wired together, offline.

    The returned transport is the REAL ModbusRtuTransport talking to the stub, so
    tests exercise production framing code. `strict_ownership` stays off because
    unit tests legitimately call the transport from several threads; the ROS driver
    enables it.
    """
    from .board_session import BoardSession
    from .transport_modbus import ModbusRtuTransport

    fake_clock = clock or FakeClock()
    board = FakeBoard(uid=uid, clock=fake_clock, channels=channels, **board_kwargs)
    stub = SerialStub(board, clock=fake_clock)

    def factory(*args, **kwargs):
        # The stub is reused across close()/open() so that a session reconnect can be
        # exercised offline; a real serial factory would create a fresh object.
        stub.is_open = True
        return stub

    transport = ModbusRtuTransport(port=stub.port, address=board.address,
                                   response_timeout=response_timeout,
                                   serial_factory=factory,
                                   clock=fake_clock, strict_ownership=False)
    transport.open()
    session = BoardSession(transport, expected_uid=expected_uid if expected_uid else uid,
                           read_only=read_only, clock=fake_clock)
    if identify:
        session.identify()
    return transport, board, stub, session, fake_clock
