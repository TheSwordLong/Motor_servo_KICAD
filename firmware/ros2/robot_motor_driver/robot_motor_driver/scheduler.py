"""Single-owner board worker: joint locks, STOP priority, bounded waits.

Every byte that reaches the serial port goes through this worker thread. That is
the plan's "one worker thread or async scheduler owns each serial port" rule made
literal: the ROS node never touches the transport directly, it submits tasks and
reads the published snapshot.

Behaviour that matters for safety and for honesty:

* STOP preempts QUEUED work. It cannot preempt a half-sent frame, and it does not
  pretend to: `stop_confirmed` is only set after the board reports the channel
  settled with duty 0.
* A conflicting motion on a joint that is already moving is rejected immediately
  with DRIVER_BUSY instead of being queued behind itself.
* A MOVE timeout never invents a new sequence. The worker reconciles through
  0x0036/0x0037 and, when that is inconclusive, reports UNKNOWN and halts all
  further motion scheduling until an operator acknowledges it.
* Reconnecting does not resume anything. `BoardSession.reconnect` marks the state
  UNKNOWN because the board may have rebooted, which clears its dedupe cache.
* All waits are bounded by a deadline, so no task can wedge the worker forever.
"""

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from . import protocol as proto
from .board_session import (DRIVER_BAD_REQUEST, DRIVER_BUSY, DRIVER_CONFIG_MISMATCH, DRIVER_HALTED, DRIVER_LIMIT,
                            DRIVER_LIMITS_UNCONFIRMED, DRIVER_NOT_CONNECTED, DRIVER_NO_OP,
                            DRIVER_OK, DRIVER_READONLY, DRIVER_RPM_MISMATCH, DRIVER_STALE_POSITION,
                            DRIVER_TIMEOUT, DRIVER_UNKNOWN, DRIVER_VALID_REQUIRED, BoardSession,
                            BoardSnapshot, CommandOutcome)

#: Task priorities. Lower runs first; STOP is always at the head of the queue.
PRIORITY_STOP = 0
PRIORITY_MOTION = 20
PRIORITY_MAINTENANCE = 30
#: Forced snapshot refreshes are pure reads and must never overtake a stop.
PRIORITY_POLL = 40

#: MoveJoint modes, mirrored from the action definition.
MOVE_RELATIVE = 0
MOVE_ABSOLUTE = 1

#: Cadence of the targeted status read while a motion is in flight.
MOTION_POLL_INTERVAL = 0.05
#: How long to wait for the board to confirm a stop after sending one.
STOP_CONFIRM_TIMEOUT = 2.0


@dataclass
class TaskRequest(object):
    """One unit of work submitted to the worker."""

    kind: str                                   # move, selftest, stop, zero, brake,
                                                # clear_fault, save, config, poll
    channel: int = 0
    opcode: int = 0
    argument: int = 0
    timeout_sec: float = 30.0
    ros_target_id: str = ""
    joint: str = ""
    scope_all: bool = False
    require_valid: bool = False
    staged_configs: Optional[List[proto.ChannelConfigValues]] = None
    staged_board: Optional[List[int]] = None
    commit: bool = True
    feedback: Optional[Callable[[dict], None]] = None
    #: 0 = relative (argument is the delta), 1 = absolute (target_deci_deg is the
    #: goal). An absolute goal's delta is computed by the WORKER from a fresh
    #: reading so two goals cannot both measure the same start position.
    mode: int = 0
    target_deci_deg: Optional[int] = None
    #: Soft limits in radians, or None when the mechanical limits are unconfirmed.
    limits_rad: Optional[Tuple[float, float]] = None
    #: When True a null limits_rad is tolerated (commissioning/debug mode) and the
    #: driver states plainly that no soft-limit check was possible.
    allow_without_limits: bool = False
    #: Hard ceiling on one command while commissioning, in 0.1 deg units.
    max_step_deci_deg: Optional[int] = None
    #: Requested pre-gear rpm; 0 means "use whatever the board has stored".
    expect_rpm: int = 0
    #: Allow a RELATIVE move while position_valid is false. Enforced by the worker
    #: against a fresh reading, not by the caller.
    allow_unreferenced: bool = False


@dataclass
class TaskResult(object):
    """Everything the caller is allowed to claim about one task."""

    accepted: bool = False
    completed: bool = False        # motion/self test reached a terminal board state
    persisted: bool = False        # dirty, storage_error and pending_save all zero
    direction: bool = False
    direction_confirmed: bool = False
    flags_valid: bool = False
    dirty: bool = False
    storage_error: bool = False
    pending_save: bool = False
    position_valid: bool = False
    stop_confirmed: bool = False
    unknown: bool = False
    cancelled: bool = False
    result_code: int = DRIVER_OK
    sequence: int = 0
    message: str = ""
    state: int = 0
    fault: int = 0
    final_angle_deci_deg: int = 0
    target_angle_deci_deg: int = 0
    final_angle_rad: float = 0.0
    target_angle_rad: float = 0.0
    measured_rpm: int = 0
    selftest: Optional[proto.SelftestRecord] = None
    detail: str = ""
    #: Batch self test only: the probe session ran to the end of the queue. This is
    #: NOT the same as every channel passing.
    batch_completed: bool = False
    #: Batch self test only: every channel reported PASSED.
    all_passed: bool = False
    #: Self test only: the probe passed. For a batch this mirrors all_passed.
    passed: bool = False
    #: Per-channel self test results, in protocol channel order. Each entry is a dict
    #: with channel/executed/state/reason/phase/outbound_pulses/return_pulses/
    #: residual_pulses/powered_ms/return_ms/sequence/position_valid/fault.
    channel_results: List[dict] = field(default_factory=list)
    #: Channel the batch is on, for feedback reporting; -1 when not applicable.
    active_channel: int = -1
    #: Configuration transaction only: the staged and applied areas as read back from
    #: the board, in protocol channel order. Populated by the worker because only the
    #: worker may touch the bus.
    staged_configs: List[proto.ChannelConfigValues] = field(default_factory=list)
    applied_configs: List[proto.ChannelConfigValues] = field(default_factory=list)
    manual_mode: int = 0

    @property
    def result_name(self):
        # type: () -> str
        from .board_session import DRIVER_RESULT_NAMES
        if self.result_code in DRIVER_RESULT_NAMES:
            return DRIVER_RESULT_NAMES[self.result_code]
        return proto.business_result_name(self.result_code)


class TaskHandle(object):
    """Handle for a submitted task: wait, cancel, inspect."""

    def __init__(self, request):
        # type: (TaskRequest) -> None
        self.request = request
        self._done = threading.Event()
        self._cancel = threading.Event()
        self.result = TaskResult()
        self.rejected = False

    def wait(self, timeout=None):
        # type: (Optional[float]) -> bool
        return self._done.wait(timeout)

    def done(self):
        # type: () -> bool
        return self._done.is_set()

    def cancel(self):
        # type: () -> None
        """Ask for a stop. The worker sends STOP and waits for confirmation."""
        self._cancel.set()

    @property
    def cancel_requested(self):
        # type: () -> bool
        return self._cancel.is_set()

    def _resolve(self, result):
        # type: (TaskResult) -> None
        self.result = result
        self._done.set()

    def _reject(self, code, message):
        # type: (int, str) -> None
        self.rejected = True
        self._resolve(TaskResult(accepted=False, result_code=code, message=message))


class _QueueItem(object):
    """Priority queue payload; the counter keeps FIFO order inside a priority."""

    __slots__ = ("priority", "counter", "handle")

    def __init__(self, priority, counter, handle):
        # type: (int, int, TaskHandle) -> None
        self.priority = priority
        self.counter = counter
        self.handle = handle

    def __lt__(self, other):
        if self.priority != other.priority:
            return self.priority < other.priority
        return self.counter < other.counter


class BoardWorker(object):
    """Runs every board transaction on one thread."""

    def __init__(self, session, poll_interval=0.2, clock=None, logger=None,
                 reconnect_delay=1.0, allow_motion=False, allowed_channels=None, expected_configs=None, save_recovery_only=False):
        # type: (BoardSession, float, Optional[Callable[[], float]], Optional[Callable[[str, str], None]], float, bool) -> None
        self.session = session
        self.poll_interval = float(poll_interval)
        self._clock = clock or time.monotonic
        self._log = logger or (lambda level, message: None)
        self.reconnect_delay = float(reconnect_delay)
        self.allow_motion = bool(allow_motion)
        self.save_recovery_only = bool(save_recovery_only)
        self.allowed_channels = None if allowed_channels is None else frozenset(allowed_channels)
        self.expected_configs = dict(expected_configs or {})

        self._queue = queue.PriorityQueue()
        self._counter = 0
        self._thread = None      # type: Optional[threading.Thread]
        self._stopping = threading.Event()
        self._started = threading.Event()

        self._snapshot_lock = threading.RLock()
        self._snapshot = BoardSnapshot()
        self._snapshot_time = 0.0
        self._last_poll = 0.0
        #: Bumped on every successful refresh; lets refresh_now() prove the snapshot it
        #: hands back is the one that read just happened, not an old one.
        self._refresh_seq = 0

        self._active_lock = threading.Lock()
        self._active_joints = {}      # type: Dict[int, TaskHandle]
        self._pending_stops = set()   # type: set
        self._stop_all_requested = threading.Event()

        self.connected = False
        self.communication_error = ""
        self.reconnect_attempts = 0

    # ---------------------------------------------------------------- lifecycle

    def start(self):
        # type: () -> None
        if self._thread is not None:
            return
        self._stopping.clear()
        self._thread = threading.Thread(target=self._run, name="robot-board-worker", daemon=True)
        self._thread.start()
        self._started.wait(5.0)

    def stop(self, timeout=5.0, best_effort_stop_all=False):
        # type: (float, bool) -> bool
        """Stop the worker.

        `best_effort_stop_all` sends STOP_ALL before shutting down. It is only
        honoured when writes are allowed; in read-only mode the worker cannot stop
        the board and says so, because pretending otherwise would be worse than
        doing nothing. Software stop is never a substitute for a hardware e-stop.
        """
        if best_effort_stop_all and not self.session.read_only:
            try:
                handle = self.submit(TaskRequest(kind="stop", scope_all=True, timeout_sec=2.0,
                                                 ros_target_id="shutdown"))
                handle.wait(timeout)
                if not handle.result.stop_confirmed:
                    self._log("warn", "shutdown STOP_ALL was not confirmed by the board")
            except Exception as error:      # pragma: no cover - defensive
                self._log("warn", "shutdown STOP_ALL failed: %s" % error)
        elif best_effort_stop_all:
            self._log("warn", "read-only mode: shutdown STOP_ALL was suppressed, the board may "
                              "still be moving; use the hardware e-stop")
        self._stopping.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout)
            return not thread.is_alive()
        return True

    @property
    def running(self):
        # type: () -> bool
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------ snapshot

    @property
    def snapshot(self):
        # type: () -> BoardSnapshot
        with self._snapshot_lock:
            return self._snapshot

    def snapshot_age(self):
        # type: () -> float
        with self._snapshot_lock:
            if not self._snapshot_time:
                return float("inf")
            return max(0.0, self._clock() - self._snapshot_time)

    def refresh_snapshot(self, force=False):
        # type: (bool) -> BoardSnapshot
        """Full board poll. Called from the worker thread only."""
        now = self._clock()
        if not force and (now - self._last_poll) < self.poll_interval:
            return self.snapshot
        snapshot = self.session.poll()
        with self._snapshot_lock:
            self._snapshot = snapshot
            self._snapshot_time = now
            self._last_poll = now
        if snapshot.communication_ok:
            self.connected = True
            self.communication_error = ""
            with self._snapshot_lock:
                self._refresh_seq += 1
        else:
            self.connected = False
            self.communication_error = snapshot.error
        return snapshot

    def refresh_now(self, timeout=2.0):
        # type: (float) -> Optional[BoardSnapshot]
        """Return a snapshot produced by a poll that happened NOW, or None.

        Returning the previous snapshot on failure would be the dangerous option: a
        caller computing an absolute move delta from it would command the motor to a
        position derived from data that may be minutes old. Callers must treat None
        as "do not move".
        """
        with self._snapshot_lock:
            before = self._refresh_seq
        handle = self.submit(TaskRequest(kind="poll", timeout_sec=timeout))
        if handle.rejected:
            return None
        if not handle.wait(timeout):
            return None
        if handle.result.result_code != DRIVER_OK:
            return None
        with self._snapshot_lock:
            if self._refresh_seq == before:
                return None
            return self._snapshot

    # ------------------------------------------------------------------- submit

    def submit(self, request):
        # type: (TaskRequest) -> TaskHandle
        """Queue a task. Rejections are immediate and never reach the bus."""
        handle = TaskHandle(request)
        if self.save_recovery_only and request.kind not in ("poll", "stop", "retry_save"):
            handle._reject(DRIVER_READONLY, "SAVE recovery only refuses %s; no command sent" % request.kind)
            return handle
        restricted = (request.kind in ("move", "selftest", "zero") or
                      (request.kind == "brake" and request.argument == 0))
        if restricted and self.allowed_channels is not None:
            selected = set(range(proto.CHANNEL_COUNT)) if request.scope_all else {request.channel}
            if not selected.issubset(self.allowed_channels):
                handle._reject(DRIVER_READONLY, "joint is outside allowed_motion_joints; no command sent")
                return handle
        if request.kind not in ("stop", "poll") and self.session.read_only:
            # "poll" is a pure read and stays allowed; every write, STOP included,
            # is intercepted in read-only mode.
            handle._reject(DRIVER_READONLY,
                           "read-only mode refuses %s; every write is intercepted, including STOP"
                           % request.kind)
            return handle
        if request.kind == "stop" and self.session.read_only:
            handle._reject(DRIVER_READONLY,
                           "read-only mode refuses STOP: the driver cannot write to the board. "
                           "Use the hardware e-stop, or enable the maintenance entry.")
            return handle
        if self.session.unknown_command_state and request.kind not in ("stop", "poll"):
            # "stop" must stay possible: it is how an operator regains control.
            # "poll" is a pure read and is needed to confirm that stop, so blocking
            # it would leave the driver holding a stale snapshot with no way out.
            handle._reject(DRIVER_HALTED,
                           "driver halted after an UNKNOWN outcome (%s); acknowledge it first"
                           % self.session.unknown_detail)
            return handle
        if request.kind in ("move", "selftest") and not self.allow_motion:
            handle._reject(DRIVER_READONLY,
                           "%s is refused: allow_motion is false" % request.kind)
            return handle
        if not self._started.is_set():
            handle._reject(DRIVER_NOT_CONNECTED, "worker is not running")
            return handle
        with self._active_lock:
            if request.kind in ("retry_save", "direction") and self._active_joints:
                handle._reject(DRIVER_BUSY, "motion/manual entry is queued; no maintenance command sent")
                return handle
            if request.kind == "manual_adjust" and request.argument == 1:
                if self._active_joints:
                    handle._reject(DRIVER_BUSY, "motion or manual entry already queued; no command sent")
                    return handle
                self._active_joints[-1] = handle
            if request.kind in ("move", "selftest"):
                if -1 in self._active_joints or any(self.snapshot.manual_modes):
                    handle._reject(DRIVER_BUSY, "manual adjustment is active or queued")
                    return handle
                if request.channel in self._active_joints:
                    handle._reject(DRIVER_BUSY,
                                   "channel %d already has a motion in flight (%s); the firmware "
                                   "rejects a new MOVE on a busy channel"
                                   % (request.channel, self._active_joints[request.channel].request.kind))
                    return handle
                self._active_joints[request.channel] = handle
        priority = {"stop": PRIORITY_STOP, "poll": PRIORITY_POLL}.get(
            request.kind,
            PRIORITY_MOTION if request.kind in ("move", "selftest") else PRIORITY_MAINTENANCE)
        with self._active_lock:
            self._counter += 1
            counter = self._counter
        self._queue.put(_QueueItem(priority, counter, handle))
        return handle

    def request_stop(self, channel=None, ros_target_id=""):
        # type: (Optional[int], str) -> TaskHandle
        """Ask for an immediate stop of one channel, or of the whole board."""
        request = TaskRequest(kind="stop", scope_all=channel is None,
                              channel=0 if channel is None else int(channel),
                              timeout_sec=STOP_CONFIRM_TIMEOUT, ros_target_id=ros_target_id)
        with self._active_lock:
            for active in self._active_joints.values():
                if channel is None or active.request.scope_all or active.request.channel == int(channel):
                    active.cancel()
            if channel is None:
                self._stop_all_requested.set()
            else:
                self._pending_stops.add(int(channel))
        return self.submit(request)

    def release_joint(self, channel):
        # type: (int) -> None
        with self._active_lock:
            self._active_joints.pop(int(channel), None)

    # --------------------------------------------------------------------- loop

    def _run(self):
        # type: () -> None
        self._started.set()
        self._log("info", "board worker started (poll_interval=%.3f s)" % self.poll_interval)
        while not self._stopping.is_set():
            try:
                snapshot = self.refresh_snapshot()
                if not snapshot.communication_ok:
                    # session.poll() swallows transport errors into the snapshot, so
                    # a dead port would otherwise pass unnoticed: the worker would
                    # keep publishing stale data and never halt scheduling.
                    self._handle_communication_loss(snapshot.error or "no valid reply")
            except Exception as error:      # pragma: no cover - defensive
                self._log("error", "poll failed: %s" % error)
                self._handle_communication_loss(str(error))
            try:
                item = self._queue.get(timeout=self.poll_interval)
            except queue.Empty:
                continue
            handle = item.handle     # type: TaskHandle
            try:
                self._execute(handle)
            except Exception as error:      # pragma: no cover - defensive
                handle._resolve(TaskResult(accepted=False, result_code=DRIVER_UNKNOWN,
                                           unknown=True,
                                           message="worker error: %s" % error))
                self._log("error", "task %s failed: %s" % (handle.request.kind, error))
            finally:
                if handle.request.kind == "manual_adjust" and handle.request.argument == 1:
                    self.release_joint(-1)
                if handle.request.kind in ("move", "selftest"):
                    self.release_joint(handle.request.channel)
        self._log("info", "board worker stopped")

    def _handle_communication_loss(self, detail):
        # type: (str) -> None
        if not self.connected and self.session.unknown_command_state \
                and self.communication_error == detail:
            # Already reported and already halted; do not spam retries every poll.
            return
        self.connected = False
        self.communication_error = detail
        self.session.mark_unknown("communication lost: %s" % detail)
        if self.reconnect_attempts >= 3:
            return
        self.reconnect_attempts += 1
        self._log("warn", "communication lost (%s); reconnect attempt %d"
                  % (detail, self.reconnect_attempts))
        time.sleep(self.reconnect_delay)
        try:
            self.session.reconnect()
            self.session.identify()
            self.connected = True
            self.communication_error = ""
            self._log("warn", "reconnected and re-identified; no motion was resumed, state is "
                              "UNKNOWN until an operator acknowledges it")
        except Exception as error:
            self._log("error", "reconnect failed: %s" % error)

    # ----------------------------------------------------------------- dispatch

    def _execute(self, handle):
        # type: (TaskHandle) -> None
        kind = handle.request.kind
        if kind in ("move", "selftest") and handle.cancel_requested:
            handle._resolve(TaskResult(cancelled=True, message="cancelled before dispatch; no motion sent"))
            return
        handler = {
            "stop": self._task_stop,
            "move": self._task_move,
            "selftest": self._task_selftest,
            "manual_adjust": self._task_manual_adjust,
            "zero": self._task_simple_command,
            "brake": self._task_simple_command,
            "direction": self._task_direction,
            "clear_fault": self._task_simple_command,
            "save": self._task_simple_command,
            "retry_save": self._task_retry_save,
            "config": self._task_config,
            "poll": self._task_poll,
        }.get(kind)
        if handler is None:
            handle._resolve(TaskResult(result_code=DRIVER_UNKNOWN,
                                       message="unknown task kind %r" % kind))
            return
        handler(handle)

    # -- stop ------------------------------------------------------------------

    def _task_stop(self, handle):
        # type: (TaskHandle) -> None
        request = handle.request
        result = TaskResult()
        opcode = proto.Opcode.STOP_ALL if request.scope_all else proto.Opcode.STOP
        try:
            outcome = self.session.send_command(opcode, request.channel, 0,
                                                ros_target_id=request.ros_target_id,
                                                allow_while_unknown=True)
        except proto.ReadOnlyViolation as error:
            handle._resolve(TaskResult(result_code=DRIVER_READONLY, message=str(error)))
            return
        except proto.ProtocolError as error:
            # Refused or unreachable: the mechanism is NOT confirmed stopped, and no
            # previously reported UNKNOWN state is cleared by this failure.
            handle._resolve(TaskResult(result_code=DRIVER_UNKNOWN, unknown=True,
                                       stop_confirmed=False,
                                       message="stop not sent, NOT stopped: %s" % error))
            return
        result.sequence = outcome.sequence
        result.accepted = outcome.accepted
        result.result_code = outcome.result if not outcome.accepted else DRIVER_OK
        result.unknown = outcome.unknown
        result.message = outcome.detail
        if outcome.unknown:
            # A stop whose outcome is unknown must never be reported as stopped.
            result.stop_confirmed = False
            result.message = ("STOP outcome is UNKNOWN; the mechanism may still be moving. "
                              + outcome.detail)
            handle._resolve(result)
            return
        if not outcome.accepted:
            handle._resolve(result)
            return
        result.stop_confirmed = self._wait_stop_confirmed(request, result)
        if request.scope_all:
            self._stop_all_requested.clear()
        else:
            self._pending_stops.discard(request.channel)
        result.message = ("stop confirmed: channel settled with duty 0" if result.stop_confirmed
                          else "stop was accepted but NOT confirmed within %.1f s"
                               % request.timeout_sec)
        handle._resolve(result)

    def _wait_stop_confirmed(self, request, result):
        # type: (TaskRequest, TaskResult) -> bool
        deadline = self._clock() + max(0.1, min(request.timeout_sec, STOP_CONFIRM_TIMEOUT))
        channels = (list(range(proto.CHANNEL_COUNT)) if request.scope_all
                    else [request.channel])
        while self._clock() < deadline:
            settled = True
            for channel in channels:
                try:
                    status, _ = self.session.read_status_and_position(channel)
                except proto.ProtocolError as error:
                    result.message = "stop confirmation read failed: %s" % error
                    return False
                result.state = status.state
                result.fault = status.fault
                if status.state in proto.BUSY_STATES or status.duty_permille:
                    settled = False
            if settled:
                return True
            time.sleep(MOTION_POLL_INTERVAL)
        return False

    # -- motion ----------------------------------------------------------------

    def _task_move(self, handle):
        # type: (TaskHandle) -> None
        """One MOVE, decided entirely on the bus-owning thread.

        Reading the position, deriving an absolute goal's delta, checking the soft
        limits and the stored rpm, and sending the command all happen here in order
        while this task holds the joint slot. Doing the read in one step and the send
        in another (as the ROS side used to) lets two goals measure the same start
        position and both commit a movement from it.
        """
        request = handle.request
        channel = int(request.channel)
        try:
            config = self.session.read_channel_config(channel)
            status, position = self.session.read_status_and_position(channel)
        except proto.ProtocolError as error:
            # Refuse rather than moving from data of unknown age.
            handle._resolve(TaskResult(
                result_code=DRIVER_STALE_POSITION,
                message=("refused: no fresh position for channel %d (%s); the driver will not "
                         "compute a movement from a possibly stale snapshot" % (channel, error))))
            return
        result = TaskResult(state=status.state, fault=status.fault,
                           position_valid=status.valid)
        result.measured_rpm = status.rpm
        mismatch = self._config_mismatch(channel, config)
        if mismatch:
            handle._resolve(TaskResult(result_code=DRIVER_CONFIG_MISMATCH, message=mismatch))
            return

        if request.expect_rpm and int(request.expect_rpm) != int(config.rpm):
            handle._resolve(TaskResult(
                result_code=DRIVER_RPM_MISMATCH, state=status.state, fault=status.fault,
                position_valid=status.valid,
                message=("channel %d is configured for %d rpm but the goal asked for %d rpm, so "
                         "nothing was sent. MOVE carries only the angle (BSP/robot.c:334); to "
                         "change speed use a whole-board-quiet configuration transaction (this "
                         "board's apply_config service) first"
                         % (channel, config.rpm, request.expect_rpm))))
            return

        if request.mode == MOVE_ABSOLUTE:
            if request.target_deci_deg is None:
                handle._resolve(TaskResult(
                    result_code=DRIVER_BAD_REQUEST, state=status.state, fault=status.fault,
                    message="absolute mode requires target_deci_deg"))
                return
            if not status.valid:
                handle._resolve(TaskResult(
                    result_code=DRIVER_VALID_REQUIRED, state=status.state, fault=status.fault,
                    message=("absolute move refused: channel %d has position_valid=false, so there "
                             "is no trusted zero to measure from; the driver never zeroes "
                             "itself" % channel)))
                return
            deci = int(request.target_deci_deg) - int(position.angle_deci_deg)
        else:
            deci = int(request.argument)
            if not status.valid and not request.allow_unreferenced:
                handle._resolve(TaskResult(
                    result_code=DRIVER_VALID_REQUIRED, state=status.state, fault=status.fault,
                    position_valid=False,
                    message=("relative move refused: channel %d has position_valid=false and "
                             "allow_unreferenced_relative is false. Enable it explicitly to move "
                             "without a zero reference; the driver never zeroes automatically."
                             % channel)))
                return

        try:
            proto.validate_move_argument(deci)
        except ValueError as error:
            handle._resolve(TaskResult(
                result_code=DRIVER_LIMIT, state=status.state, fault=status.fault,
                position_valid=status.valid,
                message=("the goal needs %d (0.1 deg) from the current pose, which exceeds the "
                         "single-MOVE limit of +/-360 deg and is not split: %s" % (deci, error))))
            return

        if request.max_step_deci_deg is not None and abs(deci) > int(request.max_step_deci_deg):
            handle._resolve(TaskResult(
                result_code=DRIVER_LIMIT, state=status.state, fault=status.fault,
                position_valid=status.valid,
                message=("commissioning step %.1f deg exceeds the allowed %.1f deg per command; "
                         "nothing was sent" % (abs(deci) / 10.0, request.max_step_deci_deg / 10.0))))
            return

        if proto.quantises_to_zero(deci, config.gear):
            unconfigured = ("" if config.gear else
                            " Channel %d has gear 0 and enabled=%s, i.e. it is not configured for "
                            "motion at all; /robot/apply_config must set a legal gear and enable it "
                            "before any move can happen." % (channel, config.enabled))
            handle._resolve(TaskResult(
                result_code=DRIVER_NO_OP, state=status.state, fault=status.fault,
                position_valid=status.valid,
                final_angle_deci_deg=position.angle_deci_deg,
                target_angle_deci_deg=position.target_deci_deg,
                final_angle_rad=proto.deci_deg_to_rad(position.angle_deci_deg),
                message=("goal quantises to 0 steps at gear %d (argument %d = 0.1 deg); nothing was "
                         "sent and no new DONE is expected.%s" % (config.gear, deci, unconfigured))))
            return

        limit_note = self._check_soft_limits(request, config, status, position, deci)
        if limit_note is not None:
            code, message = limit_note
            handle._resolve(TaskResult(result_code=code, state=status.state, fault=status.fault,
                                       position_valid=status.valid, message=message))
            return

        request.argument = deci
        outcome = self._send(handle, request, result)
        if outcome is None:
            return
        if limit_note is None and request.limits_rad is None and request.allow_without_limits:
            result.detail = "no soft-limit check was possible (limits unconfirmed)"
        if not outcome.accepted:
            self._fill_state(request.channel, result)
            handle._resolve(result)
            return
        self._await_motion(handle, request, result,
                           terminal_states=(int(proto.MotionState.DONE),))

    def _check_soft_limits(self, request, config, status, position, deci):
        # type: (TaskRequest, proto.ChannelConfigValues, proto.ChannelStatus, proto.ChannelPosition, int) -> Optional[Tuple[int, str]]
        """Return (code, message) when the predicted endpoint is not acceptable.

        Applies to relative moves too: what matters mechanically is where the joint
        ends up, not how far it was told to travel. When the limits are unconfirmed,
        or the position is not trusted, the endpoint cannot be verified at all, and
        the driver says so instead of pretending the check passed.
        """
        channel = int(request.channel)
        end_deci = int(position.angle_deci_deg) + int(deci)
        if request.limits_rad is None:
            if request.allow_without_limits:
                return None
            return (DRIVER_LIMITS_UNCONFIRMED,
                    "channel %d has limits_rad null, meaning the mechanical limits are not "
                    "confirmed, so the driver cannot tell whether a move ending at %.1f deg is "
                    "safe. Record the limits, or start with commissioning:=true to accept "
                    "unbounded motion explicitly" % (channel, end_deci / 10.0))
        low, high = request.limits_rad
        if not status.valid:
            if request.allow_without_limits:
                return None
            return (DRIVER_LIMITS_UNCONFIRMED,
                    "channel %d has position_valid=false, so the predicted endpoint cannot be "
                    "checked against limits_rad; the driver will not claim it verified the "
                    "destination. Set a zero reference or use commissioning:=true" % channel)
        low_deci = proto.rad_to_deci_deg(low)
        high_deci = proto.rad_to_deci_deg(high)
        if low_deci > high_deci:
            low_deci, high_deci = high_deci, low_deci
        if not low_deci <= end_deci <= high_deci:
            return (DRIVER_LIMIT,
                    "predicted endpoint %.1f deg is outside limits_rad [%.1f, %.1f] for channel %d; "
                    "nothing was sent" % (end_deci / 10.0, low_deci / 10.0, high_deci / 10.0,
                                          channel))
        return None

    def _config_mismatch(self, channel, config):
        expected = self.expected_configs.get(channel)
        actual = (bool(config.enabled), int(config.role), int(config.gear), int(config.inverted))
        if expected is not None and tuple(expected) != actual:
            return "channel %d configuration mismatch: expected %s, board %s; no command sent" % (channel, expected, actual)
        return ""

    def _task_selftest(self, handle):
        # type: (TaskHandle) -> None
        request = handle.request
        result = TaskResult()
        outcome = self._send(handle, request, result)
        if outcome is None:
            return
        if not outcome.accepted:
            # Even a refused batch leaves records behind: on a storage failure the
            # firmware marks every selected channel FAILED with reason storage_error
            # (BSP/robot.c:185-190), and the caller is entitled to see that.
            channels = (list(range(proto.CHANNEL_COUNT)) if request.scope_all
                        else [int(request.channel)])
            self._collect_selftest(result, channels, int(outcome.sequence))
            note = self._annotate_selftest_channels(result, int(outcome.sequence))
            if note:
                result.message += "; " + note
            self._fill_state(request.channel, result)
            handle._resolve(result)
            return
        self._await_selftest(handle, request, result)

    def _send(self, handle, request, result):
        # type: (TaskHandle, TaskRequest, TaskResult) -> Optional[CommandOutcome]
        if handle.cancel_requested and request.kind in ("move", "selftest"):
            handle._resolve(TaskResult(cancelled=True, message="cancelled before transmission; no motion sent"))
            return None
        try:
            if request.kind == "manual_adjust" and not self.session.capabilities & proto.CAP_MANUAL_ADJUST:
                handle._resolve(TaskResult(result_code=int(proto.BusinessResult.UNREADY),
                                           message="firmware does not support manual adjustment; no command sent"))
                return None
            if request.kind in ("move", "selftest") and self.session.capabilities & proto.CAP_MANUAL_ADJUST:
                modes = [self.session.transport.read_registers(proto.channel_base(ch) + proto.OFF_MANUAL_ADJUST, 1)[0]
                         for ch in range(proto.CHANNEL_COUNT)]
                if any(modes):
                    handle._resolve(TaskResult(result_code=DRIVER_BUSY, message="manual/exit-wait channel blocks board motion"))
                    return None
            if request.kind in ("selftest", "direction") or (request.kind == "brake" and request.argument == 0):
                channels = range(proto.CHANNEL_COUNT) if request.scope_all else [request.channel]
                for channel in channels:
                    mismatch = self._config_mismatch(channel, self.session.read_channel_config(channel))
                    if mismatch:
                        handle._resolve(TaskResult(result_code=DRIVER_CONFIG_MISMATCH, message=mismatch))
                        return None
            outcome = self.session.send_command(request.opcode, request.channel, request.argument,
                                                ros_target_id=request.ros_target_id)
        except proto.ReadOnlyViolation as error:
            handle._resolve(TaskResult(result_code=DRIVER_READONLY, message=str(error)))
            return None
        except proto.ProtocolError as error:
            handle._resolve(TaskResult(result_code=DRIVER_UNKNOWN, unknown=True,
                                       message="%s failed: %s" % (request.kind, error)))
            return None
        result.sequence = outcome.sequence
        result.accepted = outcome.accepted
        result.unknown = outcome.unknown
        result.detail = outcome.detail
        if not outcome.accepted:
            result.result_code = outcome.result
            result.message = "%s refused by the board: %s (%s)" % (
                request.kind, outcome.detail, outcome.result_name)
        else:
            result.result_code = DRIVER_OK
        return outcome

    def _task_manual_adjust(self, handle):
        request = handle.request
        if handle.cancel_requested:
            handle._resolve(TaskResult(cancelled=True, message="manual entry cancelled before dispatch"))
            return
        request.opcode = int(proto.Opcode.MANUAL_ADJUST)
        result = TaskResult()
        outcome = self._send(handle, request, result)
        if outcome is None:
            return
        if not outcome.accepted:
            handle._resolve(result)
            return
        deadline = self._clock() + request.timeout_sec
        while True:
            snapshot = self.refresh_snapshot(force=True)
            if not snapshot.communication_ok:
                result.unknown = True
                result.result_code = DRIVER_UNKNOWN
                result.message = "accepted but read-back failed; use STOP to regain control"
                break
            result.manual_mode = snapshot.manual_modes[request.channel]
            result.position_valid = snapshot.channel_status(request.channel).valid
            try:
                result.persisted = not any(self.session.read_save_flags())
            except proto.ProtocolError as error:
                result.unknown = True
                result.result_code = DRIVER_UNKNOWN
                result.message = "manual command accepted (seq %d), save state unknown: %s" % (result.sequence, error)
                break
            if request.argument or result.manual_mode != 2:
                result.completed = True
                result.message = ("manual active: position invalid, dirty retained; no automatic exit"
                                  if result.manual_mode == 1 else
                                  "manual closed; explicit zero confirmation required; persisted=%s" % result.persisted)
                break
            if self._clock() >= deadline or self._stop_all_requested.is_set() or self._pending_stops:
                result.message = "exit accepted; still waiting for 500 ms without FG"
                break
            time.sleep(MOTION_POLL_INTERVAL)
        handle._resolve(result)

    def _await_motion(self, handle, request, result, terminal_states):
        # type: (TaskHandle, TaskRequest, TaskResult, tuple) -> None
        """Poll until the board reports a terminal state, a fault, or the deadline."""
        deadline = self._clock() + max(0.1, request.timeout_sec)
        while self._clock() < deadline:
            if handle.cancel_requested or request.channel in self._pending_stops \
                    or self._stop_all_requested.is_set():
                self._cancel_motion(handle, request, result)
                return
            try:
                status, position = self.session.read_status_and_position(request.channel)
            except proto.ProtocolError as error:
                self._communication_failure(result, "status read failed: %s" % error)
                handle._resolve(result)
                return
            result.state = status.state
            result.fault = status.fault
            result.position_valid = status.valid
            result.measured_rpm = status.rpm
            result.final_angle_deci_deg = position.angle_deci_deg
            result.target_angle_deci_deg = position.target_deci_deg
            result.final_angle_rad = proto.deci_deg_to_rad(position.angle_deci_deg)
            result.target_angle_rad = proto.deci_deg_to_rad(position.target_deci_deg)
            self._emit_feedback(request, result, status)
            # Keep the published snapshot fresh while a motion runs: the worker is
            # busy here, so without this the topics would go stale exactly when
            # somebody is most interested in them. refresh_snapshot is rate limited
            # by poll_interval, so this adds no extra bus load beyond the configured
            # polling rate.
            self.refresh_snapshot()
            if status.fault or status.state == int(proto.MotionState.FAULT):
                result.completed = False
                result.result_code = DRIVER_OK
                result.message = ("motion ended with fault %d (%s); it is NOT cleared "
                                  "automatically" % (status.fault, status.fault_name))
                handle._resolve(result)
                return
            if status.state in terminal_states:
                persisted = self._await_persistence(deadline)
                result.persisted = persisted
                result.completed = True
                if persisted:
                    result.message = ("motion completed (state DONE, fault 0, brake %s) and the "
                                      "board reports dirty/storage_error/pending_save all zero"
                                      % status.brake)
                else:
                    result.message = ("motion completed but the board has NOT finished saving "
                                      "(dirty/storage_error/pending_save not all zero); do not "
                                      "report the position as persisted")
                self._fill_state(request.channel, result)
                handle._resolve(result)
                return
            time.sleep(MOTION_POLL_INTERVAL)
        # Deadline reached. Do NOT allocate a new sequence and resend: that is how a
        # single goal turns into two motions.
        result.completed = False
        result.result_code = DRIVER_TIMEOUT
        result.unknown = True
        result.message = ("no terminal state within %.1f s; the command was accepted (seq %d) so "
                          "the motion may still be running. No new sequence was issued. Send an "
                          "explicit stop and reconcile." % (request.timeout_sec, result.sequence))
        self.session.mark_unknown(result.message)
        self._fill_state(request.channel, result)
        handle._resolve(result)

    def _await_selftest(self, handle, request, result):
        # type: (TaskHandle, TaskRequest, TaskResult) -> None
        """Wait for a probe to finish, batch aware.

        SELFTEST_ALL is SEQUENTIAL in firmware: when one channel finishes it starts the
        next ST_QUEUED channel (BSP/robot.c:294-296). Reporting as soon as the first
        channel ends would claim a four-channel result from one channel.

        The batch is over only when nothing is queued, running or settling and no motor
        is in S_SELFTEST. Channels the firmware dropped from the queue read
        ST_UNTESTED (BSP/robot.c:102-106), which is NOT a final state, so waiting for
        "all four final" would hang forever after a cancel or a safety abort.

        Records are matched by the command sequence, which the firmware stamps into
        every selected record (BSP/robot.c:179) using the sequence already stored at
        BSP/robot.c:488, so stale records from an earlier batch cannot be mistaken for
        this one.
        """
        batch = bool(request.scope_all)
        sequence = int(result.sequence)
        deadline = self._clock() + max(0.1, request.timeout_sec)
        channels = list(range(proto.CHANNEL_COUNT)) if batch else [int(request.channel)]
        while self._clock() < deadline:
            if handle.cancel_requested or self._stop_all_requested.is_set():
                self._cancel_motion(handle, request, result)
                return
            try:
                records = {}
                statuses = {}
                for channel in channels:
                    records[channel] = self.session.read_selftest(channel)
                    statuses[channel] = self.session.read_status_and_position(channel)[0]
            except proto.ProtocolError as error:
                self._communication_failure(result, "self test read failed: %s" % error)
                handle._resolve(result)
                return
            ours = [ch for ch in channels if records[ch].sequence == sequence]
            if not ours:
                # Accepted but the records are not visible yet; keep waiting rather than
                # concluding that a batch which never appeared has finished.
                self.refresh_snapshot()
                time.sleep(MOTION_POLL_INTERVAL)
                continue
            active = [ch for ch in ours
                      if records[ch].state in (int(proto.SelftestState.QUEUED),
                                               int(proto.SelftestState.RUNNING),
                                               int(proto.SelftestState.SETTLING))
                      or statuses[ch].state == int(proto.MotionState.SELFTEST)]
            focus = active[0] if active else ours[0]
            result.selftest = records[focus]
            result.state = statuses[focus].state
            result.fault = statuses[focus].fault
            result.position_valid = statuses[focus].valid
            result.active_channel = focus
            self._emit_feedback(request, result, statuses[focus], records[focus],
                                active_channel=focus)
            self.refresh_snapshot()
            if not active:
                self._finish_selftest(request, result, channels, records, statuses,
                                      sequence, batch, ours, deadline)
                handle._resolve(result)
                return
            time.sleep(MOTION_POLL_INTERVAL)
        result.completed = False
        result.result_code = DRIVER_TIMEOUT
        result.unknown = True
        result.message = ("self test did not finish within %.1f s; no new sequence was issued. "
                          "The probe may still be energising a channel, so send an explicit stop "
                          "and reconcile before anything else." % request.timeout_sec)
        self.session.mark_unknown(result.message)
        handle._resolve(result)

    @staticmethod
    def _selftest_entry(channel, record, status, sequence):
        # type: (int, proto.SelftestRecord, proto.ChannelStatus, int) -> dict
        return {
            "channel": int(channel),
            "sequence": int(record.sequence),
            "executed": bool(record.sequence == sequence),
            "state": int(record.state),
            "reason": int(record.reason),
            "phase": int(record.phase),
            "outbound_pulses": int(record.outbound_pulses),
            "return_pulses": int(record.return_pulses),
            "residual_pulses": int(record.residual_pulses),
            "powered_ms": int(record.powered_ms),
            "return_ms": int(record.return_ms),
            "position_valid": bool(status.valid),
            "fault": int(status.fault),
        }

    def _collect_selftest(self, result, channels, sequence):
        # type: (TaskResult, List[int], int) -> bool
        """Read every channel's record back, for the cancel and refusal paths.

        A batch that was cut short still has to report what each channel ended up as:
        channels the firmware dropped from the queue read UNTESTED with this batch's
        sequence, which is the only way a caller can see that they never ran.
        """
        entries = []
        for channel in channels:
            try:
                record = self.session.read_selftest(channel)
                status = self.session.read_status_and_position(channel)[0]
            except proto.ProtocolError:
                return False
            entries.append(self._selftest_entry(channel, record, status, sequence))
        result.channel_results = entries
        return True

    def _annotate_selftest_channels(self, result, sequence):
        # type: (TaskResult, int) -> str
        """Describe channels that were selected but never energised."""
        never = [entry for entry in result.channel_results
                 if entry["executed"] and entry["state"] == int(proto.SelftestState.UNTESTED)]
        failed = [entry for entry in result.channel_results
                  if entry["executed"] and entry["state"] == int(proto.SelftestState.FAILED)]
        notes = []
        if never:
            notes.append("channels %s were selected but NEVER ENERGISED (state untested), so their "
                         "result is unknown, not passed" % [entry["channel"] for entry in never])
        if failed:
            notes.append("channels %s FAILED and are not cleared automatically"
                         % [entry["channel"] for entry in failed])
        return "; ".join(notes)

    def _finish_selftest(self, request, result, channels, records, statuses, sequence,
                         batch, ours, deadline):
        # type: (TaskRequest, TaskResult, List[int], Dict[int, object], Dict[int, object], int, bool, List[int], float) -> None
        """Summarise a finished probe: per channel, plus honest batch-level verdicts."""
        for channel in channels:
            result.channel_results.append(
                self._selftest_entry(channel, records[channel], statuses[channel], sequence))
        executed = [entry for entry in result.channel_results if entry["executed"]]
        #: ST_UNTESTED with our sequence means "selected but never energised": the
        #: firmware dropped the rest of the queue (cancel, stop timeout, capture error
        #: or storage failure -- BSP/robot.c:130-139).
        never_energised = [entry for entry in executed
                           if entry["state"] == int(proto.SelftestState.UNTESTED)]
        skipped = [entry for entry in executed
                   if entry["state"] == int(proto.SelftestState.SKIPPED)]
        passed = [entry for entry in executed
                  if entry["state"] == int(proto.SelftestState.PASSED)]
        failed = [entry for entry in executed
                  if entry["state"] == int(proto.SelftestState.FAILED)]
        cancelled = [entry for entry in executed
                     if entry["state"] == int(proto.SelftestState.CANCELLED)]

        persisted = self._await_persistence(deadline)
        result.persisted = persisted
        storage_note = ""
        try:
            dirty, storage_error, pending = self.session.read_save_flags()
            if storage_error:
                storage_note = ("; the board reports storage_error, so a self test result may not "
                                "have been saved and later moves are refused until a save succeeds")
        except proto.ProtocolError:
            storage_note = "; save flags could not be read"

        summary = ", ".join(
            "ch%d=%s%s" % (entry["channel"], proto.selftest_state_name(entry["state"]),
                           "" if entry["reason"] == 0
                           else "(%s)" % proto.selftest_reason_name(entry["reason"]))
            for entry in executed) or "no channel was probed"

        if batch:
            result.batch_completed = not never_energised and not cancelled
            result.all_passed = bool(executed) and result.batch_completed and not failed \
                and len(passed) == len(executed) - len(skipped)
            result.completed = result.batch_completed
            result.passed = result.all_passed
            result.message = (
                "batch self test %s: %s%s. Passing only proves at least %d feedback pulses were "
                "captured per channel; it does NOT prove motor wiring, direction, or an accurate "
                "return to the start pose."
                % ("completed" if result.batch_completed else "did NOT run to completion",
                   summary, storage_note, proto.SELFTEST_TARGET_PULSES))
            if never_energised:
                result.message += (" Channels %s were selected but never energised, so the batch "
                                   "was aborted (cancel, stop timeout, capture error or storage "
                                   "failure) -- their result is unknown, not passed."
                                   % [entry["channel"] for entry in never_energised])
            if failed:
                result.message += (" Channels %s FAILED and are NOT cleared automatically."
                                   % [entry["channel"] for entry in failed])
        else:
            only = executed[0] if executed else None
            result.completed = bool(only) and only["state"] in proto.SELFTEST_FINAL_STATES
            result.passed = bool(only) and only["state"] == int(proto.SelftestState.PASSED)
            result.message = (
                "self test finished in state %s (reason %s)%s; passing only proves at least %d "
                "feedback pulses were captured, not a mechanical connection or a direction"
                % (proto.selftest_state_name(only["state"]) if only else "unknown",
                   proto.selftest_reason_name(only["reason"]) if only else "unknown",
                   storage_note, proto.SELFTEST_TARGET_PULSES))
        result.result_code = DRIVER_OK if result.completed else DRIVER_UNKNOWN
        if not result.completed and not batch:
            result.unknown = True

    def _cancel_motion(self, handle, request, result):
        # type: (TaskHandle, TaskRequest, TaskResult) -> None
        """Cancel by sending STOP, then report whether the board confirmed it.

        A batch self test is cancelled with STOP_ALL: the firmware probes channels one
        after another, so stopping only the current channel would let the batch move on
        to the next one.
        """
        result.cancelled = True
        scope_all = bool(request.scope_all)
        opcode = proto.Opcode.STOP_ALL if scope_all else proto.Opcode.STOP
        try:
            outcome = self.session.send_command(opcode, request.channel, 0,
                                                ros_target_id=request.ros_target_id + ":cancel",
                                                allow_while_unknown=True)
        except proto.ProtocolError as error:
            result.stop_confirmed = False
            result.unknown = True
            result.result_code = DRIVER_UNKNOWN
            result.message = ("cancel could not send %s (%s); the mechanism may still be moving and "
                              "this is NOT a confirmed stop"
                              % (proto.opcode_name(int(opcode)), error))
            handle._resolve(result)
            return
        result.accepted = result.accepted or outcome.accepted
        if not outcome.accepted:
            result.stop_confirmed = False
            result.result_code = outcome.result
            result.message = "cancel: %s was refused (%s); stop NOT confirmed" % (
                proto.opcode_name(int(opcode)), outcome.detail)
            handle._resolve(result)
            return
        stop_request = TaskRequest(kind="stop", channel=request.channel, scope_all=scope_all,
                                   timeout_sec=STOP_CONFIRM_TIMEOUT,
                                   ros_target_id=request.ros_target_id + ":confirm")
        confirm = TaskResult()
        result.stop_confirmed = self._wait_stop_confirmed(stop_request, confirm)
        self._pending_stops.discard(request.channel)
        if scope_all:
            self._stop_all_requested.clear()
        result.message = ("cancelled; stop confirmed by the board" if result.stop_confirmed
                          else "cancelled; %s was accepted but the board did NOT confirm the "
                               "channel settled" % proto.opcode_name(int(opcode)))
        if request.kind == "selftest":
            # Cancelling a batch drops the rest of the queue, so report which channels
            # never ran instead of leaving the caller to guess from an empty list.
            channels = (list(range(proto.CHANNEL_COUNT)) if request.scope_all
                        else [int(request.channel)])
            result.batch_completed = False
            result.all_passed = False
            result.completed = False
            if self._collect_selftest(result, channels, int(result.sequence)):
                note = self._annotate_selftest_channels(result, int(result.sequence))
                if note:
                    result.message += "; " + note
        self._fill_state(request.channel, result)
        handle._resolve(result)

    def _await_persistence(self, deadline):
        # type: (float) -> bool
        """Wait for dirty/storage_error/pending_save to reach zero, bounded by deadline."""
        while self._clock() < deadline:
            try:
                dirty, storage_error, pending = self.session.read_save_flags()
            except proto.ProtocolError:
                return False
            if not (dirty or storage_error or pending):
                return True
            time.sleep(MOTION_POLL_INTERVAL)
        return False

    def _fill_state(self, channel, result):
        # type: (int, TaskResult) -> None
        try:
            status, position = self.session.read_status_and_position(channel)
        except proto.ProtocolError:
            return
        result.state = status.state
        result.fault = status.fault
        result.position_valid = status.valid
        result.measured_rpm = status.rpm
        result.final_angle_deci_deg = position.angle_deci_deg
        result.target_angle_deci_deg = position.target_deci_deg
        result.final_angle_rad = proto.deci_deg_to_rad(position.angle_deci_deg)
        result.target_angle_rad = proto.deci_deg_to_rad(position.target_deci_deg)

    def _communication_failure(self, result, detail):
        # type: (TaskResult, str) -> None
        result.unknown = True
        result.result_code = DRIVER_UNKNOWN
        result.message = "%s; outcome UNKNOWN, motion scheduling halted" % detail
        self.session.mark_unknown(detail)
        self._handle_communication_loss(detail)

    def _emit_feedback(self, request, result, status, record=None, active_channel=-1):
        # type: (TaskRequest, TaskResult, proto.ChannelStatus, Optional[proto.SelftestRecord], int) -> None
        if request.feedback is None:
            return
        payload = {"sequence": result.sequence, "state": status.state, "fault": status.fault,
                   "position_valid": status.valid, "brake": status.brake,
                   "communication_ok": self.connected,
                   "current_angle_deci_deg": result.final_angle_deci_deg,
                   "target_angle_deci_deg": result.target_angle_deci_deg,
                   "current_angle_rad": result.final_angle_rad,
                   "target_angle_rad": result.target_angle_rad,
                   "measured_motor_rpm": status.rpm,
                   "phase": record.phase if record else 0,
                   "powered_ms": record.powered_ms if record else 0,
                   "return_ms": record.return_ms if record else 0,
                   "active_channel": int(active_channel)}
        try:
            request.feedback(payload)
        except Exception as error:      # pragma: no cover - defensive
            self._log("error", "feedback callback failed: %s" % error)

    # -- maintenance -----------------------------------------------------------

    def _task_poll(self, handle):
        # type: (TaskHandle) -> None
        """Pure read task: refresh the snapshot on the worker thread."""
        snapshot = self.refresh_snapshot(force=True)
        handle._resolve(TaskResult(accepted=snapshot.communication_ok,
                                   completed=snapshot.communication_ok,
                                   result_code=DRIVER_OK if snapshot.communication_ok
                                   else DRIVER_NOT_CONNECTED,
                                   message=snapshot.error or "snapshot refreshed",
                                   position_valid=False))

    def _task_direction(self, handle):
        result = TaskResult()
        with self._active_lock:
            queued = bool(self._active_joints)
        snapshot = self.refresh_snapshot(force=True)
        channel = handle.request.channel
        if not snapshot.communication_ok or len(snapshot.statuses) != proto.CHANNEL_COUNT:
            handle._resolve(TaskResult(result_code=DRIVER_NOT_CONNECTED, message="fresh state unavailable; no DIR sent"))
            return
        status = snapshot.channel_status(channel)
        if snapshot.info.storage_error:
            handle._resolve(TaskResult(result_code=int(proto.BusinessResult.STORAGE), message="storage error; no DIR sent"))
            return
        if queued or snapshot.busy_channels or any(snapshot.manual_modes) or status.fault or not status.brake or status.duty_permille or status.rpm:
            handle._resolve(TaskResult(result_code=DRIVER_BUSY,
                message="stop motion, exit manual and brake channel before DIR switching; no command sent"))
            return
        outcome = self._send(handle, handle.request, result)
        if outcome is None:
            return
        snapshot = self.refresh_snapshot(force=True)
        if not snapshot.communication_ok:
            result.unknown = True
            result.result_code = DRIVER_UNKNOWN
            result.message = "DIR read-back unavailable; outcome unknown, no automatic toggle"
        else:
            result.direction = bool(snapshot.channel_status(channel).direction)
            result.direction_confirmed = bool(outcome.accepted) and result.direction == bool(handle.request.argument)
            result.completed = result.direction_confirmed
            if outcome.accepted:
                result.message = ("DIR command read-back confirmed %s; not a voltage measurement; next MOVE resets DIR"
                                  % ("HIGH" if result.direction else "LOW")) if result.direction_confirmed else "DIR accepted but read-back does not match"
        handle._resolve(result)

    def _task_retry_save(self, handle):
        # All I/O stays on this worker; never issue ZERO or stage configuration.
        result = TaskResult()
        with self._active_lock:
            queued = bool(self._active_joints)
        snapshot = self.refresh_snapshot(force=True)
        if not snapshot.communication_ok or len(snapshot.statuses) != proto.CHANNEL_COUNT:
            handle._resolve(TaskResult(result_code=DRIVER_NOT_CONNECTED,
                message="fresh board state unavailable; no SAVE sent"))
            return
        if queued or any(snapshot.manual_modes) or any(
                s.busy or not s.brake or s.duty_permille or s.rpm for s in snapshot.statuses):
            handle._resolve(TaskResult(result_code=DRIVER_BUSY,
                message="all four channels must be stopped, braked and out of manual mode; no SAVE sent"))
            return
        outcome = self._send(handle, handle.request, result)
        if outcome is None:
            return
        try:
            result.dirty, result.storage_error, result.pending_save = self.session.read_save_flags()
            result.flags_valid = True
            result.persisted = bool(outcome.accepted) and not any(
                (result.dirty, result.storage_error, result.pending_save))
            result.completed = result.persisted
            if outcome.accepted:
                result.message = ("SAVE succeeded; all save flags zero; zero and faults unchanged" if result.persisted
                                  else "SAVE accepted but save flags not clean; persistence NOT confirmed")
            self.refresh_snapshot(force=True)
        except proto.ProtocolError as error:
            result.unknown = True
            result.persisted = False
            result.result_code = DRIVER_UNKNOWN
            result.message = "SAVE read-back failed; outcome unknown, no automatic retry: %s" % error
        handle._resolve(result)

    def _task_simple_command(self, handle):
        # type: (TaskHandle) -> None
        request = handle.request
        result = TaskResult()
        outcome = self._send(handle, request, result)
        if outcome is None:
            return
        if outcome.accepted:
            result.completed = True
            result.message = "%s accepted (seq %d)" % (request.kind, outcome.sequence)
            if request.kind in ("zero", "save", "clear_fault"):
                result.persisted = self._await_persistence(self._clock() + 2.0)
                if not result.persisted:
                    result.message += "; the board has not finished saving yet"
        self._fill_state(request.channel, result)
        handle._resolve(result)

    def _task_config(self, handle):
        # type: (TaskHandle) -> None
        """Stage the whole board, verify the staged area, then CONFIG once.

        Every outcome reports what the board actually holds: the staged area read back
        before committing, and the applied area read back afterwards. A refusal or a
        failed save still returns the real state, because "we asked for X" is not
        evidence that the board has X.
        """
        from .board_session import DRIVER_CONFIG_MISMATCH

        request = handle.request
        result = TaskResult()
        staged = list(request.staged_configs or [])

        def read_back_staged():
            # type: () -> Optional[List[proto.ChannelConfigValues]]
            values = []
            for index in range(len(staged)):
                try:
                    values.append(self.session.read_channel_staged(index))
                except proto.ProtocolError:
                    return None
            return values

        def read_back_applied():
            # type: () -> Optional[List[proto.ChannelConfigValues]]
            values = []
            for index in range(len(staged)):
                try:
                    values.append(self.session.read_channel_config(index))
                except proto.ProtocolError:
                    return None
            return values

        try:
            if request.staged_board:
                self.session.stage_board_config(*request.staged_board)
            for index, values in enumerate(staged):
                self.session.stage_channel_config(index, values)
        except proto.ReadOnlyViolation as error:
            handle._resolve(TaskResult(result_code=DRIVER_READONLY, message=str(error)))
            return
        except proto.ProtocolError as error:
            handle._resolve(TaskResult(result_code=DRIVER_UNKNOWN, unknown=True,
                                       message="staging failed: %s" % error))
            return

        # CONFIG validates the whole staged area at once and writes flash, so a staged
        # value that did not take must be caught before the command, not after.
        actual_staged = read_back_staged()
        if actual_staged is None:
            handle._resolve(TaskResult(result_code=DRIVER_UNKNOWN, unknown=True,
                                       message="the staged area could not be read back, so CONFIG "
                                               "was NOT sent"))
            return
        result.staged_configs = actual_staged
        mismatch = [(index, staged[index].to_registers(), actual_staged[index].to_registers())
                    for index in range(len(staged))
                    if staged[index].to_registers() != actual_staged[index].to_registers()]
        if mismatch and request.commit:
            handle._resolve(TaskResult(
                result_code=DRIVER_CONFIG_MISMATCH, accepted=False,
                staged_configs=actual_staged,
                message="staged configuration did not read back as requested, so CONFIG was NOT "
                        "sent and no flash was written: %s" % mismatch))
            return
        if not request.commit:
            handle._resolve(TaskResult(accepted=True, completed=False,
                                       staged_configs=actual_staged,
                                       message="staged only; CONFIG was not sent, nothing applied",
                                       detail=str(mismatch)))
            return
        outcome = self._send(handle, request, result)
        if outcome is None:
            return
        applied = read_back_applied()
        if applied is not None:
            result.applied_configs = applied
        if outcome.accepted:
            result.completed = True
            result.persisted = self._await_persistence(self._clock() + 2.0)
            result.message = ("CONFIG accepted (seq %d); pwm_hz applies immediately but protocol, "
                              "address and node only take effect after a power cycle"
                              % outcome.sequence)
            if not result.persisted:
                result.message += ("; the board has NOT finished saving (dirty/storage_error/"
                                   "pending_save not all zero), so do not treat this as persisted")
            if applied is None:
                result.message += "; the applied area could NOT be read back, so it is unverified"
            else:
                still = [(index, staged[index].to_registers(), applied[index].to_registers())
                         for index in range(len(staged))
                         if staged[index].to_registers() != applied[index].to_registers()]
                if still:
                    result.message += ("; WARNING the applied area differs from what was staged: %s"
                                       % still)
        handle._resolve(result)
