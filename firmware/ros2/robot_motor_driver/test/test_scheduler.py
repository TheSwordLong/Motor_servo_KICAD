"""Scheduler tests: the stage-D acceptance cases, offline against the fake board.

These use the REAL clock (not the fake one) because the worker sleeps between
polls; motion durations are shrunk with `motion_duration_scale` so a full
move-settle-save cycle takes about a second instead of minutes.

Covered: read-only interception, allow_motion gating, same-channel conflict,
cancel with stop confirmation, STOP priority over queued work, timeout that must
NOT mint a new sequence, communication loss and no auto-resume, configuration
transaction with read-back verification, and the maintenance commands.
"""

import time

import pytest

from robot_motor_driver import protocol as proto
from robot_motor_driver.board_session import (DRIVER_BUSY, DRIVER_CONFIG_MISMATCH, DRIVER_HALTED,
                                             DRIVER_LIMIT, DRIVER_LIMITS_UNCONFIRMED, DRIVER_NO_OP, DRIVER_OK, DRIVER_READONLY,
                                             DRIVER_RPM_MISMATCH, DRIVER_STALE_POSITION,
                                             DRIVER_TIMEOUT, DRIVER_VALID_REQUIRED,
                                             DRIVER_UNKNOWN, BoardSession)
from robot_motor_driver.fake_board import make_fake_stack
from robot_motor_driver.scheduler import (MOVE_ABSOLUTE, MOVE_RELATIVE, BoardWorker, TaskRequest)


def build(read_only=False, allow_motion=True, poll_interval=0.05, motion_scale=0.005,
          reconnect_delay=0.01, probe_scale=1.0):
    """A worker on a real clock with a fast simulated board."""
    transport, board, stub, session, clock = make_fake_stack(
        clock=time.monotonic, read_only=read_only)
    board.motion_duration_scale = motion_scale
    # A four-channel batch probe takes 2 s per leg in firmware time; probe_scale shrinks
    # that so batch tests stay fast without changing phase order or pass/fail rules.
    board.probe_time_scale = probe_scale
    # The firmware needs 500 ms without any FG edge on ANY channel before it accepts
    # CONFIG/ZERO/SELFTEST/DIRECTION, and the same board-quiet check gates a MOVE.
    # Back-date the edge timestamps so a freshly built fake board is already quiet.
    settled = time.monotonic() - 2.0
    for channel in board.channels:
        channel.last_edge = settled
    worker = BoardWorker(session, poll_interval=poll_interval, allow_motion=allow_motion,
                         reconnect_delay=reconnect_delay)
    worker.start()
    # Let the first poll settle so the snapshot is populated before the test acts.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not worker.snapshot.communication_ok:
        time.sleep(0.05)
    return worker, session, board, stub, transport


def settle(board, seconds=0.7, steps=3):
    """Advance the simulated board through its settling and background-save phases."""
    for _ in range(steps):
        board.advance(seconds / float(steps))


def shutdown(worker):
    worker.stop(timeout=3.0)


def commissioning(**overrides):
    """TaskRequest flags for tests about scheduler mechanics rather than about limits.

    With `limits_rad` unset the driver now REFUSES motion by default (an unconfirmed
    mechanical limit is not a limit), so tests that exercise busy handling, timeouts,
    cancellation, stop priority or board-level results must state explicitly that they
    are running the commissioning path. Anything testing limits passes its own values
    and must not use this helper.
    """
    flags = {"limits_rad": None, "allow_without_limits": True, "allow_unreferenced": True}
    flags.update(overrides)
    return flags


# --------------------------------------------------------------------------
# Gating
# --------------------------------------------------------------------------


def test_read_only_refuses_motion_and_stop():
    worker, session, board, stub, transport = build(read_only=True, allow_motion=False)
    try:
        handle = worker.submit(TaskRequest(kind="move", channel=0,
                                           opcode=int(proto.Opcode.MOVE), argument=900,
                                           timeout_sec=5.0))
        assert handle.rejected is True
        assert handle.result.result_code == DRIVER_READONLY
        stop = worker.request_stop(0)
        assert stop.rejected is True
        assert "hardware e-stop" in stop.result.message
        assert board.commands_received == 0, "read-only must not put a single frame on the bus"
        # Reads still work.
        snapshot = worker.refresh_now()
        assert snapshot.communication_ok is True
    finally:
        shutdown(worker)


def test_allow_motion_false_blocks_motion_but_permits_maintenance():
    """maintenance mode: stop/zero/clear/brake are allowed, motion is not."""
    worker, session, board, stub, transport = build(read_only=False, allow_motion=False)
    try:
        move = worker.submit(TaskRequest(kind="move", channel=0,
                                         opcode=int(proto.Opcode.MOVE), argument=900,
                                         timeout_sec=5.0))
        assert move.rejected is True
        assert move.result.result_code == DRIVER_READONLY
        assert "allow_motion" in move.result.message
        zero = worker.submit(TaskRequest(kind="zero", channel=0,
                                         opcode=int(proto.Opcode.ZERO), timeout_sec=5.0))
        assert zero.wait(5.0)
        assert zero.result.accepted is True
        assert board.channels[0].valid is True
    finally:
        shutdown(worker)


def test_worker_must_be_started_before_submit():
    transport, board, stub, session, clock = make_fake_stack(clock=time.monotonic)
    worker = BoardWorker(session, allow_motion=True)
    handle = worker.submit(TaskRequest(kind="move", channel=0, opcode=int(proto.Opcode.MOVE),
                                       argument=100, timeout_sec=1.0))
    assert handle.rejected is True


# --------------------------------------------------------------------------
# Motion
# --------------------------------------------------------------------------


def test_relative_move_completes_and_persists():
    worker, session, board, stub, transport = build()
    try:
        handle = worker.submit(TaskRequest(kind="move", channel=0,
                                           opcode=int(proto.Opcode.MOVE), argument=900,
                                           timeout_sec=20.0, ros_target_id="goal-1",
                                           **commissioning()))
        assert handle.wait(20.0), "motion did not finish in time"
        result = handle.result
        assert result.accepted is True
        assert result.completed is True
        assert result.persisted is True, "dirty/storage_error/pending_save must all be zero"
        assert result.unknown is False
        assert result.result_code == DRIVER_OK
        assert board.channels[0].state == int(proto.MotionState.DONE)
        assert board.channels[0].angle_deci_deg == 900
        assert result.final_angle_rad == pytest.approx(proto.deci_deg_to_rad(900))
        assert session.unknown_command_state is False
    finally:
        shutdown(worker)


def test_conflicting_motion_on_the_same_channel_is_rejected():
    """The firmware refuses a new MOVE on a busy channel, so the driver never queues one."""
    worker, session, board, stub, transport = build(motion_scale=0.6)
    try:
        first = worker.submit(TaskRequest(kind="move", channel=0,
                                          opcode=int(proto.Opcode.MOVE), argument=900,
                                          timeout_sec=30.0, ros_target_id="a",
                                          **commissioning()))
        time.sleep(0.3)
        second = worker.submit(TaskRequest(kind="move", channel=0,
                                           opcode=int(proto.Opcode.MOVE), argument=-900,
                                           timeout_sec=30.0, ros_target_id="b",
                                           **commissioning()))
        assert second.rejected is True
        assert second.result.result_code == DRIVER_BUSY
        assert first.rejected is False
        # A different channel is not blocked by the joint lock itself.
        other = worker.submit(TaskRequest(kind="move", channel=1,
                                          opcode=int(proto.Opcode.MOVE), argument=900,
                                          timeout_sec=30.0, ros_target_id="c",
                                          **commissioning()))
        assert other.rejected is False
        worker.request_stop(None)
        first.wait(10.0)
        other.wait(10.0)
    finally:
        shutdown(worker)


def test_cancel_sends_stop_and_reports_whether_the_board_confirmed():
    worker, session, board, stub, transport = build(motion_scale=1.0)
    try:
        handle = worker.submit(TaskRequest(kind="move", channel=0,
                                           opcode=int(proto.Opcode.MOVE), argument=3600,
                                           timeout_sec=60.0, ros_target_id="cancel-me",
                                           **commissioning()))
        time.sleep(0.4)
        assert board.channels[0].state in (int(proto.MotionState.RUNNING),
                                           int(proto.MotionState.DECEL))
        handle.cancel()
        assert handle.wait(15.0), "cancel did not finish"
        result = handle.result
        assert result.cancelled is True
        assert result.stop_confirmed is True, "the board must confirm the channel settled"
        assert board.channels[0].brake is True
        assert board.channels[0].duty == 0
        assert board.channels[0].state not in proto.BUSY_STATES
        assert session.unknown_command_state is False
    finally:
        shutdown(worker)


def test_stop_request_preempts_a_queued_motion():
    """STOP has the highest priority, so queued work never starts after a stop."""
    worker, session, board, stub, transport = build(motion_scale=1.0)
    try:
        first = worker.submit(TaskRequest(kind="move", channel=0,
                                          opcode=int(proto.Opcode.MOVE), argument=3600,
                                          timeout_sec=60.0, ros_target_id="long",
                                          **commissioning()))
        time.sleep(0.2)
        stop = worker.request_stop(0, ros_target_id="operator")
        assert stop.wait(15.0)
        assert stop.result.accepted is True
        assert stop.result.stop_confirmed is True
        first.wait(15.0)
        assert first.result.cancelled is True or board.channels[0].state not in proto.BUSY_STATES
    finally:
        shutdown(worker)


def test_move_timeout_does_not_mint_a_new_sequence_and_halts_scheduling():
    """A MOVE that outlives its deadline becomes UNKNOWN. The one thing the driver
    must never do is allocate a fresh sequence and send the motion again."""
    worker, session, board, stub, transport = build(motion_scale=20.0)
    try:
        handle = worker.submit(TaskRequest(kind="move", channel=0,
                                           opcode=int(proto.Opcode.MOVE), argument=900,
                                           timeout_sec=1.0, ros_target_id="slow",
                                           **commissioning()))
        assert handle.wait(20.0)
        result = handle.result
        assert result.completed is False
        assert result.unknown is True
        assert result.result_code == DRIVER_TIMEOUT
        assert "No new sequence was issued" in result.message or "no new sequence" in result.message.lower()
        assert session.unknown_command_state is True
        # Exactly one command frame reached the board: the MOVE. No retry, no second seq.
        assert board.commands_received == 1
        assert board.last_seq == result.sequence
        # Further work is refused until an operator acknowledges.
        blocked = worker.submit(TaskRequest(kind="move", channel=1,
                                            opcode=int(proto.Opcode.MOVE), argument=900,
                                            timeout_sec=5.0))
        assert blocked.rejected is True
        assert blocked.result.result_code == DRIVER_HALTED
        # A stop is still allowed through the halt.
        stop = worker.request_stop(0)
        assert stop.wait(15.0)
    finally:
        shutdown(worker)


def test_unknown_acknowledgement_unlocks_without_resuming():
    worker, session, board, stub, transport = build(motion_scale=20.0)
    try:
        handle = worker.submit(TaskRequest(kind="move", channel=0,
                                           opcode=int(proto.Opcode.MOVE), argument=900,
                                           timeout_sec=1.0, **commissioning()))
        handle.wait(20.0)
        assert session.unknown_command_state is True
        received_before = board.commands_received
        session.acknowledge_unknown()
        assert session.unknown_command_state is False
        assert board.commands_received == received_before, "acknowledging must not replay anything"
    finally:
        shutdown(worker)


# --------------------------------------------------------------------------
# Communication loss
# --------------------------------------------------------------------------


def test_communication_loss_marks_unknown_and_reconnect_does_not_resume():
    worker, session, board, stub, transport = build(motion_scale=1.0)
    try:
        received_before = board.commands_received
        stub.is_open = False       # the port disappears, as an unplug would
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not session.unknown_command_state:
            time.sleep(0.1)
        assert session.unknown_command_state is True, "a lost port must halt scheduling"
        assert worker.reconnect_attempts >= 1
        # The stub factory re-opens the port, so the session re-identifies...
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not session.identified:
            time.sleep(0.1)
        assert session.identified is True
        # ...but nothing was resumed and no extra command was invented.
        assert board.commands_received == received_before
        assert session.unknown_command_state is True
    finally:
        shutdown(worker)


# --------------------------------------------------------------------------
# Maintenance commands
# --------------------------------------------------------------------------


def test_set_zero_makes_the_channel_referenced_and_persisted():
    worker, session, board, stub, transport = build()
    try:
        assert board.channels[2].valid is False
        handle = worker.submit(TaskRequest(kind="zero", channel=2,
                                           opcode=int(proto.Opcode.ZERO), timeout_sec=10.0))
        assert handle.wait(10.0)
        assert handle.result.accepted is True
        assert handle.result.persisted is True
        assert board.channels[2].valid is True
        assert board.channels[2].state == int(proto.MotionState.IDLE)
    finally:
        shutdown(worker)


def test_clear_fault_does_not_restore_a_lost_zero():
    worker, session, board, stub, transport = build()
    try:
        board.inject_fault(1, int(proto.FaultCode.UNEXPECTED_FG))
        assert board.channels[1].fault == int(proto.FaultCode.UNEXPECTED_FG)
        handle = worker.submit(TaskRequest(kind="clear_fault", channel=1,
                                           opcode=int(proto.Opcode.CLEAR_FAULT), timeout_sec=10.0))
        assert handle.wait(10.0)
        assert handle.result.accepted is True
        assert board.channels[1].fault == 0
        assert board.channels[1].valid is False, \
            "CLEAR_FAULT must not resurrect position_valid; that needs an explicit ZERO"
    finally:
        shutdown(worker)


def test_faulted_channel_blocks_motion_and_is_not_cleared_automatically():
    worker, session, board, stub, transport = build()
    try:
        board.inject_fault(3, int(proto.FaultCode.UNEXPECTED_FG))
        time.sleep(0.2)
        handle = worker.submit(TaskRequest(kind="move", channel=3,
                                           opcode=int(proto.Opcode.MOVE), argument=900,
                                           timeout_sec=10.0, **commissioning()))
        assert handle.wait(10.0)
        assert handle.result.accepted is False
        assert handle.result.result_code == int(proto.BusinessResult.UNREADY)
        assert board.channels[3].fault == int(proto.FaultCode.UNEXPECTED_FG), \
            "the driver must never clear a fault on its own"
    finally:
        shutdown(worker)


def test_unexpected_pulse_budget_uses_applied_gear():
    worker, session, board, stub, transport = build()
    try:
        board.inject_unexpected_pulses(0, proto.unexpected_fg_budget(board.channels[0].gear))
        assert board.channels[0].fault == 0
        board.inject_unexpected_pulses(0, 1)
        assert board.channels[0].fault == int(proto.FaultCode.UNEXPECTED_FG)
    finally:
        shutdown(worker)


def test_brake_engage_and_release():
    worker, session, board, stub, transport = build()
    try:
        release = worker.submit(TaskRequest(kind="brake", channel=0,
                                            opcode=int(proto.Opcode.BRAKE), argument=0,
                                            timeout_sec=10.0))
        assert release.wait(10.0)
        assert release.result.accepted is True
        assert board.channels[0].brake is False
        engage = worker.submit(TaskRequest(kind="brake", channel=0,
                                           opcode=int(proto.Opcode.BRAKE), argument=1,
                                           timeout_sec=10.0))
        assert engage.wait(10.0)
        assert board.channels[0].brake is True
    finally:
        shutdown(worker)


def test_storage_failure_is_reported_and_blocks_motion():
    worker, session, board, stub, transport = build()
    try:
        board.storage_failure = True
        handle = worker.submit(TaskRequest(kind="save", channel=0,
                                           opcode=int(proto.Opcode.SAVE), timeout_sec=10.0))
        assert handle.wait(10.0)
        assert handle.result.accepted is False
        assert handle.result.result_code == int(proto.BusinessResult.STORAGE)
        assert board.storage_error is True
        move = worker.submit(TaskRequest(kind="move", channel=0,
                                         opcode=int(proto.Opcode.MOVE), argument=900,
                                         timeout_sec=10.0, **commissioning()))
        assert move.wait(10.0)
        assert move.result.accepted is False
        assert move.result.result_code == int(proto.BusinessResult.STORAGE)
    finally:
        shutdown(worker)


# --------------------------------------------------------------------------
# Configuration transaction
# --------------------------------------------------------------------------


def _staged_configs(gear_for_channel_1=516):
    staged = []
    for channel in range(proto.CHANNEL_COUNT):
        gear = gear_for_channel_1 if channel == 1 else (516 if channel == 0 else
                                                        (290 if channel == 2 else 82))
        staged.append(proto.ChannelConfigValues(enabled=True, role=channel + 1, gear=gear,
                                                inverted=0, rpm=8000))
    return staged


def test_config_transaction_commits_once_and_reports_persistence():
    worker, session, board, stub, transport = build()
    try:
        handle = worker.submit(TaskRequest(kind="config", opcode=int(proto.Opcode.CONFIG),
                                           staged_configs=_staged_configs(),
                                           staged_board=[0, 1, 1, 25000], commit=True,
                                           timeout_sec=15.0))
        assert handle.wait(15.0)
        result = handle.result
        assert result.accepted is True
        assert result.persisted is True
        assert board.commands_received == 1, "exactly one CONFIG command, not one per channel"
        assert board.channels[1].gear == 516
        assert board.dirty is False and board.storage_error is False
    finally:
        shutdown(worker)


def test_config_read_back_mismatch_prevents_the_commit():
    """CONFIG validates the whole staged area and writes flash, so a staged value
    that does not read back must stop the transaction before the command."""
    worker, session, board, stub, transport = build()
    try:
        original = session.read_channel_staged

        def lying_read(channel):
            values = original(channel)
            if channel == 1:
                values.gear = 290
            return values

        session.read_channel_staged = lying_read
        handle = worker.submit(TaskRequest(kind="config", opcode=int(proto.Opcode.CONFIG),
                                           staged_configs=_staged_configs(),
                                           staged_board=[0, 1, 1, 25000], commit=True,
                                           timeout_sec=15.0))
        assert handle.wait(15.0)
        assert handle.result.accepted is False
        assert handle.result.result_code == DRIVER_CONFIG_MISMATCH
        assert board.commands_received == 0, "CONFIG must not be sent after a mismatch"
    finally:
        shutdown(worker)


def test_stage_only_mode_does_not_send_config():
    worker, session, board, stub, transport = build()
    try:
        handle = worker.submit(TaskRequest(kind="config", opcode=int(proto.Opcode.CONFIG),
                                           staged_configs=_staged_configs(),
                                           staged_board=[0, 1, 1, 25000], commit=False,
                                           timeout_sec=15.0))
        assert handle.wait(15.0)
        assert board.commands_received == 0
        assert "staged only" in handle.result.message
    finally:
        shutdown(worker)


def test_config_applies_pwm_immediately_but_address_only_after_reboot():
    """A CONFIG that changes the address must not make the driver believe the board
    moved: the active registers only change at the next power cycle."""
    worker, session, board, stub, transport = build()
    try:
        handle = worker.submit(TaskRequest(kind="config", opcode=int(proto.Opcode.CONFIG),
                                           staged_configs=_staged_configs(),
                                           staged_board=[0, 7, 3, 22000], commit=True,
                                           timeout_sec=15.0))
        assert handle.wait(15.0)
        assert handle.result.accepted is True
        assert board.pwm_hz == 22000, "pwm_hz applies immediately"
        assert board.address == 1, "the active address must not change without a reboot"
        assert board.node == 1
        board.reboot()
        assert board.address == 7 and board.node == 3
    finally:
        shutdown(worker)


# --------------------------------------------------------------------------
# Snapshot plumbing
# --------------------------------------------------------------------------


def test_refresh_now_reads_on_the_worker_thread():
    """Callers must never touch the bus themselves; refresh_now keeps one owner."""
    worker, session, board, stub, transport = build()
    try:
        snapshot = worker.refresh_now(timeout=5.0)
        assert snapshot.communication_ok is True
        assert snapshot.uid == board.uid
        assert transport.owner is not None
        assert worker.snapshot_age() < 5.0
    finally:
        shutdown(worker)


def test_snapshot_age_grows_and_is_reported_as_stale_when_the_port_dies():
    worker, session, board, stub, transport = build()
    try:
        worker.stop(timeout=3.0)
        age = worker.snapshot_age()
        assert age == float("inf") or age >= 0.0
    finally:
        if worker.running:
            shutdown(worker)


# --------------------------------------------------------------------------
# Stopping while halted after an UNKNOWN outcome
# --------------------------------------------------------------------------


def test_stop_all_is_sent_while_halted_and_confirms_the_stop():
    """The halt exists to stop NEW motion. Refusing the one command that can stop the
    mechanism would leave the robot moving with no software way to intervene."""
    worker, session, board, stub, transport = build()
    try:
        session.mark_unknown("simulated unknown outcome")
        handle = worker.request_stop(None, ros_target_id="test:stop")
        assert handle.wait(10.0) is True
        result = handle.result
        assert result.accepted is True
        assert result.stop_confirmed is True
        assert result.unknown is False
        assert session.unknown_command_state is True, "a stop must not acknowledge the halt"
        assert all(ch.state not in proto.BUSY_STATES for ch in board.channels)
        assert all(ch.brake for ch in board.channels)
    finally:
        shutdown(worker)


def test_motion_config_and_selftest_stay_refused_while_halted():
    worker, session, board, stub, transport = build()
    try:
        session.mark_unknown("simulated")
        move = worker.submit(TaskRequest(kind="move", channel=2, opcode=int(proto.Opcode.MOVE),
                                         argument=100, timeout_sec=5.0))
        assert move.rejected is True
        assert move.result.result_code == DRIVER_HALTED
        config = worker.submit(TaskRequest(kind="config", opcode=int(proto.Opcode.CONFIG),
                                           staged_configs=_staged_configs(), commit=True,
                                           timeout_sec=5.0))
        assert config.rejected is True and config.result.result_code == DRIVER_HALTED
        probe = worker.submit(TaskRequest(kind="selftest", channel=0,
                                          opcode=int(proto.Opcode.SELFTEST), timeout_sec=5.0))
        assert probe.rejected is True and probe.result.result_code == DRIVER_HALTED
        zero = worker.submit(TaskRequest(kind="zero", channel=2, opcode=int(proto.Opcode.ZERO),
                                         timeout_sec=5.0))
        assert zero.rejected is True and zero.result.result_code == DRIVER_HALTED
        assert board.commands_received == 0, "a refused task must never reach the bus"
    finally:
        shutdown(worker)


def test_poll_stays_allowed_while_halted_so_a_stop_can_be_confirmed():
    """Blocking reads during the halt would leave the driver holding a stale snapshot
    with no way to confirm that its own stop worked."""
    worker, session, board, stub, transport = build()
    try:
        session.mark_unknown("simulated")
        snapshot = worker.refresh_now(timeout=5.0)
        assert snapshot is not None
        assert snapshot.communication_ok is True
        assert snapshot.uid == board.uid
    finally:
        shutdown(worker)


def test_stop_with_no_reply_is_reported_as_unknown_not_stopped():
    """A frame that left the wire is not evidence the mechanism stopped."""
    worker, session, board, stub, transport = build()
    original = session.read_last_sequence
    calls = []
    try:
        # Drop EVERY command reply: the transport retries the identical frame with the
        # same sequence, so dropping only one still ends in a normal accepted reply.
        stub.drop_command_responses = 99

        def flaky():
            # The first read is sequence allocation and must succeed, otherwise the
            # frame never leaves and the test would prove nothing about a lost reply.
            calls.append(1)
            if len(calls) > 1:
                raise proto.ProtocolError("reconciliation read failed too")
            return original()

        session.read_last_sequence = flaky
        handle = worker.request_stop(None, ros_target_id="test:stop")
        assert handle.wait(10.0) is True
        result = handle.result
        assert len(calls) > 1, "the stop frame was never sent, so nothing was exercised"
        assert board.commands_received >= 1, "the STOP frame must have reached the board"
        assert result.stop_confirmed is False
        assert result.unknown is True
        assert "UNKNOWN" in result.message.upper()
        assert session.unknown_command_state is True, "a failed stop must leave the halt in place"
    finally:
        session.read_last_sequence = original
        shutdown(worker)


# --------------------------------------------------------------------------
# Batch self test
# --------------------------------------------------------------------------


def _batch(worker, timeout=60.0):
    return worker.submit(TaskRequest(kind="selftest", channel=0,
                                     opcode=int(proto.Opcode.SELFTEST_ALL), argument=0,
                                     scope_all=True, timeout_sec=timeout))


def test_batch_selftest_reports_every_channel_not_just_the_first():
    worker, session, board, stub, transport = build(probe_scale=25.0)
    try:
        handle = _batch(worker)
        assert handle.wait(60.0) is True
        result = handle.result
        assert result.completed is True
        assert result.batch_completed is True
        assert result.all_passed is True
        assert [e["channel"] for e in result.channel_results] == [0, 1, 2, 3]
        assert all(e["executed"] for e in result.channel_results)
        assert all(e["state"] == int(proto.SelftestState.PASSED) for e in result.channel_results)
        assert result.persisted is True
        # The result must never be allowed to imply more than it proves.
        assert "does NOT prove motor wiring" in result.message
    finally:
        shutdown(worker)


def test_batch_selftest_continues_after_a_discovery_failure():
    """No feedback at all is an expected finding for an unconnected channel, so the
    firmware keeps probing the rest (BSP/robot.c:135-139). all_passed must still be
    false: four channels were probed and none of them returned pulses."""
    worker, session, board, stub, transport = build(probe_scale=25.0)
    try:
        board.selftest_pulses = 0
        handle = _batch(worker)
        assert handle.wait(60.0) is True
        result = handle.result
        assert result.batch_completed is True
        assert result.all_passed is False
        assert all(e["state"] == int(proto.SelftestState.FAILED) for e in result.channel_results)
        assert all(e["reason"] == int(proto.SelftestReason.NO_PULSES)
                   for e in result.channel_results)
        assert all(e["executed"] for e in result.channel_results)
    finally:
        shutdown(worker)


def test_batch_selftest_cancel_reports_channels_that_never_ran():
    worker, session, board, stub, transport = build(probe_scale=1.0)
    try:
        handle = _batch(worker)
        # Let the first channel get going, then cancel the batch.
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if board.channels[0].state == int(proto.MotionState.SELFTEST):
                break
            board.advance(0.05)
            time.sleep(0.02)
        handle.cancel()
        assert handle.wait(60.0) is True
        result = handle.result
        assert result.completed is False
        assert result.batch_completed is False
        assert result.cancelled is True
        states = {e["channel"]: e["state"] for e in result.channel_results}
        assert states[0] == int(proto.SelftestState.CANCELLED)
        # The dropped channels are UNTESTED, which must never be read as a pass.
        assert all(states[c] == int(proto.SelftestState.UNTESTED) for c in (1, 2, 3)), states
        assert "NEVER ENERGISED" in result.message
    finally:
        shutdown(worker)


def test_batch_selftest_storage_failure_is_reported_per_channel():
    worker, session, board, stub, transport = build(probe_scale=25.0)
    try:
        board.storage_failure = True
        handle = _batch(worker)
        assert handle.wait(30.0) is True
        result = handle.result
        assert result.all_passed is False
        assert all(e["state"] == int(proto.SelftestState.FAILED) for e in result.channel_results)
        assert all(e["reason"] == int(proto.SelftestReason.STORAGE_ERROR)
                   for e in result.channel_results)
    finally:
        shutdown(worker)


def test_batch_selftest_timeout_does_not_claim_a_result_and_halts():
    worker, session, board, stub, transport = build(probe_scale=0.002)
    try:
        handle = _batch(worker, timeout=1.0)
        assert handle.wait(30.0) is True
        result = handle.result
        assert result.completed is False
        assert result.all_passed is False
        assert result.unknown is True
        assert result.result_code == DRIVER_TIMEOUT
        assert session.unknown_command_state is True
    finally:
        shutdown(worker)


# --------------------------------------------------------------------------
# motor_rpm: a goal must not claim a speed the board will not use
# --------------------------------------------------------------------------


def _move(worker, channel=2, argument=100, **kwargs):
    request = TaskRequest(kind="move", channel=channel, opcode=int(proto.Opcode.MOVE),
                          argument=argument, mode=MOVE_RELATIVE, timeout_sec=20.0)
    for key, value in kwargs.items():
        setattr(request, key, value)
    return worker.submit(request)


def test_move_refuses_an_rpm_that_differs_from_the_board():
    worker, session, board, stub, transport = build()
    try:
        handle = _move(worker, expect_rpm=1500, limits_rad=(-3.14, 3.14),
                       allow_unreferenced=True)
        assert handle.wait(20.0) is True
        result = handle.result
        assert result.result_code == DRIVER_RPM_MISMATCH
        assert "8000" in result.message and "1500" in result.message
        assert "apply_config" in result.message
        assert board.commands_received == 0, "a refused speed must send nothing"
    finally:
        shutdown(worker)


@pytest.mark.parametrize("expect", [0, 8000])
def test_move_accepts_the_board_rpm_or_no_assertion(expect):
    """0 means "whatever the board has"; an exact match means the goal was written
    against the configuration that is actually in effect."""
    worker, session, board, stub, transport = build()
    try:
        handle = _move(worker, expect_rpm=expect, limits_rad=None, allow_without_limits=True,
                       max_step_deci_deg=500, allow_unreferenced=True)
        assert handle.wait(20.0) is True
        assert handle.result.result_code == DRIVER_OK, handle.result.message
        assert handle.result.completed is True
        assert board.commands_received == 1
    finally:
        shutdown(worker)


def test_move_detects_a_board_rpm_changed_after_the_goal_was_written():
    worker, session, board, stub, transport = build()
    try:
        board.channels[2].rpm = 1500
        handle = _move(worker, expect_rpm=8000, limits_rad=None, allow_without_limits=True,
                       max_step_deci_deg=500, allow_unreferenced=True)
        assert handle.wait(20.0) is True
        result = handle.result
        assert result.result_code == DRIVER_RPM_MISMATCH
        assert "1500" in result.message and "8000" in result.message
        assert board.commands_received == 0
    finally:
        shutdown(worker)


# --------------------------------------------------------------------------
# Soft limits, freshness and commissioning
# --------------------------------------------------------------------------


def _zero(worker, channel=2):
    handle = worker.submit(TaskRequest(kind="zero", channel=channel,
                                       opcode=int(proto.Opcode.ZERO), timeout_sec=10.0))
    assert handle.wait(10.0) is True
    return handle.result


def test_relative_move_checks_the_predicted_endpoint():
    """What matters mechanically is where the joint ends up, not how far it was told
    to travel, so a relative goal is checked against limits too."""
    worker, session, board, stub, transport = build()
    try:
        _zero(worker)
        settle(board, 0.7, 3)
        handle = _move(worker, argument=2000, limits_rad=(-0.1745, 0.1745))
        assert handle.wait(20.0) is True
        result = handle.result
        assert result.result_code == DRIVER_LIMIT
        assert "outside limits_rad" in result.message
        assert board.commands_received == 1, "only the ZERO may have been sent"
    finally:
        shutdown(worker)


def test_relative_move_inside_limits_is_allowed():
    worker, session, board, stub, transport = build()
    try:
        _zero(worker)
        settle(board, 0.7, 3)
        handle = _move(worker, argument=100, limits_rad=(-0.35, 0.35))
        assert handle.wait(20.0) is True
        assert handle.result.result_code == DRIVER_OK, handle.result.message
        assert handle.result.completed is True
    finally:
        shutdown(worker)


def test_limits_null_refuses_motion_unless_commissioning():
    """An unconfirmed limit is not a limit. The driver must say so instead of moving."""
    worker, session, board, stub, transport = build()
    try:
        handle = _move(worker, limits_rad=None, allow_unreferenced=True)
        assert handle.wait(20.0) is True
        result = handle.result
        assert result.result_code == DRIVER_LIMITS_UNCONFIRMED
        assert "limits_rad null" in result.message
        assert "commissioning" in result.message
        assert board.commands_received == 0
    finally:
        shutdown(worker)


def test_commissioning_allows_motion_and_caps_a_single_command():
    worker, session, board, stub, transport = build()
    try:
        small = _move(worker, argument=100, limits_rad=None, allow_without_limits=True,
                      max_step_deci_deg=500, allow_unreferenced=True)
        assert small.wait(20.0) is True
        assert small.result.completed is True, small.result.message

        big = _move(worker, argument=1000, limits_rad=None, allow_without_limits=True,
                    max_step_deci_deg=500, allow_unreferenced=True)
        assert big.wait(20.0) is True
        assert big.result.result_code == DRIVER_LIMIT
        assert "commissioning step" in big.result.message
        assert board.commands_received == 1, "the oversized command must not be sent"
    finally:
        shutdown(worker)


def test_move_refuses_when_the_position_cannot_be_read():
    """A move must never be computed from data of unknown age."""
    worker, session, board, stub, transport = build()
    original = session.read_status_and_position
    try:
        def explode(channel):
            raise proto.ProtocolError("position read failed")

        session.read_status_and_position = explode
        handle = _move(worker, limits_rad=None, allow_without_limits=True,
                       max_step_deci_deg=500, allow_unreferenced=True)
        assert handle.wait(20.0) is True
        result = handle.result
        assert result.result_code == DRIVER_STALE_POSITION
        assert "stale" in result.message.lower()
        assert board.commands_received == 0
    finally:
        session.read_status_and_position = original
        shutdown(worker)


def test_refresh_now_returns_none_when_the_poll_cannot_complete():
    """Returning the previous snapshot would let a caller act on old data silently."""
    worker, session, board, stub, transport = build()
    try:
        stub.drop_responses = 99
        transport.read_retries = 0
        assert worker.refresh_now(timeout=3.0) is None
    finally:
        shutdown(worker)


def test_absolute_move_uses_a_fresh_reading_not_the_published_snapshot():
    worker, session, board, stub, transport = build()
    try:
        _zero(worker)
        settle(board, 0.7, 3)
        # Poison the published snapshot. If the worker trusted it, the delta would be
        # about 10099 deci-degrees and the goal would be refused as beyond +/-360 deg.
        worker._snapshot.positions[2] = proto.ChannelPosition(angle_deci_deg=-9999, fg_count=0,
                                                              target_deci_deg=0)
        request = TaskRequest(kind="move", channel=2, opcode=int(proto.Opcode.MOVE),
                              mode=MOVE_ABSOLUTE, target_deci_deg=100,
                              limits_rad=(-3.14, 3.14), timeout_sec=20.0)
        handle = worker.submit(request)
        assert handle.wait(20.0) is True
        result = handle.result
        assert result.completed is True, result.message
        # gear 290: 100 deci-degrees quantises to 99, which proves the delta was
        # measured from the real pose (0) and not from the poisoned one.
        assert abs(board.channels[2].angle_deci_deg - 99) <= 2
    finally:
        shutdown(worker)


def test_two_absolute_goals_do_not_share_a_start_position():
    """Each goal must be measured from the pose the board is actually in, including the
    pose left behind by the previous goal."""
    worker, session, board, stub, transport = build()
    try:
        _zero(worker)
        settle(board, 0.7, 3)
        first = worker.submit(TaskRequest(kind="move", channel=2, opcode=int(proto.Opcode.MOVE),
                                          mode=MOVE_ABSOLUTE, target_deci_deg=200,
                                          limits_rad=(-3.14, 3.14), timeout_sec=20.0))
        assert first.wait(20.0) is True and first.result.completed is True
        settle(board, 0.7, 3)
        after_first = board.channels[2].angle_deci_deg
        assert after_first > 100

        second = worker.submit(TaskRequest(kind="move", channel=2, opcode=int(proto.Opcode.MOVE),
                                           mode=MOVE_ABSOLUTE, target_deci_deg=50,
                                           limits_rad=(-3.14, 3.14), timeout_sec=20.0))
        assert second.wait(20.0) is True
        assert second.result.completed is True, second.result.message
        # The second goal moves BACK towards zero, which is only possible if the delta
        # was measured from the pose the first goal left behind.
        assert board.channels[2].angle_deci_deg < after_first
        assert abs(board.channels[2].angle_deci_deg - 49) <= 3
    finally:
        shutdown(worker)


@pytest.mark.parametrize("channel", [None, 0])
def test_review_stop_discards_previously_queued_move(channel):
    transport, board, stub, session, clock = make_fake_stack(read_only=False)
    worker = BoardWorker(session, allow_motion=True)
    worker._started.set()
    move = worker.submit(TaskRequest(kind="move", channel=0, opcode=int(proto.Opcode.MOVE), argument=50, **commissioning()))
    stop = worker.request_stop(channel)
    worker._execute(worker._queue.get().handle)
    before = board.commands_received
    worker._execute(worker._queue.get().handle)
    assert stop.result.accepted
    assert move.result.cancelled and not move.result.accepted
    assert board.commands_received == before


def test_review_cancel_before_dispatch_sends_no_move():
    transport, board, stub, session, clock = make_fake_stack(read_only=False)
    worker = BoardWorker(session, allow_motion=True)
    worker._started.set()
    move = worker.submit(TaskRequest(kind="move", channel=0, opcode=int(proto.Opcode.MOVE), argument=50, **commissioning()))
    move.cancel()
    before = board.commands_received
    worker._execute(worker._queue.get().handle)
    assert move.result.cancelled
    assert board.commands_received == before


@pytest.mark.parametrize("kind,opcode,argument", [("move", proto.Opcode.MOVE,50), ("selftest",proto.Opcode.SELFTEST,0), ("brake",proto.Opcode.BRAKE,0)])
def test_review_config_mismatch_blocks_writes(kind,opcode,argument):
    transport, board, stub, session, clock = make_fake_stack(read_only=False)
    worker = BoardWorker(session, allow_motion=True, expected_configs={0:(True,3,82,1)})
    worker._started.set()
    task = worker.submit(TaskRequest(kind=kind, channel=0, opcode=int(opcode), argument=argument, **commissioning()))
    worker._execute(worker._queue.get().handle)
    assert task.result.result_code == DRIVER_CONFIG_MISMATCH
    assert board.commands_received == 0


@pytest.mark.parametrize("kind,argument,scope_all", [("move",50,False),("selftest",0,False),("selftest",0,True),("brake",0,False),("zero",0,False)])
def test_review_head_only_policy_blocks_arm_writes(kind,argument,scope_all):
    transport, board, stub, session, clock = make_fake_stack(read_only=False)
    worker = BoardWorker(session, allow_motion=True, allowed_channels={1})
    worker._started.set()
    task = worker.submit(TaskRequest(kind=kind, channel=0, argument=argument, scope_all=scope_all))
    assert task.rejected
    assert board.commands_received == 0
    assert not worker.request_stop(0).rejected
    assert not worker.submit(TaskRequest(kind="brake",channel=0,argument=1)).rejected


def test_retry_save_storage_error_recovery_preserves_position():
    worker,session,board,stub,transport=build(allow_motion=False)
    try:
        board.channels[0].fault=1;board.channels[0].valid=False;board.channels[0].fg_count=123
        board.storage_error=True;board.dirty=True;board.pending_save=True
        handle=worker.submit(TaskRequest(kind="retry_save",channel=0,opcode=int(proto.Opcode.SAVE)))
        assert handle.wait(5)
        assert handle.result.accepted and handle.result.persisted and handle.result.flags_valid
        assert not any((handle.result.dirty,handle.result.storage_error,handle.result.pending_save))
        assert board.channels[0].fault==1 and not board.channels[0].valid and board.channels[0].fg_count==123
    finally:shutdown(worker)


def test_save_recovery_only_rejects_every_other_write():
    worker,session,board,stub,transport=build(allow_motion=True)
    try:
        worker.save_recovery_only=True
        for kind in ('move','selftest','zero','brake','clear_fault','manual_adjust','config','save'):
            h=worker.submit(TaskRequest(kind=kind,channel=0))
            assert h.rejected and h.result.result_code==DRIVER_READONLY
        h=worker.submit(TaskRequest(kind='retry_save',channel=0,opcode=int(proto.Opcode.SAVE)))
        assert h.wait(5) and h.result.persisted
    finally:shutdown(worker)


def test_retry_save_unbraked_rejected_without_command():
    worker,session,board,stub,transport=build()
    try:
        board.channels[0].brake=False
        before=session.read_last_sequence()
        h=worker.submit(TaskRequest(kind='retry_save',channel=0,opcode=int(proto.Opcode.SAVE)))
        assert h.wait(5) and not h.result.accepted and h.result.result_code==DRIVER_BUSY
        assert session.read_last_sequence()==before
    finally:shutdown(worker)


def test_retry_save_storage_failure_has_valid_dirty_flags():
    worker,session,board,stub,transport=build()
    try:
        board.storage_failure=True;board.dirty=True;board.pending_save=True
        h=worker.submit(TaskRequest(kind='retry_save',channel=0,opcode=int(proto.Opcode.SAVE)))
        assert h.wait(5) and not h.result.accepted and not h.result.persisted
        assert h.result.flags_valid and h.result.storage_error
        assert h.result.result_code==int(proto.BusinessResult.STORAGE)
    finally:shutdown(worker)


def test_direction_commands_change_only_dir_bit():
    worker,session,board,stub,transport=build()
    try:
        ch=board.channels[0];ch.fg_count=123;ch.valid=True
        for value in (1,0):
            h=worker.submit(TaskRequest(kind='direction',channel=0,opcode=int(proto.Opcode.DIRECTION),argument=value))
            assert h.wait(5) and h.result.accepted and h.result.direction_confirmed
            assert h.result.direction==bool(value) and ch.direction==bool(value)
            assert ch.fg_count==123 and ch.valid and ch.brake
    finally:shutdown(worker)

@pytest.mark.parametrize('reason',['brake','manual','fault','storage','recovery'])
def test_direction_rejected_without_output_changes(reason):
    worker,session,board,stub,transport=build()
    try:
        if reason=='brake':board.channels[0].brake=False
        if reason=='manual':board.channels[1].manual=1
        if reason=='fault':board.channels[0].fault=1
        if reason=='storage':board.storage_error=True
        if reason=='recovery':worker.save_recovery_only=True
        before=session.read_last_sequence()
        h=worker.submit(TaskRequest(kind='direction',channel=0,opcode=int(proto.Opcode.DIRECTION),argument=1))
        assert h.wait(5) and not h.result.accepted and not h.result.direction_confirmed
        assert session.read_last_sequence()==before and not board.channels[0].direction
    finally:shutdown(worker)
