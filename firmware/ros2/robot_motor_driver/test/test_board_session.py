"""Board session tests: identity binding, sequences, reconciliation, read-only.

The reconciliation tests are the important ones. A lost reply is not a failed
command and not a successful one: the board records every ACCEPTED request in
0x0036/0x0037, so reading those two registers is what turns "no idea" into either
"a definite result" or "UNKNOWN, stop scheduling".
"""

import pytest

from robot_motor_driver import protocol as proto
from robot_motor_driver.board_session import (DRIVER_HALTED, DRIVER_TIMEOUT, DRIVER_UNKNOWN,
                                              BoardSession, CommandOutcome, RequestLog)
from robot_motor_driver.fake_board import make_fake_stack


def stack(**kwargs):
    return make_fake_stack(**kwargs)


# --------------------------------------------------------------------------
# Identity
# --------------------------------------------------------------------------


def test_identify_binds_uid_and_protocol_revision():
    transport, board, stub, session, clock = stack()
    assert session.identified is True
    assert session.uid == board.uid
    assert session.command_protocol == proto.COMMAND_PROTOCOL_REVISION


def test_uid_mismatch_is_refused_and_blocks_writes():
    transport, board, stub, session, clock = stack(
        expected_uid="FFFFFFFFFFFFFFFFFFFFFFFF", identify=False)
    with pytest.raises(proto.IdentityError) as info:
        session.identify()
    assert "does not match" in str(info.value)
    assert session.identified is False
    session.read_only = False
    with pytest.raises(proto.IdentityError):
        session.send_command(proto.Opcode.STOP_ALL, 0, 0)


def test_wrong_command_protocol_revision_is_refused():
    """A board that is not on revision 2 has no single-frame command support."""
    transport, board, stub, session, clock = stack(identify=False)
    board.command_protocol = 1
    with pytest.raises(proto.IdentityError) as info:
        session.identify()
    assert "revision" in str(info.value)


def test_expected_uid_is_normalised_to_upper_case():
    transport, board, stub, session, clock = stack(
        expected_uid="c23b378115b092083830364a")
    assert session.expected_uid == "C23B378115B092083830364A"


# --------------------------------------------------------------------------
# Read-only
# --------------------------------------------------------------------------


@pytest.mark.parametrize("opcode", [proto.Opcode.CONFIG, proto.Opcode.ZERO, proto.Opcode.MOVE,
                                    proto.Opcode.STOP, proto.Opcode.STOP_ALL, proto.Opcode.BRAKE,
                                    proto.Opcode.DIRECTION, proto.Opcode.SAVE,
                                    proto.Opcode.CLEAR_FAULT, proto.Opcode.SELFTEST,
                                    proto.Opcode.SELFTEST_ALL])
def test_read_only_refuses_every_write_including_stop(opcode):
    """Read-only means read-only: even STOP is intercepted, and the refusal says so."""
    transport, board, stub, session, clock = stack(read_only=True)
    with pytest.raises(proto.ReadOnlyViolation) as info:
        session.send_command(opcode, 0, 0)
    assert "read-only" in str(info.value)
    assert board.commands_received == 0


def test_read_only_refuses_configuration_staging():
    transport, board, stub, session, clock = stack(read_only=True)
    with pytest.raises(proto.ReadOnlyViolation):
        session.stage_board_config(0, 1, 1, 25000)
    with pytest.raises(proto.ReadOnlyViolation):
        session.stage_channel_config(0, proto.ChannelConfigValues(True, 1, 516, 0, 8000))


def test_read_only_still_allows_every_read():
    transport, board, stub, session, clock = stack(read_only=True)
    snapshot = session.poll()
    assert snapshot.communication_ok is True
    assert snapshot.uid == board.uid
    assert len(snapshot.statuses) == proto.CHANNEL_COUNT
    assert len(snapshot.configs) == proto.CHANNEL_COUNT
    assert session.read_board_info().schema == proto.FLASH_SCHEMA


# --------------------------------------------------------------------------
# Sequence allocation
# --------------------------------------------------------------------------


def test_sequence_starts_after_the_board_last_one():
    transport, board, stub, session, clock = stack()
    board.last_seq = 41
    assert session.allocate_sequence() == 42


def test_sequence_wraps_past_zero():
    transport, board, stub, session, clock = stack()
    board.last_seq = 0xFFFF
    assert session.allocate_sequence() == 1


def test_sequence_skips_entries_still_in_the_local_dedupe_mirror():
    """Reusing a cached sequence with new parameters would be rejected, so the
    allocator steps over anything it has already used."""
    transport, board, stub, session, clock = stack()
    board.last_seq = 10
    session._remember(11, int(proto.Opcode.STOP), 0, 0, 0)
    assert session.allocate_sequence() == 12


def test_zero_sequence_is_never_sent():
    transport, board, stub, session, clock = stack()
    session.read_only = False
    with pytest.raises(ValueError):
        session.send_command(proto.Opcode.STOP, 0, 0, sequence=0)


def test_sequence_outside_the_window_is_refused_locally():
    transport, board, stub, session, clock = stack()
    session.read_only = False
    board.last_seq = 1000
    board.have_seq = True
    with pytest.raises(proto.ProtocolError) as info:
        session.send_command(proto.Opcode.STOP, 0, 0, sequence=5)
    assert "window" in str(info.value)
    assert board.commands_received == 0


# --------------------------------------------------------------------------
# Reconciliation
# --------------------------------------------------------------------------


def test_lost_reply_after_execution_is_reconciled_as_accepted():
    """The reply vanished but the board took the command: 0x0036 proves it.

    Only the COMMAND reply is dropped, so the reconciliation reads that follow are
    unaffected and the outcome can be established instead of guessed.
    """
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    stub.drop_command_responses = 1
    outcome = session.send_command(proto.Opcode.STOP_ALL, 0, 0, retries=0,
                                   ros_target_id="t1")
    assert outcome.accepted is True
    assert outcome.result == int(proto.BusinessResult.OK)
    assert outcome.reconciled is True
    assert outcome.unknown is False
    assert board.last_seq == outcome.sequence
    assert session.unknown_command_state is False


def test_lost_reply_without_execution_is_reported_as_not_accepted():
    """No reply AND no record on the board: the command did not happen, so a retry
    with the SAME sequence is safe and no UNKNOWN state is needed."""
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    original = board.handle_adu

    def swallow_commands(frame):
        if len(frame) > 1 and frame[1] == 16:
            return None
        return original(frame)

    board.handle_adu = swallow_commands
    outcome = session.send_command(proto.Opcode.STOP_ALL, 0, 0, retries=0,
                                   ros_target_id="t2")
    assert outcome.accepted is False
    assert outcome.result == DRIVER_TIMEOUT
    assert outcome.unknown is False
    assert session.unknown_command_state is False
    assert board.commands_received == 0


def test_replayed_command_with_different_arguments_yields_sequence_result():
    """Same sequence, different parameters -> R_SEQUENCE, surfaced as exception 3
    and resolved through 0x0037 because the board did record the request."""
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    first = session.send_command(proto.Opcode.STOP, 0, 0, ros_target_id="a")
    assert first.accepted is True
    second = session.send_command(proto.Opcode.STOP, 1, 0, sequence=first.sequence,
                                  ros_target_id="b")
    assert second.accepted is False
    assert second.result == int(proto.BusinessResult.SEQUENCE)
    assert board.commands_executed == 1


def test_identical_replay_returns_the_cached_result_with_zero_side_effects():
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    first = session.send_command(proto.Opcode.STOP_ALL, 0, 0, ros_target_id="a")
    executed_before = board.commands_executed
    second = session.send_command(proto.Opcode.STOP_ALL, 0, 0, sequence=first.sequence,
                                  ros_target_id="a-retry")
    assert second.accepted is True
    assert second.result == int(proto.BusinessResult.OK)
    assert board.commands_executed == executed_before
    assert board.commands_replayed >= 1


def test_reconcile_treats_exception_three_with_unchanged_last_seq_as_sequence_rejection():
    """The only reliable discriminator for the ambiguous 0x03: if 0x0036 did not
    move, the sequence itself was rejected and nothing executed."""
    transport, board, stub, session, clock = stack(read_only=False)
    session._last_sequence = 500
    outcome = CommandOutcome(sequence=501, opcode=int(proto.Opcode.MOVE), channel=0, argument=100)
    session.read_last_sequence = lambda: (500, 0)     # unchanged by the board
    resolved = session._reconcile(outcome, int(proto.ModbusException.ILLEGAL_DATA_VALUE))
    assert resolved.result == int(proto.BusinessResult.SEQUENCE)
    assert resolved.unknown is False
    assert session.unknown_command_state is False


def test_reconcile_treats_exception_four_without_a_record_as_unknown():
    """0x04 with no recorded sequence should not happen; guessing between UNREADY
    and STORAGE would be fabrication, so it halts instead."""
    transport, board, stub, session, clock = stack(read_only=False)
    outcome = CommandOutcome(sequence=12, opcode=int(proto.Opcode.MOVE), channel=0, argument=100)
    session.read_last_sequence = lambda: (11, 0)
    resolved = session._reconcile(outcome, int(proto.ModbusException.SLAVE_DEVICE_FAILURE))
    assert resolved.unknown is True
    assert resolved.result == DRIVER_UNKNOWN
    assert session.unknown_command_state is True


def test_reconcile_failure_marks_unknown_and_halts_scheduling():
    transport, board, stub, session, clock = stack(read_only=False)

    def broken():
        raise proto.ModbusTimeout("no reply")

    session.read_last_sequence = broken
    outcome = CommandOutcome(sequence=3, opcode=int(proto.Opcode.MOVE), channel=0, argument=100)
    resolved = session._reconcile(outcome, None)
    assert resolved.unknown is True
    assert session.unknown_command_state is True
    with pytest.raises(proto.ProtocolError) as info:
        session.send_command(proto.Opcode.MOVE, 0, 100)
    assert "halted" in str(info.value) or "UNKNOWN" in str(info.value)


def test_acknowledge_unknown_clears_the_halt_without_resuming():
    transport, board, stub, session, clock = stack(read_only=False)
    session.mark_unknown("test")
    detail = session.acknowledge_unknown()
    assert detail == "test"
    assert session.unknown_command_state is False
    clock.advance(1.0)
    outcome = session.send_command(proto.Opcode.STOP_ALL, 0, 0)
    assert outcome.accepted is True, "acknowledging only unlocks; it does not replay anything"
    assert board.commands_received == 1


# --------------------------------------------------------------------------
# Request log
# --------------------------------------------------------------------------


def test_request_identity_is_logged_before_the_frame_leaves():
    """A lost reply must still leave the sequence behind, or a retry cannot reuse it."""
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    stub.drop_command_responses = 1
    outcome = session.send_command(proto.Opcode.MOVE, 0, 900, ros_target_id="goal-42")
    entries = session.request_log.find_by_sequence(outcome.sequence)
    assert len(entries) == 1
    entry = entries[0]
    assert entry["opcode"] == int(proto.Opcode.MOVE)
    assert entry["channel"] == 0
    assert entry["argument"] == 900
    assert entry["uid"] == board.uid
    assert entry["ros_target_id"] == "goal-42"
    assert "seq=%d" % outcome.sequence in entry["request_id"]


def test_request_log_is_bounded():
    log = RequestLog(capacity=3)
    for index in range(10):
        log.record({"sequence": index})
    assert len(log) == 3
    assert [entry["sequence"] for entry in log.entries()] == [7, 8, 9]


# --------------------------------------------------------------------------
# Reconnection
# --------------------------------------------------------------------------


def test_reconnect_clears_identity_and_dedupe_and_does_not_resume():
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    session.send_command(proto.Opcode.STOP_ALL, 0, 0)
    session.reconnect()
    assert session.identified is False
    assert session.reconnects == 1
    assert session.unknown_command_state is True
    assert session.cached_result(1, int(proto.Opcode.STOP_ALL), 0, 0) is None
    # Nothing is resumed: a new command is refused until the state is acknowledged.
    with pytest.raises(proto.ProtocolError):
        session.send_command(proto.Opcode.MOVE, 0, 100)


def test_reconnect_requires_re_identification_before_writes():
    transport, board, stub, session, clock = stack(read_only=False)
    session.reconnect()
    session.acknowledge_unknown()
    with pytest.raises(proto.IdentityError):
        session.send_command(proto.Opcode.STOP_ALL, 0, 0)
    session.identify()
    clock.advance(1.0)
    assert session.send_command(proto.Opcode.STOP_ALL, 0, 0).accepted is True


# --------------------------------------------------------------------------
# Snapshots
# --------------------------------------------------------------------------


def test_poll_reports_communication_failure_instead_of_raising():
    transport, board, stub, session, clock = stack()
    stub.drop_responses = 99
    transport.read_retries = 0
    snapshot = session.poll()
    assert snapshot.communication_ok is False
    assert snapshot.error


def test_snapshot_exposes_busy_channels_and_save_flags():
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    session.send_command(proto.Opcode.MOVE, 0, 900)
    snapshot = session.poll()
    assert snapshot.busy_channels == [0]
    assert snapshot.info.dirty is True
    assert snapshot.save_clean is False


# --------------------------------------------------------------------------
# Stopping while the driver is halted after an UNKNOWN outcome
# --------------------------------------------------------------------------


def test_stop_all_is_attempted_while_unknown_and_does_not_clear_the_halt():
    """The halt exists to stop NEW motion. Refusing the one command that can stop the
    mechanism would leave the robot moving with no software way to intervene, which is
    strictly worse than the problem the halt is meant to contain."""
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    session.mark_unknown("simulated unknown outcome")
    assert session.unknown_command_state is True

    outcome = session.send_command(proto.Opcode.STOP_ALL, 0, 0, allow_while_unknown=True)

    assert outcome.accepted is True
    assert outcome.unknown is False
    # Sending a stop is not an acknowledgement: scheduling stays halted.
    assert session.unknown_command_state is True
    assert board.commands_received == 1
    assert all(ch.state not in proto.BUSY_STATES for ch in board.channels)
    assert all(ch.brake for ch in board.channels)


def test_single_channel_stop_is_attempted_while_unknown():
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    session.mark_unknown("simulated")
    outcome = session.send_command(proto.Opcode.STOP, 1, 0, allow_while_unknown=True)
    assert outcome.accepted is True
    assert board.commands_received == 1
    assert board.channels[1].state not in proto.BUSY_STATES
    assert board.channels[1].brake is True
    assert session.unknown_command_state is True


@pytest.mark.parametrize("opcode", [proto.Opcode.CONFIG, proto.Opcode.ZERO, proto.Opcode.MOVE,
                                    proto.Opcode.SAVE, proto.Opcode.DIRECTION,
                                    proto.Opcode.SELFTEST, proto.Opcode.CLEAR_FAULT,
                                    proto.Opcode.BRAKE, proto.Opcode.SELFTEST_ALL])
def test_every_other_write_stays_refused_while_unknown(opcode):
    """Asking for the bypass on a non-stop opcode must not open the gate. BRAKE(1) is
    refused even though the firmware treats arg=1 as a stop: only STOP/STOP_ALL pass."""
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    session.mark_unknown("simulated")
    with pytest.raises(proto.ProtocolError) as info:
        session.send_command(opcode, 0, 1, allow_while_unknown=True)
    assert "not a stop command" in str(info.value) or "refused" in str(info.value)
    assert board.commands_received == 0


def test_stop_is_still_refused_in_read_only_mode_even_while_unknown():
    transport, board, stub, session, clock = stack(read_only=True)
    session.mark_unknown("simulated")
    with pytest.raises(proto.ReadOnlyViolation):
        session.send_command(proto.Opcode.STOP_ALL, 0, 0, allow_while_unknown=True)
    assert board.commands_received == 0


def test_stop_without_any_reply_is_unknown_and_never_reported_as_stopped():
    """A frame that left the wire is not evidence the mechanism stopped."""
    transport, board, stub, session, clock = stack(read_only=False)
    clock.advance(1.0)
    original = session.read_last_sequence
    calls = []
    try:
        # Drop EVERY command reply: the transport retries the identical frame with the
        # same sequence, so dropping only one still ends in a normal accepted reply.
        stub.drop_command_responses = 99

        def flaky():
            # Call 1 is sequence allocation; it must succeed so the frame really goes
            # out. Later calls are the reconciliation read, which is what fails here.
            calls.append(1)
            if len(calls) > 1:
                raise proto.ProtocolError("reconciliation read failed too")
            return original()

        session.read_last_sequence = flaky
        outcome = session.send_command(proto.Opcode.STOP_ALL, 0, 0, allow_while_unknown=True)

        assert len(calls) > 1, "the frame was never sent, so nothing was exercised"
        assert board.commands_received >= 1
        assert outcome.unknown is True
        assert outcome.accepted is False
        assert session.unknown_command_state is True
        assert "UNKNOWN" in outcome.detail.upper()
    finally:
        session.read_last_sequence = original
