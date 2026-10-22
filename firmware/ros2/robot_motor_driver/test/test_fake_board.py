"""Simulated-board semantics tests.

These pin the behaviour the driver relies on: quiet requirements, the dedupe
cache, the dirty/pending/storage lifecycle, the quantisation trap, and the fact
that a rejected command still consumes a sequence. The simulator is a model of the
firmware, so these tests are evidence about the DRIVER's expectations, not about
real hardware.
"""

import pytest

from robot_motor_driver import protocol as proto
from robot_motor_driver.fake_board import FakeBoard, FakeClock, SerialStub, make_fake_stack


def quiet_stack(**kwargs):
    """A stack whose board has been idle long enough to accept commands."""
    transport, board, stub, session, clock = make_fake_stack(**kwargs)
    clock.advance(1.0)
    return transport, board, stub, session, clock


def send(session, opcode, channel=0, argument=0, sequence=None):
    return session.send_command(opcode, channel, argument, sequence=sequence)


def raw_command(transport, sequence, opcode, channel=0, argument=0):
    """Send a command frame straight onto the bus, bypassing the session's own
    sequence guards, so board-side rules can be provoked on purpose."""
    high, low = proto.pack_i32(argument)
    pdu = proto.write_multi_pdu(proto.REG_COMMAND,
                                [sequence & 0xFFFF, int(opcode), channel, high, low])
    return transport.exchange(pdu)


def settle(board, seconds=3.0, steps=6):
    """Advance the simulated board in several small steps.

    One big jump is not enough: motion and self-test state machines move one phase
    per evaluation, exactly like the firmware's 10 ms control loop needs several
    passes to run a probe to completion.
    """
    for _ in range(steps):
        board.advance(seconds / float(steps))


def exception_code(callable_, *args, **kwargs):
    with pytest.raises(proto.ModbusExceptionResponse) as info:
        callable_(*args, **kwargs)
    return info.value.code


# --------------------------------------------------------------------------
# Interlocks
# --------------------------------------------------------------------------


def test_move_requires_a_quiet_board():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    assert send(session, proto.Opcode.MOVE, 0, 900).accepted is True
    # Immediately after acceptance the board is busy, so a second move is refused.
    second = send(session, proto.Opcode.MOVE, 1, 900)
    assert second.accepted is False
    assert second.result == int(proto.BusinessResult.BUSY)


def test_move_argument_out_of_range_is_a_value_error():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    outcome = send(session, proto.Opcode.MOVE, 0, proto.MAX_MOVE_DECI_DEG + 1)
    assert outcome.accepted is False
    assert outcome.result == int(proto.BusinessResult.VALUE)


def test_disabled_or_ungeared_channel_is_not_ready():
    channels = [dict(DEFAULT) for DEFAULT in (
        {"enabled": False, "role": 0, "gear": 0, "inverted": 0, "rpm": 8000},
        {"enabled": True, "role": 1, "gear": 0, "inverted": 0, "rpm": 8000},
        {"enabled": True, "role": 2, "gear": 516, "inverted": 0, "rpm": 8000},
        {"enabled": True, "role": 4, "gear": 290, "inverted": 0, "rpm": 8000},
    )]
    transport, board, stub, session, clock = quiet_stack(read_only=False, channels=channels)
    for channel in (0, 1):
        outcome = send(session, proto.Opcode.MOVE, channel, 900)
        assert outcome.accepted is False
        assert outcome.result == int(proto.BusinessResult.UNREADY)


def test_stop_input_blocks_motion_but_never_blocks_stop():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    board.stop_input = True
    assert send(session, proto.Opcode.MOVE, 0, 900).result == int(proto.BusinessResult.BUSY)
    assert send(session, proto.Opcode.SELFTEST, 0, 0).result == int(proto.BusinessResult.BUSY)
    assert send(session, proto.Opcode.STOP, 0).accepted is True
    assert send(session, proto.Opcode.STOP_ALL).accepted is True
    assert send(session, proto.Opcode.BRAKE, 0, 1).accepted is True
    assert send(session, proto.Opcode.BRAKE, 0, 0).result == int(proto.BusinessResult.BUSY)


def test_direction_command_needs_a_quiet_channel():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    assert send(session, proto.Opcode.DIRECTION, 0, 1).accepted is True
    assert board.channels[0].direction is True
    board.channels[0].last_edge = clock()      # a fresh edge makes it unquiet
    assert send(session, proto.Opcode.DIRECTION, 0, 0).result == int(proto.BusinessResult.BUSY)


def test_invalid_direction_and_brake_arguments_are_rejected():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    assert send(session, proto.Opcode.DIRECTION, 0, 2).result == int(proto.BusinessResult.VALUE)
    assert send(session, proto.Opcode.BRAKE, 0, 7).result == int(proto.BusinessResult.VALUE)
    assert send(session, proto.Opcode.SELFTEST, 0, 3).result == int(proto.BusinessResult.VALUE)


# --------------------------------------------------------------------------
# Direction and quantisation
# --------------------------------------------------------------------------


def test_move_derives_direction_from_sign_and_inverted():
    """MOVE overwrites any DIRECTION setting: direction = (arg < 0) XOR inverted."""
    plain = [{"enabled": True, "role": 1, "gear": 516, "inverted": 0, "rpm": 8000},
             {"enabled": True, "role": 2, "gear": 516, "inverted": 0, "rpm": 8000},
             {"enabled": True, "role": 4, "gear": 290, "inverted": 0, "rpm": 8000},
             {"enabled": True, "role": 6, "gear": 82, "inverted": 0, "rpm": 8000}]
    transport, board, stub, session, clock = quiet_stack(read_only=False, channels=plain)
    assert send(session, proto.Opcode.MOVE, 0, -900).accepted is True
    assert board.channels[0].direction is True, "negative argument, not inverted"
    assert send(session, proto.Opcode.MOVE, 1, 900).accepted is False  # board is busy now

    flipped = [dict(channel) for channel in plain]
    flipped[0]["inverted"] = 1
    transport, board, stub, session, clock = quiet_stack(read_only=False, channels=flipped)
    assert send(session, proto.Opcode.MOVE, 0, -900).accepted is True
    assert board.channels[0].direction is False, "inverted flips a negative argument"

    # An explicit DIRECTION setting is overwritten by the next MOVE.
    settle(board)
    assert send(session, proto.Opcode.DIRECTION, 0, 1).accepted is True
    assert board.channels[0].direction is True
    settle(board)
    # inverted=1 with a NEGATIVE argument: direction = (arg < 0) XOR inverted
    #                            = True XOR True = False, i.e. the MOVE overwrote the
    # explicit DIRECTION=1 that was just set.
    assert send(session, proto.Opcode.MOVE, 0, -900).accepted is True
    assert board.channels[0].direction is False


def test_argument_that_quantises_to_zero_still_reports_done():
    """The trap the driver must guard against: at gear 82 an argument of 0.1 deg
    produces zero steps, yet the board walks to DONE. Nothing moved."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    gear = board.channels[3].gear
    assert gear == 82
    assert proto.deci_deg_to_steps(1, gear) == 0
    assert proto.quantises_to_zero(1, gear) is True
    assert proto.quantises_to_zero(900, gear) is False
    outcome = send(session, proto.Opcode.MOVE, 3, 1)
    assert outcome.accepted is True
    settle(board)
    assert board.channels[3].state == int(proto.MotionState.DONE)
    assert board.channels[3].fg_count == 0, "the output shaft never moved"
    assert board.channels[3].angle_deci_deg == 0


# --------------------------------------------------------------------------
# Sequence, dedupe and replay
# --------------------------------------------------------------------------


def test_rejected_command_still_consumes_a_sequence():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    first = send(session, proto.Opcode.MOVE, 0, proto.MAX_MOVE_DECI_DEG + 1)
    assert first.accepted is False
    assert board.last_seq == first.sequence, "a refusal is still a consumed request identity"
    assert board.last_result == int(proto.BusinessResult.VALUE)


def test_identical_replay_is_a_cached_no_op():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    first = send(session, proto.Opcode.STOP_ALL)
    executed = board.commands_executed
    replay = send(session, proto.Opcode.STOP_ALL, sequence=first.sequence)
    assert replay.accepted is True
    assert board.commands_executed == executed
    assert board.commands_replayed >= 1


def test_replayed_failure_returns_the_same_failure():
    """A cached rejection is replayed too, so a retry cannot turn it into success."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    first = send(session, proto.Opcode.MOVE, 0, proto.MAX_MOVE_DECI_DEG + 1)
    assert first.result == int(proto.BusinessResult.VALUE)
    replay = send(session, proto.Opcode.MOVE, 0, proto.MAX_MOVE_DECI_DEG + 1,
                  sequence=first.sequence)
    assert replay.result == int(proto.BusinessResult.VALUE)
    assert board.commands_executed == 0


def test_same_sequence_with_different_arguments_is_a_sequence_error():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    first = send(session, proto.Opcode.STOP, 0)
    conflict = send(session, proto.Opcode.STOP, 1, sequence=first.sequence)
    assert conflict.result == int(proto.BusinessResult.SEQUENCE)


def test_cache_holds_sixteen_entries_and_evicts_the_oldest():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    for index in range(proto.SEQUENCE_CACHE_ENTRIES + 1):
        send(session, proto.Opcode.STOP_ALL)
    assert len(board.cache) == proto.SEQUENCE_CACHE_ENTRIES
    assert board.cache_order[0] != 1, "the oldest entry must have been evicted"


def test_auto_allocated_sequence_is_reported_through_last_sequence():
    """Sequence 0 asks the board to choose; the identity is only readable afterwards.

    The driver itself refuses to send sequence 0 (proto.command_values rejects it),
    so this goes on the bus raw to prove the board-side behaviour the refusal is
    protecting against.
    """
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    raw_command(transport, 0, proto.Opcode.STOP_ALL)
    assert board.last_seq >= 1
    assert board.last_result == int(proto.BusinessResult.OK)
    with pytest.raises(ValueError):
        proto.command_values(0, int(proto.Opcode.STOP_ALL), 0, 0)


def test_sequence_outside_the_window_is_rejected():
    """A stale sequence is refused with no side effects: 0x0036 does not move, which
    is the discriminator the driver uses to tell this from an executed refusal."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.STOP_ALL, sequence=1000)
    assert board.last_seq == 1000
    executed = board.commands_executed
    stale = (1000 + 0x9000) & 0xFFFF
    assert not proto.sequence_acceptable(stale, 1000)
    code = exception_code(raw_command, transport, stale, proto.Opcode.STOP_ALL)
    assert code == int(proto.ModbusException.ILLEGAL_DATA_VALUE)
    assert board.last_seq == 1000, "a sequence rejection must not consume the identity"
    assert board.commands_executed == executed
    # The session's own guard refuses the same thing before it reaches the bus.
    with pytest.raises(proto.ProtocolError):
        session.send_command(proto.Opcode.STOP_ALL, 0, 0, sequence=stale)


# --------------------------------------------------------------------------
# Retired and illegal accesses
# --------------------------------------------------------------------------


def test_legacy_commit_register_is_a_tombstone():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    before = board.commands_received
    code = exception_code(transport.write_registers, proto.REG_COMMAND_TOMBSTONE,
                          [proto.LEGACY_COMMIT_MAGIC])
    assert code == int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert board.commands_received == before
    assert transport.read_registers(proto.REG_COMMAND_TOMBSTONE, 1) == [0]


def test_command_block_requires_fc16_address_30_count_5():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    before = board.commands_received
    assert exception_code(transport.write_registers, proto.REG_COMMAND, [1]) == \
        int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert exception_code(transport.write_registers, proto.REG_COMMAND + 1,
                          [1, 3, 0, 0, 100]) == int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert exception_code(transport.write_registers, proto.REG_COMMAND,
                          [1, 3, 0, 0]) == int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert board.commands_received == before


def test_unmapped_and_hole_crossing_reads_are_rejected():
    transport, board, stub, session, clock = quiet_stack()
    base = proto.channel_base(0)
    assert exception_code(transport.read_registers, 0x0011, 1) == \
        int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert exception_code(transport.read_registers, base + 5, 1) == \
        int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert exception_code(transport.read_registers, base + proto.OFF_STATUS, 14) == \
        int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)


def test_reading_the_command_snapshot_does_not_prove_execution():
    """0x0030..0x0034 is overwritten before validation, so it echoes even rejects."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.MOVE, 0, proto.MAX_MOVE_DECI_DEG + 1)   # rejected
    snapshot = transport.read_registers(proto.REG_COMMAND, 5)
    assert snapshot[1] == int(proto.Opcode.MOVE)
    assert board.commands_executed == 0


# --------------------------------------------------------------------------
# Persistence lifecycle
# --------------------------------------------------------------------------


def test_move_prewrites_a_dirty_record_and_saves_after_settling():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.MOVE, 0, 900)
    assert board.dirty is True, "accepting a MOVE pre-writes a dirty flash record"
    settle(board, 3.0, 6)
    assert board.channels[0].state == int(proto.MotionState.DONE)
    assert board.channels[0].angle_deci_deg == 900
    settle(board, 1.5, 3)
    assert board.dirty is False
    assert board.pending_save is False
    assert board.storage_error is False
    assert board.flash_writes >= 2, "one pre-write plus the settling save"


def test_reboot_with_a_dirty_image_costs_every_channel_its_zero():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.ZERO, 0)
    assert board.channels[0].valid is True
    send(session, proto.Opcode.MOVE, 0, 900)
    assert board.dirty is True
    board.reboot()
    assert board.channels[0].valid is False
    assert board.channels[0].state == int(proto.MotionState.UNREADY)
    assert board.cache == {}


def test_clean_reboot_keeps_the_zero_reference():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.ZERO, 0)
    clock.advance(1.0)
    board.advance()
    assert board.dirty is False
    board.reboot()
    assert board.channels[0].valid is True


def test_storage_failure_blocks_motion_until_a_save_succeeds():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    board.storage_failure = True
    assert send(session, proto.Opcode.SAVE).result == int(proto.BusinessResult.STORAGE)
    assert board.storage_error is True
    clock.advance(1.0)
    assert send(session, proto.Opcode.MOVE, 0, 900).result == int(proto.BusinessResult.STORAGE)
    assert send(session, proto.Opcode.SELFTEST, 0, 0).result == int(proto.BusinessResult.STORAGE)
    board.storage_failure = False
    clock.advance(1.0)
    assert send(session, proto.Opcode.SAVE).accepted is True
    assert board.storage_error is False
    assert send(session, proto.Opcode.MOVE, 0, 900).accepted is True


def test_clear_fault_leaves_valid_alone():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.ZERO, 0)
    assert board.channels[0].valid is True
    board.inject_fault(0, int(proto.FaultCode.POSITION))
    board.channels[0].valid = False       # a lost zero, as after an interrupted motion
    clock.advance(1.0)
    assert send(session, proto.Opcode.CLEAR_FAULT, 0).accepted is True
    assert board.channels[0].fault == 0
    assert board.channels[0].valid is False


def test_selftest_is_blocked_by_any_faulted_channel():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    board.inject_fault(2, int(proto.FaultCode.CAPTURE))
    clock.advance(1.0)
    assert send(session, proto.Opcode.SELFTEST, 0, 0).result == int(proto.BusinessResult.UNREADY)
    assert send(session, proto.Opcode.SELFTEST_ALL, 0, 0).result == int(proto.BusinessResult.UNREADY)


def test_selftest_pass_and_no_pulse_failure():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    assert send(session, proto.Opcode.SELFTEST, 0, 0).accepted is True
    settle(board, 4.0, 8)
    assert board.channels[0].probe_state == int(proto.SelftestState.PASSED)
    assert board.channels[0].probe_outbound >= proto.SELFTEST_TARGET_PULSES
    assert board.channels[0].probe_phase == int(proto.SelftestPhase.FINISHED)
    assert board.channels[0].probe_sequence >= 1

    board.selftest_pulses = 0
    settle(board, 1.0, 2)
    assert send(session, proto.Opcode.SELFTEST, 1, 0).accepted is True
    settle(board, 4.0, 8)
    assert board.channels[1].probe_state == int(proto.SelftestState.FAILED)
    assert board.channels[1].probe_reason == int(proto.SelftestReason.NO_PULSES)


def test_selftest_all_ignores_an_out_of_range_channel_field():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    outcome = send(session, proto.Opcode.SELFTEST_ALL, 999, 0)
    assert outcome.accepted is True
    # All four are queued, but only the first is energised: the firmware probes one
    # channel at a time and starts the next only after the current one settles
    # (BSP/robot.c:294-296).
    probing = [c.index for c in board.channels if c.state == int(proto.MotionState.SELFTEST)]
    assert probing == [0], probing
    assert all(c.probe_state in (int(proto.SelftestState.QUEUED),
                                 int(proto.SelftestState.RUNNING)) for c in board.channels)


def test_batch_probe_energises_one_channel_at_a_time_in_order():
    """The order and the exclusivity are what make a batch result mean anything: if two
    channels ran at once, "which channel produced these pulses" would be ambiguous."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    assert send(session, proto.Opcode.SELFTEST_ALL, 0, 0).accepted is True
    finished = []
    for _ in range(200):
        board.advance(0.05)
        probing = [c.index for c in board.channels
                   if c.state == int(proto.MotionState.SELFTEST)]
        assert len(probing) <= 1, "two channels energised at once: %s" % probing
        for channel in board.channels:
            if channel.probe_state in proto.SELFTEST_FINAL_STATES and channel.index not in finished:
                finished.append(channel.index)
        if not board.probe_active:
            break
    assert finished == [0, 1, 2, 3], finished
    assert all(c.probe_state == int(proto.SelftestState.PASSED) for c in board.channels)
    assert board.pending_save is True or board.dirty is False


def test_batch_probe_drops_the_remaining_queue_when_cancelled():
    """Channels the firmware never energised read UNTESTED, which is NOT a final state
    and must never be mistaken for a pass (BSP/robot.c:102-106)."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    sequence = send(session, proto.Opcode.SELFTEST_ALL, 0, 0).sequence
    settle(board, 0.6, 3)
    assert board.channels[0].state == int(proto.MotionState.SELFTEST)
    assert send(session, proto.Opcode.STOP_ALL, 0, 0).accepted is True

    assert board.channels[0].probe_state == int(proto.SelftestState.CANCELLED)
    for index in (1, 2, 3):
        record = board.channels[index]
        assert record.probe_state == int(proto.SelftestState.UNTESTED), index
        assert record.probe_reason == int(proto.SelftestReason.CANCELLED)
        # Still stamped with this batch, which is how the driver knows they were
        # selected and dropped rather than belonging to some earlier probe.
        assert record.probe_sequence == sequence
    assert board.probe_active is False


def test_scan_all_continues_after_a_no_pulses_discovery_failure():
    """A missing motor is an expected finding, not a safety abort: the firmware keeps
    probing the rest of the batch (BSP/robot.c:135-139)."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    board.selftest_pulses = 0
    assert send(session, proto.Opcode.SELFTEST_ALL, 0, 0).accepted is True
    for _ in range(200):
        board.advance(0.05)
        if not board.probe_active:
            break
    assert all(c.probe_state == int(proto.SelftestState.FAILED) for c in board.channels)
    assert all(c.probe_reason == int(proto.SelftestReason.NO_PULSES) for c in board.channels)
    assert board.probe_aborting is False
    # A non-passing probe costs the zero reference and sets the self-test fault.
    assert all(c.valid is False for c in board.channels)
    assert all(c.fault == int(proto.FaultCode.SELFTEST) for c in board.channels)


def test_single_probe_leaves_other_channels_records_alone():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    before = [(c.probe_state, c.probe_sequence) for c in board.channels]
    assert send(session, proto.Opcode.SELFTEST, 0, 0).accepted is True
    settle(board, 4.0, 8)
    assert board.channels[0].probe_state == int(proto.SelftestState.PASSED)
    for index in (1, 2, 3):
        assert (board.channels[index].probe_state,
                board.channels[index].probe_sequence) == before[index], index


def test_batch_probe_refused_on_storage_error_marks_every_selected_channel():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    board.storage_failure = True
    outcome = send(session, proto.Opcode.SELFTEST_ALL, 0, 0)
    assert outcome.accepted is False
    assert outcome.result == int(proto.BusinessResult.STORAGE)
    for channel in board.channels:
        assert channel.probe_state == int(proto.SelftestState.FAILED)
        assert channel.probe_reason == int(proto.SelftestReason.STORAGE_ERROR)
    assert board.probe_active is False


def test_selftest_records_are_ram_only():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.SELFTEST, 0, 0)
    settle(board, 4.0, 8)
    assert board.channels[0].probe_state == int(proto.SelftestState.PASSED)
    board.reboot()
    assert board.channels[0].probe_state == int(proto.SelftestState.UNTESTED)
    assert board.channels[0].probe_sequence == 0


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------


def test_config_rejects_an_illegal_gear_before_touching_flash():
    """Two layers: the session refuses to stage an illegal gear at all, and if the
    staged area somehow holds one anyway, the board rejects CONFIG without writing."""
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    with pytest.raises(ValueError):
        session.stage_channel_config(0, proto.ChannelConfigValues(True, 1, 100, 0, 8000))
    # Bypass the session and write the staged registers straight to the bus.
    transport.write_registers(proto.channel_base(0) + proto.OFF_CONFIG_STAGED,
                              [1, 1, 100, 0, 8000])
    writes = board.flash_writes
    assert send(session, proto.Opcode.CONFIG).result == int(proto.BusinessResult.VALUE)
    assert board.flash_writes == writes, "a rejected CONFIG must not write flash"
    assert board.channels[0].gear == 516, "the applied configuration must be untouched"


def test_config_rejects_duplicate_roles():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    session.stage_channel_config(0, proto.ChannelConfigValues(True, 1, 516, 0, 8000))
    session.stage_channel_config(1, proto.ChannelConfigValues(True, 1, 516, 0, 8000))
    assert send(session, proto.Opcode.CONFIG).result == int(proto.BusinessResult.VALUE)


def test_config_changes_gear_invalidate_the_zero_and_selftest():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.ZERO, 0)
    assert board.channels[0].valid is True
    session.stage_channel_config(0, proto.ChannelConfigValues(True, 2, 290, 0, 8000))
    assert send(session, proto.Opcode.CONFIG).accepted is True
    assert board.channels[0].gear == 290
    assert board.channels[0].valid is False


def test_config_applies_pwm_immediately_and_identity_after_reboot():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    session.stage_board_config(0, 9, 5, 21000)
    assert send(session, proto.Opcode.CONFIG).accepted is True
    assert board.pwm_hz == 21000
    assert board.address == 1 and board.node == 1
    board.reboot()
    assert board.address == 9 and board.node == 5


def test_config_requires_a_quiet_board():
    transport, board, stub, session, clock = quiet_stack(read_only=False)
    send(session, proto.Opcode.MOVE, 0, 900)
    assert send(session, proto.Opcode.CONFIG).result == int(proto.BusinessResult.BUSY)


# --------------------------------------------------------------------------
# Silence and framing at board level
# --------------------------------------------------------------------------


def test_bad_crc_and_wrong_address_are_ignored_silently():
    board = FakeBoard(clock=FakeClock(auto_advance=0))
    good = proto.build_adu(1, proto.read_pdu(proto.REG_UID, 6))
    assert board.handle_adu(good) is not None
    corrupted = bytearray(good)
    corrupted[-1] ^= 0xFF
    assert board.handle_adu(bytes(corrupted)) is None
    assert board.handle_adu(proto.build_adu(2, proto.read_pdu(proto.REG_UID, 6))) is None
    assert board.silent_drops == 2


def test_unknown_function_code_gets_exception_one():
    board = FakeBoard(clock=FakeClock(auto_advance=0))
    reply = board.handle_adu(proto.build_adu(1, bytes(bytearray([0x07, 0, 0, 0, 1]))))
    assert reply is not None
    assert reply[1] == 0x87
    assert reply[2] == int(proto.ModbusException.ILLEGAL_FUNCTION)


def test_serial_stub_delivers_a_fragmented_reply():
    board = FakeBoard(clock=FakeClock(auto_advance=0))
    stub = SerialStub(board)
    stub.fragment_size = 1
    stub.write(proto.build_adu(1, proto.read_pdu(proto.REG_UID, 6)))
    collected = b""
    for _ in range(64):
        chunk = stub.read(8)
        if not chunk:
            break
        collected += chunk
    assert proto.frame_crc_ok(collected)


def test_serial_stub_assembles_a_split_request():
    board = FakeBoard(clock=FakeClock(auto_advance=0))
    stub = SerialStub(board)
    frame = proto.build_adu(1, proto.read_pdu(proto.REG_UID, 6))
    stub.write(frame[:3])
    assert stub.read(64) == b"", "no reply until the request is complete"
    stub.write(frame[3:])
    assert proto.frame_crc_ok(stub.read(64))


@pytest.mark.parametrize("gear,budget", [(516,86),(290,48),(82,13)])
def test_fg_ten_degree_boundary(gear,budget):
    board = FakeBoard()
    board.channels[0].gear=gear
    for _ in range(budget):
        board.inject_unexpected_pulses(0,1)
    assert board.channels[0].fault == 0
    board.inject_unexpected_pulses(0,1)
    assert board.channels[0].fault == 4


def test_fg_unconfigured_and_staged_gear_do_not_enable_fault():
    board=FakeBoard()
    c=board.channels[0]
    c.gear=0
    c.staged=(1,1,516,0,8000)
    board.inject_unexpected_pulses(0,100)
    assert c.fault == 0
    c.gear=516
    board.inject_unexpected_pulses(0,1)
    assert c.fault == 4
    c.fault=0;c.gear=0
    board.inject_unexpected_pulses(0,0xFFFFFFFF)
    assert c.unexpected_fg == 0xFFFFFFFF and c.fault == 0
