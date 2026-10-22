"""Transport tests: framing, resynchronisation, retries, ownership, bounded waits.

These run against the REAL ModbusRtuTransport with a pyserial-shaped stub, so the
production framing code is what gets exercised.
"""

import threading
import time

import pytest

from robot_motor_driver import protocol as proto
from robot_motor_driver.fake_board import FakeClock, make_fake_stack
from robot_motor_driver.transport_modbus import (ModbusRtuTransport, TransportClosed,
                                                 TransportOwnershipError)


def stack(**kwargs):
    return make_fake_stack(**kwargs)


# --------------------------------------------------------------------------
# Basic reads and writes
# --------------------------------------------------------------------------


def test_read_registers_returns_uid():
    transport, board, stub, session, clock = stack()
    values = transport.read_registers(proto.REG_UID, 6)
    assert proto.uid_from_registers(values) == board.uid


def test_read_unmapped_register_raises_exception_02():
    """0x0011..0x001F is unmapped and must come back as an illegal data address."""
    transport, board, stub, session, clock = stack()
    with pytest.raises(proto.ModbusExceptionResponse) as info:
        transport.read_registers(0x0011, 1)
    assert info.value.code == int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)


def test_read_crossing_the_channel_hole_fails_as_a_whole():
    """Offsets 0x17 between status and position is unmapped, so one 14 register
    read spanning it is rejected instead of returning partial data."""
    transport, board, stub, session, clock = stack()
    with pytest.raises(proto.ModbusExceptionResponse):
        transport.read_registers(proto.channel_base(0) + proto.OFF_STATUS, 14)
    # The two halves individually are fine.
    assert len(transport.read_registers(proto.channel_base(0) + proto.OFF_STATUS, 7)) == 7
    assert len(transport.read_registers(proto.channel_base(0) + proto.OFF_POSITION, 6)) == 6


def test_write_staged_board_configuration_is_accepted():
    transport, board, stub, session, clock = stack()
    session.read_only = False
    transport.write_registers(proto.REG_STAGED_BOARD, [0, 1, 1, 25000])
    assert board.staged_board == [0, 1, 1, 25000]


def test_legacy_a55a_commit_register_is_refused():
    """The retired commit register 0x0035 must reject writes with exception 02."""
    transport, board, stub, session, clock = stack()
    before = board.commands_received
    with pytest.raises(proto.ModbusExceptionResponse) as info:
        transport.write_registers(proto.REG_COMMAND_TOMBSTONE, [proto.LEGACY_COMMIT_MAGIC])
    assert info.value.code == int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert board.commands_received == before, "the retired commit path must not execute anything"
    assert transport.read_registers(proto.REG_COMMAND_TOMBSTONE, 1) == [0]


def test_single_register_write_to_command_block_is_refused():
    """FC6 to 0x0030 is not a command; only the atomic FC16 block is."""
    transport, board, stub, session, clock = stack()
    before = board.commands_received
    with pytest.raises(proto.ModbusExceptionResponse) as info:
        transport.write_registers(proto.REG_COMMAND, [1])
    assert info.value.code == int(proto.ModbusException.ILLEGAL_DATA_ADDRESS)
    assert board.commands_received == before


def test_partial_command_block_is_refused_without_side_effects():
    """FC16 starting at 0x0031 or with count != 5 must be rejected atomically."""
    transport, board, stub, session, clock = stack()
    clock.advance(1.0)
    before = board.commands_received
    with pytest.raises(proto.ModbusExceptionResponse):
        transport.write_registers(proto.REG_COMMAND + 1, [1, 3, 0, 0, 100])
    with pytest.raises(proto.ModbusExceptionResponse):
        transport.write_registers(proto.REG_COMMAND, [1, 3, 0, 0])
    assert board.commands_received == before
    assert board.channels[0].state == int(proto.MotionState.UNREADY)


# --------------------------------------------------------------------------
# Framing robustness
# --------------------------------------------------------------------------


def test_reply_arriving_one_byte_at_a_time_is_reassembled():
    transport, board, stub, session, clock = stack()
    stub.fragment_size = 1
    values = transport.read_registers(proto.REG_SCHEMA, 16)
    assert len(values) == 16
    assert values[proto.REG_COMMAND_PROTOCOL] == proto.COMMAND_PROTOCOL_REVISION


def test_leading_garbage_is_skipped():
    """A shared RS-485 bus or a previous timeout can leave junk in the stream."""
    transport, board, stub, session, clock = stack()
    stub.preload_garbage = bytes(bytearray([0x55, 0x66, 0x77]))
    assert transport.read_registers(proto.REG_UID, 6)[0] == 0xC23B


def test_stale_half_frame_from_a_previous_timeout_is_resynchronised():
    transport, board, stub, session, clock = stack()
    stub.leave_stale_half_frame = True
    values = transport.read_registers(proto.REG_LAST_SEQUENCE, 2)
    assert len(values) == 2


def test_corrupted_crc_is_detected_and_the_identical_frame_retried():
    """A bad CRC makes the receiver drop bytes and resynchronise, then the identical
    frame is retried; the read still comes back correct."""
    transport, board, stub, session, clock = stack()
    stub.corrupt_crc = 1
    values = transport.read_registers(proto.REG_UID, 6)
    assert proto.uid_from_registers(values) == board.uid
    assert transport.resyncs >= 1, "the corrupted frame must have forced a resynchronisation"


def test_a_stream_of_pure_noise_ends_in_a_bounded_timeout():
    """Garbage with no valid frame must not spin forever: the deadline ends it."""
    transport, board, stub, session, clock = stack()
    stub.drop_responses = 1
    stub.preload_garbage = bytes(bytearray(range(0x10))) * 4
    transport.read_retries = 0
    started = time.monotonic()
    with pytest.raises((proto.ModbusTimeout, proto.FrameError)):
        transport.read_registers(proto.REG_UID, 6)
    assert time.monotonic() - started < 5.0


def test_lost_reply_raises_timeout_without_retrying_by_default_for_writes():
    transport, board, stub, session, clock = stack()
    clock.advance(1.0)
    stub.drop_responses = 1
    started = time.monotonic()
    with pytest.raises(proto.ModbusTimeout):
        transport.write_command(7, int(proto.Opcode.MOVE), 0, 100, retries=0)
    assert time.monotonic() - started < 5.0, "the transaction window must stay bounded"
    # The board did execute it; that is exactly why a new sequence must not be used.
    assert board.last_seq == 7


def test_read_retries_recover_from_a_single_lost_reply():
    transport, board, stub, session, clock = stack()
    stub.drop_responses = 1
    values = transport.read_registers(proto.REG_UID, 6)
    assert proto.uid_from_registers(values) == board.uid


def test_closed_transport_refuses_transactions():
    transport, board, stub, session, clock = stack()
    transport.close()
    with pytest.raises(TransportClosed):
        transport.read_registers(0, 1)


# --------------------------------------------------------------------------
# Ownership
# --------------------------------------------------------------------------


def test_strict_ownership_rejects_a_second_thread():
    """One port, one owner: a second thread must be refused, not interleaved."""
    transport, board, stub, session, clock = stack()
    transport.strict_ownership = True
    # The stack was identified on this thread; hand the port over so the holder
    # thread below can legitimately become the owner.
    transport.release_ownership()
    original_read = stub.read
    held = threading.Event()

    def slow_read(count=1):
        data = original_read(count)
        if data and not held.is_set():
            held.set()
            time.sleep(0.3)      # keep the port inside a real transaction
        return data

    stub.read = slow_read
    errors = []

    def hold_the_port():
        try:
            transport.read_registers(proto.REG_UID, 6)
        except Exception as error:      # pragma: no cover - defensive
            errors.append(error)

    holder = threading.Thread(target=hold_the_port)
    holder.start()
    assert held.wait(5.0), "the first transaction never started: %s" % errors
    with pytest.raises(TransportOwnershipError):
        transport.read_registers(proto.REG_UID, 6)
    holder.join(5.0)
    assert not errors, "the owning thread must complete normally: %s" % errors


def test_release_ownership_lets_the_worker_thread_take_over():
    """Regression test for a real wiring bug: a node identifies the board on the main
    thread, then the worker thread must be allowed to own the port afterwards."""
    transport, board, stub, session, clock = stack()
    transport.strict_ownership = True
    transport.release_ownership()
    assert transport.owner is None
    results = []

    def worker():
        results.append(transport.read_registers(proto.REG_UID, 6))

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join(5.0)
    assert results and len(results[0]) == 6
    assert transport.owner == thread.ident
    # And the original thread is now the outsider.
    with pytest.raises(TransportOwnershipError):
        transport.read_registers(proto.REG_UID, 6)


def test_ownership_is_recorded_after_first_use():
    transport, board, stub, session, clock = stack()
    transport.read_registers(proto.REG_UID, 6)
    assert transport.owner == threading.current_thread().ident


# --------------------------------------------------------------------------
# Statistics and lifecycle
# --------------------------------------------------------------------------


def test_statistics_count_transactions_and_resynchronisations():
    transport, board, stub, session, clock = stack()
    transport.read_registers(proto.REG_UID, 6)
    stub.corrupt_crc = 1
    transport.read_registers(proto.REG_UID, 6)
    stats = transport.statistics()
    assert stats["transactions"] >= 2
    assert stats["resyncs"] >= 1
    assert stats["open"] is True
    assert stats["owner_thread"] == threading.current_thread().ident


def test_context_manager_closes_the_port():
    transport, board, stub, session, clock = stack()
    with transport as open_transport:
        open_transport.read_registers(proto.REG_UID, 6)
    assert transport.is_open is False
