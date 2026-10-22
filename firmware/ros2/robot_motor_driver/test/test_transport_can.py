"""CAN codec tests, plus the refusal path that keeps an unproven transport honest.

CAN has NOT been exercised on hardware in this round and python-can is not
installed on the target host, so `CanTransport.open()` must refuse by default.
What is tested here is the frame codec, which is pure and verifiable offline, and
the refusal itself.
"""

import pytest

from robot_motor_driver import protocol as proto
from robot_motor_driver.transport_can import (CAN_DLC, CanTransport, decode_read_registers,
                                              decode_response, encode_command, encode_read,
                                              encode_write, request_id, response_id)
from robot_motor_driver.transport_modbus import TransportNotAvailable


class FakeMessage(object):
    def __init__(self, arbitration_id=None, is_extended_id=False, data=b"",
                 is_remote_frame=False, is_error_frame=False):
        self.arbitration_id = arbitration_id
        self.is_extended_id = is_extended_id
        self.data = bytes(data)
        self.is_remote_frame = is_remote_frame
        self.is_error_frame = is_error_frame


class FakeBus(object):
    """Answers CAN requests the way the firmware does, byte for byte."""

    def __init__(self, interface=None, channel=None, bitrate=None):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.sent = []
        self._replies = []
        self.shutdown_called = False
        #: registers returned by reads, keyed by address
        self.registers = {0x000F: 2, 0x0008: 0xC23B, 0x0009: 0x3781}
        self.result_to_report = 0
        self.echo_wrong_seq_low = False

    def send(self, message, timeout=None):
        self.sent.append(message)
        payload = bytes(message.data)
        kind = payload[0]
        result = self.result_to_report
        if kind == 1:                       # read
            address = (payload[2] << 8) | payload[3]
            count = payload[4]
            values = [self.registers.get(address + index, 0) for index in range(2)]
            # byte 3 echoes the count, bytes 4..7 carry two registers big endian
            reply = bytes(bytearray([0x81, payload[1], result, count,
                                     values[0] >> 8, values[0] & 0xFF,
                                     values[1] >> 8, values[1] & 0xFF]))
        elif kind == 2:                     # write
            reply = bytes(bytearray([0x82, payload[1], result, 0, 0, 0, 0, 0]))
        elif kind == 3:                     # command
            seq_low = payload[2]
            if self.echo_wrong_seq_low:
                seq_low = (seq_low + 1) & 0xFF
            reply = bytes(bytearray([0x83, payload[1], result, seq_low, 0, 0, 0, 0]))
        else:
            reply = bytes(bytearray([kind | 0x80, 0, 1, 0, 0, 0, 0, 0]))
        self._replies.append(FakeMessage(arbitration_id=0x580 + 1, data=reply))
        return True

    def recv(self, timeout=None):
        if not self._replies:
            return None
        return self._replies.pop(0)

    def shutdown(self):
        self.shutdown_called = True


class FakeCanModule(object):
    Message = FakeMessage
    Bus = FakeBus


def make_transport(**kwargs):
    kwargs.setdefault("interface", "socketcan")
    kwargs.setdefault("channel", "can0")
    kwargs.setdefault("require_hardware_evidence", False)
    kwargs.setdefault("can_module", FakeCanModule)
    transport = CanTransport(**kwargs)
    transport.open()
    return transport


# --------------------------------------------------------------------------
# Refusal paths
# --------------------------------------------------------------------------


def test_open_refuses_without_hardware_evidence():
    """The default must be a loud refusal, not a silent fake success."""
    transport = CanTransport(interface="socketcan", channel="can0")
    with pytest.raises(TransportNotAvailable) as info:
        transport.open()
    assert "not been validated on hardware" in str(info.value)
    assert transport.is_open is False


def test_open_refuses_when_python_can_is_missing():
    class NoCan(object):
        def __getattr__(self, name):
            raise ImportError("No module named 'can'")

    transport = CanTransport(interface="socketcan", channel="can0",
                             require_hardware_evidence=False, can_module=None)
    import sys
    saved = sys.modules.get("can")
    sys.modules["can"] = None       # force the import to fail
    try:
        with pytest.raises((TransportNotAvailable, ImportError)):
            transport.open()
    finally:
        if saved is not None:
            sys.modules["can"] = saved
        else:
            sys.modules.pop("can", None)


# --------------------------------------------------------------------------
# Codec
# --------------------------------------------------------------------------


def test_identifiers():
    assert request_id(1) == 0x601
    assert response_id(1) == 0x581
    with pytest.raises(ValueError):
        request_id(128)


def test_read_frame_layout():
    frame = encode_read(0x2A, 0x000F, 1)
    assert len(frame) == CAN_DLC
    assert frame[0] == 1 and frame[1] == 0x2A
    assert (frame[2] << 8 | frame[3]) == 0x000F
    assert frame[4] == 1
    assert frame[5:] == b"\x00\x00\x00", "reserved bytes must be zero"


def test_read_rejects_more_than_two_registers():
    with pytest.raises(ValueError):
        encode_read(1, 0x0000, 3)


def test_write_frame_layout():
    frame = encode_write(0x07, 0x0020, 25000)
    assert len(frame) == CAN_DLC
    assert frame[0] == 2 and frame[1] == 0x07
    assert (frame[2] << 8 | frame[3]) == 0x0020
    assert (frame[5] << 8 | frame[6]) == 25000


def test_command_frame_layout_and_int16_argument():
    frame = encode_command(0x1234, int(proto.Opcode.MOVE), 2, -900)
    assert len(frame) == CAN_DLC
    assert frame[0] == 3
    assert (frame[1] << 8 | frame[2]) == 0x1234
    assert frame[3] == int(proto.Opcode.MOVE)
    assert frame[4] == 2
    assert int.from_bytes(frame[5:7], "big", signed=True) == -900
    assert frame[7] == 0


def test_command_argument_must_fit_int16():
    """Modbus carries an int32 argument; CAN only has room for an int16."""
    with pytest.raises(ValueError):
        encode_command(1, int(proto.Opcode.MOVE), 0, 40000)
    with pytest.raises(ValueError):
        encode_command(1, int(proto.Opcode.MOVE), 0, -40000)
    # The legal MOVE range still fits.
    encode_command(1, int(proto.Opcode.MOVE), 0, proto.MAX_MOVE_DECI_DEG)
    encode_command(1, int(proto.Opcode.MOVE), 0, -proto.MAX_MOVE_DECI_DEG)


def test_command_requires_nonzero_sequence():
    with pytest.raises(ValueError):
        encode_command(0, int(proto.Opcode.MOVE), 0, 100)


def test_decode_response_checks_type_tag_and_result_range():
    request = encode_read(0x11, 0x000F, 1)
    result, payload = decode_response(bytes(bytearray([0x81, 0x11, 0, 1, 0, 2, 0, 0])), request, 0x11)
    assert result == 0 and payload[5] == 2
    with pytest.raises(proto.FrameError):
        decode_response(bytes(bytearray([0x82, 0x11, 0, 1, 0, 2, 0, 0])), request, 0x11)
    with pytest.raises(proto.FrameError):
        decode_response(bytes(bytearray([0x81, 0x12, 0, 1, 0, 2, 0, 0])), request, 0x11)
    with pytest.raises(proto.FrameError):
        decode_response(bytes(bytearray([0x81, 0x11, 9, 1, 0, 2, 0, 0])), request, 0x11)
    with pytest.raises(proto.FrameError):
        decode_response(bytes(bytearray([0x81, 0x11, 0, 1, 0, 2, 0])), request, 0x11)


def test_decode_response_checks_the_command_sequence_echo():
    request = encode_command(0x1234, int(proto.Opcode.MOVE), 0, 100)
    good = bytes(bytearray([0x83, 0x12, 0, 0x34, 0, 0, 0, 0]))
    assert decode_response(good, request)[0] == 0
    bad = bytes(bytearray([0x83, 0x12, 0, 0x35, 0, 0, 0, 0]))
    with pytest.raises(proto.FrameError):
        decode_response(bad, request)


def test_decode_read_registers_validates_count():
    payload = bytes(bytearray([0x81, 1, 0, 2, 0x00, 0x0F, 0x00, 0x02]))
    assert decode_read_registers(payload, 2) == [0x000F, 0x0002]
    with pytest.raises(proto.FrameError):
        decode_read_registers(payload, 1)


# --------------------------------------------------------------------------
# Transport level, against the fake bus
# --------------------------------------------------------------------------


def test_read_chunks_into_two_register_frames():
    transport = make_transport()
    values = transport.read_registers(0x0008, 5)
    assert len(values) == 5
    assert len(transport._bus.sent) == 3, "5 registers need three CAN frames"
    assert values[0] == 0xC23B


def test_write_sends_one_frame_per_register():
    transport = make_transport()
    transport.write_registers(0x0020, [0, 1, 1, 25000])
    assert len(transport._bus.sent) == 4


def test_command_success_and_business_result():
    transport = make_transport()
    transport.write_command(7, int(proto.Opcode.STOP_ALL), 0, 0)
    assert len(transport._bus.sent) == 1
    transport._bus.result_to_report = int(proto.BusinessResult.BUSY)
    with pytest.raises(proto.ModbusExceptionResponse) as info:
        transport.write_command(8, int(proto.Opcode.MOVE), 0, 100)
    # CAN reports the raw business result, so the driver can name it exactly.
    assert "CAN business result 2" in str(info.value)


def test_command_response_with_wrong_sequence_echo_is_rejected():
    transport = make_transport()
    transport._bus.echo_wrong_seq_low = True
    with pytest.raises(proto.FrameError):
        transport.write_command(9, int(proto.Opcode.STOP_ALL), 0, 0)


def test_command_frame_matches_reference_implementation():
    """tests/test_robot_host.py:62 pins the CAN command bytes for seq=123 op=3 ch=2
    arg=-900, which also proves the argument travels as int16 here."""
    assert encode_command(123, 3, 2, -900).hex() == "03007b0302fc7c00"


def test_close_shuts_the_bus_down():
    transport = make_transport()
    bus = transport._bus
    assert transport.is_open is True
    transport.close()
    assert transport.is_open is False
    assert bus.shutdown_called is True


def test_statistics_report_the_missing_hardware_evidence():
    transport = CanTransport(interface="socketcan", channel="can0")
    stats = transport.statistics()
    assert stats["hardware_evidence"] is False
    assert stats["open"] is False
