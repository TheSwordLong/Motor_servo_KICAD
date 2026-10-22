"""Protocol layer tests: framing, register math, unit conversion, sequence rules.

Several vectors are REAL frames captured from the audited board
(UID C23B378115B092083830364A) over /dev/ttyUSB0 on 2026-09-29, so the codec is
pinned to observed hardware behaviour and not only to the simulator.
"""

import math
import struct

import pytest

from robot_motor_driver import protocol as proto


def hexbytes(text):
    return bytes(bytearray.fromhex(text.replace(" ", "")))


# Frames read from the live board during the read-only probe (all CRC verified).
REAL_FRAMES = [
    "01 03 00 01 00 04 15 C9",   # board config request
    "01 03 00 05 00 03 15 CA",   # dirty / storage_error / pending_save
    "01 03 00 08 00 06 44 0A",   # UID
    "01 03 00 0F 00 01 B4 09",   # command protocol revision
    "01 03 00 36 00 02 24 05",   # last sequence / result
    "01 03 01 00 00 05 84 35",   # M1 applied config
    "01 03 01 10 00 07 04 31",   # M1 status
    "01 03 01 18 00 06 44 33",   # M1 angle block
    "01 03 01 50 00 07 05 E5",   # M2 status
    "01 03 01 90 00 07 05 D9",   # M3 status
    "01 03 01 D0 00 07 04 0D",   # M4 status
    "01 03 01 E0 00 08 44 06",   # M4 self test summary
]

REAL_REPLIES = [
    "01 03 0C C2 3B 37 81 15 B0 92 08 38 30 36 4A 78 12",  # UID
    "01 03 02 00 02 39 85",                                 # protocol revision 2
    "01 03 04 00 90 00 00 FA 1E",                           # last seq 144, result 0
    "01 03 06 00 00 00 00 00 00 21 75",                     # save flags all zero
    "01 03 0A 00 01 00 02 02 04 00 00 1F 40 F3 04",         # M1 config
    "01 03 0E 00 01 00 00 00 01 00 01 00 00 00 00 00 00 F0 C4",  # M3 status
    "01 03 0C FF FF FB F1 FF FF FF 72 FF FF FB F1 64 5F",  # M4 angle block
    "01 03 08 00 00 00 01 00 01 61 A8 D1 F9",              # board config
]


# --------------------------------------------------------------------------
# CRC and framing
# --------------------------------------------------------------------------


@pytest.mark.parametrize("frame", REAL_FRAMES + REAL_REPLIES)
def test_crc_of_real_captured_frames(frame):
    """Every frame seen on the wire must validate, which pins the CRC routine."""
    assert proto.frame_crc_ok(hexbytes(frame)), frame


def test_crc16_known_vector():
    # tools/robot_host.py and BSP/robot_protocol.c agree on this value.
    assert proto.crc16(hexbytes("01 03 00 08 00 06")) == 0x0A44


def test_append_crc_is_little_endian():
    body = hexbytes("01 03 00 08 00 06")
    assert proto.append_crc(body) == body + b"\x44\x0A"


def test_build_adu_matches_captured_request():
    pdu = proto.read_pdu(proto.REG_UID, 6)
    assert proto.build_adu(1, pdu) == hexbytes("01 03 00 08 00 06 44 0A")


def test_command_frame_matches_reference_implementation():
    """tests/test_robot_host.py:50 pins seq=123 op=3 ch=2 arg=-900 to this exact PDU.

    The reference vector is the PDU that `Modbus.exchange` receives, i.e. without
    the slave address and without the CRC; the trailing fc7c is the low word of the
    negative argument, not a checksum.
    """
    pdu = proto.command_pdu(123, 3, 2, -900)
    assert pdu.hex() == "10003000050a007b00030002fffffc7c"
    adu = proto.build_adu(1, pdu)
    assert adu[:1] == b"\x01" and adu[1:-2] == pdu
    assert proto.frame_crc_ok(adu)
    assert len(adu) == 19


def test_command_frame_is_always_fc16_at_0x0030_with_five_registers():
    pdu = proto.command_pdu(1, int(proto.Opcode.MOVE), 3, -3600)
    assert pdu[0] == 16
    assert (pdu[1] << 8 | pdu[2]) == proto.REG_COMMAND
    assert (pdu[3] << 8 | pdu[4]) == proto.REG_COMMAND_COUNT
    assert pdu[5] == 10


def test_expected_response_length():
    assert proto.expected_response_length(hexbytes("01 83 02")) == 5
    assert proto.expected_response_length(hexbytes("01 03 0E")) == 5 + 14
    assert proto.expected_response_length(hexbytes("01 10 00 30")) == 8
    assert proto.expected_response_length(hexbytes("01 06 00 20")) == 8
    assert proto.expected_response_length(b"\x01") is None


def test_read_pdu_rejects_out_of_range_count():
    with pytest.raises(ValueError):
        proto.read_pdu(0, 0)
    with pytest.raises(ValueError):
        proto.read_pdu(0, 126)


def test_write_multi_pdu_rejects_out_of_range_count():
    with pytest.raises(ValueError):
        proto.write_multi_pdu(0x20, [])
    with pytest.raises(ValueError):
        proto.write_multi_pdu(0x20, [0] * 124)


def test_command_requires_nonzero_sequence():
    """Sequence 0 asks the board to allocate one, which is not replay safe."""
    with pytest.raises(ValueError):
        proto.command_values(0, int(proto.Opcode.MOVE), 0, 100)
    with pytest.raises(ValueError):
        proto.command_values(65536, int(proto.Opcode.MOVE), 0, 100)


def test_command_rejects_unknown_opcode():
    with pytest.raises(ValueError):
        proto.command_values(1, 13, 0, 0)
    with pytest.raises(ValueError):
        proto.command_values(1, 0, 0, 0)


# --------------------------------------------------------------------------
# Register map
# --------------------------------------------------------------------------


def test_channel_base_and_yaml_mapping():
    assert proto.channel_base(0) == 0x0100
    assert proto.channel_base(3) == 0x01C0
    assert proto.yaml_channel_to_protocol(1) == 0
    assert proto.yaml_channel_to_protocol(4) == 3
    with pytest.raises(ValueError):
        proto.yaml_channel_to_protocol(0)
    with pytest.raises(ValueError):
        proto.channel_base(4)


def test_staged_config_address_matches_reference_client():
    """tools/robot_host.py:217 stages channel config at 0x108 + i*0x40."""
    for index in range(4):
        assert proto.channel_base(index) + proto.OFF_CONFIG_STAGED == 0x108 + index * 0x40


def test_status_and_position_are_not_contiguous():
    """Offset 0x17 is unmapped, so status and position need two separate reads."""
    assert proto.OFF_STATUS + 7 == 0x17
    assert proto.OFF_POSITION == 0x18


# --------------------------------------------------------------------------
# Integer and unit conversion
# --------------------------------------------------------------------------


def test_i32_round_trip_including_negatives():
    for value in (0, 1, -1, 3600, -3600, 2 ** 31 - 1, -(2 ** 31)):
        high, low = proto.pack_i32(value)
        assert proto.unpack_i32(high, low) == value


def test_trunc_div_matches_c_semantics():
    assert proto.trunc_div(7, 2) == 3
    assert proto.trunc_div(-7, 2) == -3      # C truncates toward zero, Python floors
    assert proto.trunc_div(7, -2) == -3
    assert proto.trunc_div(-1800, 3600) == 0


def test_round_half_away_from_zero_is_symmetric():
    """Python's round() is banker's rounding; the driver must not use it here."""
    assert proto.round_half_away_from_zero(0.5) == 1
    assert proto.round_half_away_from_zero(-0.5) == -1
    assert proto.round_half_away_from_zero(2.5) == 3
    assert proto.round_half_away_from_zero(-2.5) == -3
    assert round(0.5) == 0 and round(-0.5) == 0, "builtin round is banker's rounding"
    with pytest.raises(ValueError):
        proto.round_half_away_from_zero(float("nan"))
    with pytest.raises(ValueError):
        proto.round_half_away_from_zero(float("inf"))


def test_rad_to_deci_deg_never_multiplies_by_gear():
    ten_deg = 10.0 * math.pi / 180.0
    assert proto.rad_to_deci_deg(ten_deg) == 100
    assert proto.rad_to_deci_deg(-ten_deg) == -100
    assert proto.deci_deg_to_rad(100) == pytest.approx(ten_deg)


def test_rad_conversion_rejects_non_finite():
    for bad in (float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ValueError):
            proto.rad_to_deci_deg(bad)


@pytest.mark.parametrize("deci,gear,steps", [
    (3600, 516, 3096),      # one full output revolution at gear 516
    (-3600, 516, -3096),
    (900, 516, 774),
    (-900, 516, -774),
    (0, 516, 0),            # the -1800 bias still truncates to zero
    (3600, 290, 1740),
    (3600, 82, 492),
])
def test_deci_deg_to_steps_matches_firmware_formula(deci, gear, steps):
    """BSP/robot.c:450 -> (arg*gear*6 +/- 1800)/3600 with C truncation."""
    assert proto.deci_deg_to_steps(deci, gear) == steps
    reference = (deci * gear * 6 + (1800 if deci > 0 else -1800))
    expected = int(reference / 3600.0) if reference >= 0 else -int(abs(reference) / 3600.0)
    assert proto.deci_deg_to_steps(deci, gear) == expected


def test_steps_to_deci_deg_truncates_toward_zero():
    assert proto.steps_to_deci_deg(3096, 516) == 3600.0
    assert proto.steps_to_deci_deg(774, 516) == 900.0
    assert proto.steps_to_deci_deg(-774, 516) == -900.0
    # One step at gear 516 is 3600/3096 = 1.16 deci-deg, truncated to 1, not 0.
    assert proto.steps_to_deci_deg(1, 516) == 1.0
    assert proto.steps_to_deci_deg(2, 516) == 2.0        # 2.32 -> 2
    assert proto.steps_to_deci_deg(-1, 516) == -1.0      # truncation is toward zero
    assert proto.steps_to_deci_deg(0, 516) == 0.0
    assert proto.steps_to_deci_deg(1234, 0) == 0.0       # gear 0 has no scale


def test_steps_and_angle_are_inverse_on_exact_values():
    for gear in (82, 290, 516):
        for deci in (-3600, -900, 0, 900, 3600):
            steps = proto.deci_deg_to_steps(deci, gear)
            assert proto.steps_to_deci_deg(steps, gear) == float(deci)


def test_pulses_per_output_rev():
    assert proto.pulses_per_output_rev(516) == 3096
    assert proto.pulses_per_output_rev(0) == 0


def test_motion_timeout_formula():
    """BSP/robot.c:462 -> 10000 + 4 * (distance*gear*60000/(3600*rpm))."""
    assert proto.motion_timeout_ms(3600, 516, 8000) == 25480
    assert proto.motion_timeout_ms(0, 516, 8000) == 10000
    assert proto.motion_timeout_ms(-900, 516, 400) == 10000 + 4 * (900 * 516 * 60000 // (3600 * 400))
    assert proto.motion_timeout_ms(900, 0, 8000) == 10000


def test_move_argument_limits():
    assert proto.validate_move_argument(3600) == 3600
    assert proto.validate_move_argument(-3600) == -3600
    with pytest.raises(ValueError):
        proto.validate_move_argument(3601)
    with pytest.raises(ValueError):
        proto.validate_move_argument(-3601)


# --------------------------------------------------------------------------
# Validation domains
# --------------------------------------------------------------------------


def test_configuration_domains():
    for gear in (0, 82, 290, 516):
        assert proto.validate_gear(gear) == gear
    with pytest.raises(ValueError):
        proto.validate_gear(100)
    assert proto.validate_rpm(400) == 400
    assert proto.validate_rpm(8000) == 8000
    for bad in (399, 8001, 0):
        with pytest.raises(ValueError):
            proto.validate_rpm(bad)
    assert proto.validate_pwm_hz(25000) == 25000
    with pytest.raises(ValueError):
        proto.validate_pwm_hz(19999)
    with pytest.raises(ValueError):
        proto.validate_address(248)
    with pytest.raises(ValueError):
        proto.validate_node(128)


def test_uid_validation():
    assert proto.validate_uid("c23b378115b092083830364a") == "C23B378115B092083830364A"
    for bad in ("", "C23B37", "ZZ3B378115B092083830364A", None, 123):
        with pytest.raises(ValueError):
            proto.validate_uid(bad)


def test_uid_from_registers_uses_high_word_first():
    registers = [0xC23B, 0x3781, 0x15B0, 0x9208, 0x3830, 0x364A]
    assert proto.uid_from_registers(registers) == "C23B378115B092083830364A"


def test_roles_must_be_unique():
    proto.validate_roles_unique([0, 1, 2, 4])       # unassigned may repeat
    proto.validate_roles_unique([0, 0, 0, 0])
    with pytest.raises(ValueError):
        proto.validate_roles_unique([1, 1, 0, 0])
    with pytest.raises(ValueError):
        proto.validate_roles_unique([0, 8, 0, 0])


# --------------------------------------------------------------------------
# Sequence rules
# --------------------------------------------------------------------------


def test_next_sequence_wraps_past_zero():
    assert proto.next_sequence(0) == 1
    assert proto.next_sequence(1) == 2
    assert proto.next_sequence(0xFFFF) == 1     # zero is skipped, never reused
    assert proto.next_sequence(0xFFFE) == 0xFFFF


def test_sequence_window_rejects_backward_jumps():
    assert proto.sequence_acceptable(2, 1)
    assert proto.sequence_acceptable(1, 0xFFFF)          # wrap is fine
    assert not proto.sequence_acceptable(0x9000, 1)      # distance >= 0x8000
    assert proto.sequence_acceptable(0x8000, 1)          # distance 0x7FFF is the last legal one


def test_equal_sequence_passes_the_window_and_is_handled_by_dedupe():
    """Replaying the last sequence is NOT a window violation (distance 0).

    The board answers it from its dedupe cache: identical parameters return the
    cached result with zero side effects, different parameters return R_SEQUENCE.
    """
    assert proto.sequence_distance(1, 1) == 0
    assert proto.sequence_acceptable(1, 1)


# --------------------------------------------------------------------------
# Decoders, fed with the real captured replies
# --------------------------------------------------------------------------


def test_decode_real_uid_reply():
    frame = hexbytes("01 03 0C C2 3B 37 81 15 B0 92 08 38 30 36 4A 78 12")
    registers = proto.parse_read_response(frame[1:-2], 6)
    assert proto.uid_from_registers(registers) == "C23B378115B092083830364A"


def test_decode_real_board_info_reply():
    """The captured request read 0x0001..0x0004, so pad 0x0000 and 0x0005..0x0007."""
    frame = hexbytes("01 03 08 00 00 00 01 00 01 61 A8 D1 F9")
    values = proto.parse_read_response(frame[1:-2], 4)
    info = proto.BoardInfo.from_registers([0] + values + [0, 0, 0])
    assert info.protocol == 0 and info.address == 1 and info.node == 1
    assert info.pwm_hz == 25000
    assert info.dirty is False and info.storage_error is False and info.pending_save is False
    assert info.save_clean is True


def test_decode_real_save_flags_reply():
    frame = hexbytes("01 03 06 00 00 00 00 00 00 21 75")
    values = proto.parse_read_response(frame[1:-2], 3)
    assert values == [0, 0, 0]


def test_decode_real_m3_status_reply():
    frame = hexbytes("01 03 0E 00 01 00 00 00 01 00 01 00 00 00 00 00 00 F0 C4")
    status = proto.ChannelStatus.from_registers(proto.parse_read_response(frame[1:-2], 7))
    assert status.state == int(proto.MotionState.IDLE)
    assert status.fault == 0
    assert status.valid is True
    assert status.brake is True
    assert status.busy is False
    assert status.state_name == "idle"


def test_decode_real_m4_negative_angle_reply():
    frame = hexbytes("01 03 0C FF FF FB F1 FF FF FF 72 FF FF FB F1 64 5F")
    position = proto.ChannelPosition.from_registers(proto.parse_read_response(frame[1:-2], 6))
    assert position.angle_deci_deg == -1039
    assert position.fg_count == -142
    assert position.target_deci_deg == -1039
    assert proto.deci_deg_to_rad(position.angle_deci_deg) == pytest.approx(-103.9 * math.pi / 180.0)


def test_decode_real_config_reply():
    frame = hexbytes("01 03 0A 00 01 00 02 02 04 00 00 1F 40 F3 04")
    config = proto.ChannelConfigValues.from_registers(proto.parse_read_response(frame[1:-2], 5))
    assert config.enabled is True
    assert config.role == int(proto.Role.LEFT_UPPER)
    assert config.gear == 516
    assert config.inverted == 0
    assert config.rpm == 8000
    assert config.to_registers() == [1, 2, 516, 0, 8000]


def test_decode_real_protocol_revision_reply():
    frame = hexbytes("01 03 02 00 02 39 85")
    assert proto.parse_read_response(frame[1:-2], 1) == [proto.COMMAND_PROTOCOL_REVISION]


def test_decode_real_last_sequence_reply():
    frame = hexbytes("01 03 04 00 90 00 00 FA 1E")
    assert proto.parse_read_response(frame[1:-2], 2) == [144, 0]


def test_parse_read_response_rejects_mismatched_count():
    pdu = hexbytes("03 04 00 01 00 02")
    with pytest.raises(proto.FrameError):
        proto.parse_read_response(pdu, 3)
    with pytest.raises(proto.FrameError):
        proto.parse_read_response(b"\x04\x02\x00\x01", 1)
    with pytest.raises(proto.FrameError):
        proto.parse_read_response(b"\x03", 1)


def test_validate_write_echo_detects_desync():
    request = proto.write_multi_pdu(proto.REG_COMMAND, [1, 3, 0, 0, 100])
    proto.validate_write_echo(request[:6], request)
    with pytest.raises(proto.FrameError):
        proto.validate_write_echo(hexbytes("10 00 31 00 05"), request)


def test_selftest_record_decoding():
    summary = [4, 0, 0, 6, 1200, 0, 45000, 7]
    # residual = -1 as two 16 bit words, high word first, two's complement
    detail = [5, 900, 0, 6, 0, 6, 0xFFFF, 0xFFFF]
    record = proto.SelftestRecord.from_registers(summary, detail)
    assert record.state == int(proto.SelftestState.PASSED)
    assert record.pulses == 6
    assert record.powered_ms == 1200
    assert record.completed_uptime_ms == 45000
    assert record.sequence == 7
    assert record.phase == int(proto.SelftestPhase.FINISHED)
    assert record.outbound_pulses == 6 and record.return_pulses == 6
    assert record.residual_pulses == -1
    assert record.finished is True


def test_selftest_residual_decodes_the_real_captured_pattern():
    """From the historical capture `... 00 05 00 00 00 08 FF FF FF FD ...`:
    outbound 5, return 8, residual -3."""
    detail = [5, 0x0A, 0, 5, 0, 8, 0xFFFF, 0xFFFD]
    record = proto.SelftestRecord.from_registers([5, 0, 0, 5, 1000, 0, 1, 3], detail)
    assert record.outbound_pulses == 5
    assert record.return_pulses == 8
    assert record.residual_pulses == -3


def test_enum_name_helpers_never_raise():
    assert proto.opcode_name(3) == "move"
    assert proto.opcode_name(99) == "opcode99"
    assert proto.state_name(5) == "done"
    assert proto.fault_name(4) == "unexpected_fg"
    assert proto.business_result_name(2) == "busy"
    assert proto.role_name(6) == "left_palm"


def test_exception_hint_mapping_is_ambiguous_where_documented():
    """0x03 and 0x04 each cover two business results; only 0x0036/0x0037 disambiguates."""
    assert proto.AMBIGUOUS_EXCEPTIONS[0x03] == (proto.BusinessResult.VALUE,
                                                proto.BusinessResult.SEQUENCE)
    assert proto.AMBIGUOUS_EXCEPTIONS[0x04] == (proto.BusinessResult.UNREADY,
                                                proto.BusinessResult.STORAGE)
    assert 0x02 not in proto.AMBIGUOUS_EXCEPTIONS
    assert 0x06 not in proto.AMBIGUOUS_EXCEPTIONS


def test_result_to_exception_mapping_covers_every_business_result():
    for result in proto.BusinessResult:
        if result == proto.BusinessResult.OK:
            continue
        assert int(result) in proto.RESULT_TO_EXCEPTION


def test_struct_pack_sanity_for_big_endian_registers():
    assert struct.pack(">2H", 0xC23B, 0x3781) == hexbytes("C2 3B 37 81")


# --------------------------------------------------------------------------
# Board image validation, transcribed from robot_config_valid (BSP/robot.c:11-23)
# --------------------------------------------------------------------------


def image(gear_a=516, gear_b=516, gear_c=290, gear_d=82):
    return [
        proto.ChannelConfigValues(True, 2, gear_a, 0, 8000),
        proto.ChannelConfigValues(True, 1, gear_b, 0, 8000),
        proto.ChannelConfigValues(True, 4, gear_c, 0, 8000),
        proto.ChannelConfigValues(True, 6, gear_d, 0, 8000),
    ]


def test_validate_board_image_accepts_the_audited_configuration():
    assert proto.validate_board_image(image(), [0, 1, 1, 25000]) == []


def test_validate_board_image_rejects_duplicate_role_on_a_disabled_channel():
    """BSP/robot.c:20 checks role uniqueness for every non-zero role, enabled or not.

    Filtering by `enabled` before this check is exactly how a driver ends up staging an
    image the board rejects with R_VALUE after the flash write was already prepared.
    """
    channels = image()
    # image() gives roles [2, 1, 4, 6]; make channel 1 DISABLED but still holding
    # role 2, which collides with channel 0. The firmware rejects that image.
    channels[1] = proto.ChannelConfigValues(False, 2, 0, 0, 8000)
    reasons = proto.validate_board_image(channels)
    assert any("duplicates" in r and "disabled" in r for r in reasons), reasons


def test_validate_board_image_checks_rpm_and_gear_for_disabled_channels():
    channels = image()
    channels[1] = proto.ChannelConfigValues(False, 0, 0, 0, 100)     # rpm below 400
    reasons = proto.validate_board_image(channels)
    assert any("rpm must be 400..8000 even when disabled" in r for r in reasons), reasons

    channels = image()
    channels[2] = proto.ChannelConfigValues(False, 0, 100, 0, 8000)  # gear not in the set
    reasons = proto.validate_board_image(channels)
    assert any("gear must be one of" in r for r in reasons), reasons


def test_validate_board_image_requires_gear_and_role_when_enabled():
    channels = image()
    channels[3] = proto.ChannelConfigValues(True, 6, 0, 0, 8000)     # enabled, gear 0
    reasons = proto.validate_board_image(channels)
    assert any("enabled channel needs a non-zero gear and role" in r for r in reasons), reasons

    channels = image()
    channels[3] = proto.ChannelConfigValues(True, 0, 82, 0, 8000)    # enabled, role 0
    assert any("enabled channel needs" in r for r in proto.validate_board_image(channels))


def test_validate_board_image_checks_board_level_fields():
    assert proto.validate_board_image(image(), [0, 1, 1, 25000]) == []
    assert any("protocol" in r for r in proto.validate_board_image(image(), [2, 1, 1, 25000]))
    assert any("address" in r for r in proto.validate_board_image(image(), [0, 248, 1, 25000]))
    assert any("node" in r for r in proto.validate_board_image(image(), [0, 1, 128, 25000]))
    assert any("pwm_hz" in r for r in proto.validate_board_image(image(), [0, 1, 1, 19999]))


def test_validate_board_image_requires_four_channels():
    reasons = proto.validate_board_image(image()[:3])
    assert any("exactly 4 channels" in r for r in reasons), reasons


def test_validate_board_image_rejects_out_of_range_flags():
    channels = image()
    channels[0] = proto.ChannelConfigValues(True, 2, 516, 3, 8000)   # inverted 3
    assert any("inverted must be 0 or 1" in r for r in proto.validate_board_image(channels))
    channels = image()
    channels[0] = proto.ChannelConfigValues(True, 9, 516, 0, 8000)   # role 9
    assert any("role must be 0..7" in r for r in proto.validate_board_image(channels))


def test_validate_board_image_allows_repeated_unassigned_role():
    """role 0 means unassigned and may repeat; only assigned roles must be unique."""
    channels = [proto.ChannelConfigValues(False, 0, 0, 0, 8000) for _ in range(4)]
    assert proto.validate_board_image(channels) == []

@pytest.mark.parametrize("degrees, expected", [
    (0.0,0), (5.0,50), (-5.0,-50), (0.049,0), (-0.049,0),
    (0.05,1), (-0.05,-1), (0.15,2), (-0.15,-2),
    (360.0,3600), (-360.0,-3600)])
def test_degree_conversion_boundaries(degrees, expected):
    assert proto.degrees_to_deci_deg(degrees) == expected
    assert proto.deci_deg_to_degrees(expected) == expected / 10.0

@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_degree_conversion_rejects_nonfinite(value):
    with pytest.raises(ValueError):
        proto.degrees_to_deci_deg(value)
