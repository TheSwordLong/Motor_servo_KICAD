"""Configuration tests: YAML/dict parsing, validation domains, lookup helpers.

Offline only. No hardware, no network, no ROS: `config` is importable without
rclpy and most tests drive `parse_config` with plain dicts, so they do not even
need pyyaml. The few tests that parse YAML text are marked and skip cleanly on an
interpreter without pyyaml (the remote build target has pyyaml 5.3.1).

The reference configuration is the one printed in section 7 of
docs/ROS2_头部与双臂控制_代码生成执行方案.md, kept here twice on purpose: once as
the verbatim YAML text (DOC7_YAML) and once as the equivalent mapping
(doc7_data()). test_doc7_dict_mirrors_doc7_yaml pins the two together, so the
dict-based tests cannot drift away from the documented file.

Runnable under plain python too: every test is a zero argument function using
only pytest.raises / pytest.mark.skipif / pytest.skip, so a caller can import this
module and invoke the test_* functions directly.
"""

import copy
import math
import os
import sys
import tempfile

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from robot_motor_driver import config as cfg
from robot_motor_driver import protocol as proto

try:
    import yaml
    HAVE_YAML = True
except ImportError:  # pragma: no cover - depends on the interpreter
    yaml = None
    HAVE_YAML = False

needs_yaml = pytest.mark.skipif(
    not HAVE_YAML, reason="pyyaml is not installed for this interpreter")

UID_MAIN = "C23B378115B092083830364A"
UID_SECOND = "C23B37C715B092083830364A"
BOARD_PORT = "/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0"

#: Section 7 of the task book, byte for byte (the port is the stable by-path name
#: observed on the audited board, so it survives a re-enumeration to ttyUSB0).
DOC7_YAML = """\
driver:
  allow_motion: false
  allow_unreferenced_relative: false
  polling_hz: 5.0
  board_motion_policy: serial
  speed_policy: keep_requested
boards:
  main_board:
    uid: "C23B378115B092083830364A"
    transport: modbus_rtu
    port: "/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0"
    address: 1
    baudrate: 115200
joints:
  head_joint:
    board: main_board
    channel: 2
    role: 1
    gear_ratio: 516
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  left_upper_arm_joint:
    board: main_board
    channel: 1
    role: 2
    gear_ratio: 516
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  left_forearm_joint:
    board: main_board
    channel: 3
    role: 4
    gear_ratio: 290
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  left_wrist_joint:
    board: main_board
    channel: 4
    role: 6
    gear_ratio: 82
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
"""

#: doc table: joint -> (yaml channel, protocol channel, role, gear ratio)
DOC7_JOINTS = {
    "head_joint": (2, 1, int(proto.Role.HEAD), 516),
    "left_upper_arm_joint": (1, 0, int(proto.Role.LEFT_UPPER), 516),
    "left_forearm_joint": (3, 2, int(proto.Role.LEFT_FOREARM), 290),
    "left_wrist_joint": (4, 3, int(proto.Role.LEFT_PALM), 82),
}


def doc7_data():
    """The section 7 configuration as an already parsed mapping."""
    joints = {}
    for name, (channel, _protocol_channel, role, gear) in DOC7_JOINTS.items():
        joints[name] = {
            "board": "main_board",
            "channel": channel,
            "role": role,
            "gear_ratio": gear,
            "inverted": 0,
            "default_motor_rpm": 400,
            "limits_rad": None,
        }
    return {
        "driver": {
            "allow_motion": False,
            "allow_unreferenced_relative": False,
            "polling_hz": 5.0,
            "board_motion_policy": "serial",
            "speed_policy": "keep_requested",
        },
        "boards": {
            "main_board": {
                "uid": UID_MAIN,
                "transport": "modbus_rtu",
                "port": BOARD_PORT,
                "address": 1,
                "baudrate": 115200,
            },
        },
        "joints": joints,
    }


def joint_of(data, name):
    """Mutable view of one joint entry, for tests that break exactly one field."""
    return data["joints"][name]


def board_of(data, name="main_board"):
    """Mutable view of one board entry."""
    return data["boards"][name]


def only_joints(data, *names):
    """Drop every joint except the named ones."""
    data["joints"] = dict((name, data["joints"][name]) for name in names)
    return data


def expect_config_error(data, needle, why):
    """Assert that parsing fails and that the message names the broken field.

    `needle` is matched case-insensitively so tests stay readable; `why` is the
    assertion message shown when the rule is no longer enforced.
    """
    try:
        cfg.parse_config(data)
    except cfg.ConfigError as exc:
        text = str(exc)
        assert needle.lower() in text.lower(), \
            "%s: ConfigError message %r does not mention %r" % (why, text, needle)
        return text
    raise AssertionError("%s: parse_config accepted an invalid configuration" % why)


# --------------------------------------------------------------------------
# The documented configuration loads, and the numbering conversion is right
# --------------------------------------------------------------------------


@needs_yaml
def test_doc7_yaml_loads_verbatim():
    """Section 7 of the task book must load unmodified."""
    config = cfg.load_config_from_text(DOC7_YAML)
    assert sorted(config.boards) == ["main_board"], config.boards
    assert sorted(config.joints) == sorted(DOC7_JOINTS), config.joints
    board = config.boards["main_board"]
    assert board.uid == UID_MAIN, board.uid
    assert board.transport == "modbus_rtu", board.transport
    assert board.port == BOARD_PORT, board.port
    assert board.address == 1 and board.baudrate == 115200, board
    assert config.allow_motion is False, "the documented default must be read-only"
    assert config.allow_unreferenced_relative is False, config
    assert config.polling_hz == 5.0, config.polling_hz
    assert config.board_motion_policy == "serial", config.board_motion_policy
    assert config.speed_policy == "keep_requested", config.speed_policy


@needs_yaml
def test_doc7_dict_mirrors_doc7_yaml():
    """The dict used by every other test must equal the documented YAML."""
    assert yaml.safe_load(DOC7_YAML) == doc7_data(), \
        "doc7_data() drifted from DOC7_YAML; fix the mirror before trusting these tests"


def test_doc7_channel_role_gear_mapping():
    """YAML channels 2/1/3/4 become protocol 1/0/2/3, roles 1/2/4/6, gears as documented."""
    config = cfg.parse_config(doc7_data())
    for name, (yaml_channel, protocol_channel, role, gear) in DOC7_JOINTS.items():
        joint = config.joint(name)
        assert joint.yaml_channel == yaml_channel, \
            "%s: yaml_channel %r != documented %r" % (name, joint.yaml_channel, yaml_channel)
        assert joint.channel == protocol_channel, \
            "%s: protocol channel %r != %r" % (name, joint.channel, protocol_channel)
        assert joint.channel == proto.yaml_channel_to_protocol(yaml_channel), name
        assert joint.role == role, "%s: role %r != %r" % (name, joint.role, role)
        assert joint.gear_ratio == gear, "%s: gear %r != %r" % (name, joint.gear_ratio, gear)
        assert joint.inverted == 0, name
        assert joint.default_motor_rpm == 400, name
        assert joint.limits_rad is None, "%s: documented limits_rad is null" % name


def test_yaml_channel_is_attribute_not_call():
    """yaml_channel is a property: calling it would be a bug in the caller."""
    joint = cfg.parse_config(doc7_data()).joint("head_joint")
    assert isinstance(joint.yaml_channel, int), joint.yaml_channel
    assert not callable(joint.yaml_channel), "use joint.yaml_channel, not joint.yaml_channel()"


def test_channel_to_joint_speaks_protocol_numbering():
    """channel_to_joint takes 0..3, and rejects the YAML numbering 4."""
    config = cfg.parse_config(doc7_data())
    assert config.channel_to_joint("main_board", 1).name == "head_joint"
    assert config.channel_to_joint("main_board", 0).name == "left_upper_arm_joint"
    assert config.channel_to_joint("main_board", 2).name == "left_forearm_joint"
    assert config.channel_to_joint("main_board", 3).name == "left_wrist_joint"
    with pytest.raises(cfg.ConfigError) as excinfo:
        config.channel_to_joint("main_board", 4)
    assert "0..3" in str(excinfo.value), excinfo.value


def test_channel_to_joint_returns_none_for_unmapped_channel():
    """An unused channel is a normal None, not an error."""
    data = only_joints(doc7_data(), "head_joint", "left_wrist_joint")
    config = cfg.parse_config(data)
    assert config.channel_to_joint("main_board", 0) is None, "protocol channel 0 is unmapped"
    assert config.channel_to_joint("main_board", 2) is None, "protocol channel 2 is unmapped"


def test_joints_for_board_keeps_yaml_order():
    """Serial per-board polling depends on a stable declaration order."""
    config = cfg.parse_config(doc7_data())
    names = [joint.name for joint in config.joints_for_board("main_board")]
    assert names == ["head_joint", "left_upper_arm_joint",
                     "left_forearm_joint", "left_wrist_joint"], names


def test_unknown_joint_lookup_raises_config_error():
    """A goal naming a foreign joint must be rejected, not silently ignored."""
    config = cfg.parse_config(doc7_data())
    with pytest.raises(cfg.ConfigError) as excinfo:
        config.joint("right_wrist_joint")
    text = str(excinfo.value)
    assert "unknown joint" in text, text
    assert "right_wrist_joint" in text, text
    assert "head_joint" in text, "the message should list what IS configured: %s" % text


def test_unknown_board_lookup_raises_config_error():
    config = cfg.parse_config(doc7_data())
    for call in (lambda: config.joints_for_board("arm_board"),
                 lambda: config.channel_to_joint("arm_board", 0),
                 lambda: config.board("arm_board")):
        with pytest.raises(cfg.ConfigError) as excinfo:
            call()
        assert "unknown board" in str(excinfo.value), excinfo.value


# --------------------------------------------------------------------------
# Per-board uniqueness: channel and role
# --------------------------------------------------------------------------


def test_duplicate_channel_on_same_board_rejected():
    data = doc7_data()
    joint_of(data, "left_upper_arm_joint")["channel"] = 2  # head_joint already uses 2
    text = expect_config_error(data, "channel", "two joints claimed YAML channel 2")
    assert "head_joint" in text and "left_upper_arm_joint" in text, text


def test_same_channel_on_two_boards_is_allowed():
    """Uniqueness is per board: a second board has its own four channels."""
    data = doc7_data()
    data["boards"]["arm_board"] = {"uid": UID_SECOND, "transport": "fake"}
    joint_of(data, "left_upper_arm_joint")["board"] = "arm_board"
    joint_of(data, "left_upper_arm_joint")["channel"] = 2
    config = cfg.parse_config(data)
    assert config.joint("head_joint").board == "main_board"
    assert config.joint("left_upper_arm_joint").board == "arm_board"
    assert config.channel_to_joint("main_board", 1).name == "head_joint"
    assert config.channel_to_joint("arm_board", 1).name == "left_upper_arm_joint"


def test_duplicate_role_on_same_board_rejected():
    data = doc7_data()
    joint_of(data, "left_upper_arm_joint")["role"] = int(proto.Role.HEAD)  # head_joint has it
    text = expect_config_error(data, "role", "two joints claimed role HEAD on one board")
    assert "head_joint" in text and "left_upper_arm_joint" in text, text


def test_same_role_on_two_boards_is_allowed():
    """Role uniqueness is per board, exactly like channel uniqueness."""
    data = doc7_data()
    data["boards"]["arm_board"] = {"uid": UID_SECOND, "transport": "fake"}
    joint_of(data, "left_upper_arm_joint")["board"] = "arm_board"
    joint_of(data, "left_upper_arm_joint")["role"] = int(proto.Role.HEAD)
    config = cfg.parse_config(data)
    head = config.joint("head_joint")
    upper = config.joint("left_upper_arm_joint")
    assert head.role == int(proto.Role.HEAD), head
    assert upper.role == int(proto.Role.HEAD), \
        "the same role on a DIFFERENT board must be accepted: %r" % (upper,)
    assert head.board == "main_board" and upper.board == "arm_board", (head, upper)


def test_unassigned_role_may_repeat():
    """Role 0 means unassigned, and protocol.validate_roles_unique skips it."""
    data = doc7_data()
    joint_of(data, "head_joint")["role"] = 0
    joint_of(data, "left_upper_arm_joint")["role"] = 0
    config = cfg.parse_config(data)
    assert config.joint("head_joint").role == 0
    assert config.joint("left_upper_arm_joint").role == 0


def test_role_out_of_range_rejected():
    data = doc7_data()
    joint_of(data, "head_joint")["role"] = 8
    text = expect_config_error(data, "role", "role 8 does not exist in protocol.Role")
    assert "head_joint" in text, text
    assert "7" in text, "the message should list the legal range: %s" % text


def test_role_must_be_an_integer():
    data = doc7_data()
    joint_of(data, "head_joint")["role"] = "1"
    expect_config_error(data, "role", "a quoted role is a string, not a role number")


# --------------------------------------------------------------------------
# Joint field domains (all delegated to protocol.py)
# --------------------------------------------------------------------------


def test_gear_ratio_outside_firmware_set_rejected():
    """protocol.GEAR_RATIOS is (0, 82, 290, 516); anything else is a typo."""
    for bad in (100, 515, 517, -82, 1):
        data = doc7_data()
        joint_of(data, "left_forearm_joint")["gear_ratio"] = bad
        text = expect_config_error(data, "gear_ratio", "gear %r is not a firmware gear" % bad)
        assert "left_forearm_joint" in text, text


def test_gear_ratio_accepted_values():
    for gear in proto.GEAR_RATIOS:
        data = doc7_data()
        joint_of(data, "left_forearm_joint")["gear_ratio"] = gear
        config = cfg.parse_config(data)
        assert config.joint("left_forearm_joint").gear_ratio == gear, gear


def test_gear_ratio_missing_is_rejected():
    data = doc7_data()
    del joint_of(data, "head_joint")["gear_ratio"]
    text = expect_config_error(data, "gear_ratio", "gear_ratio has no safe default")
    assert "required" in text, text


def test_rpm_out_of_range_rejected():
    """rpm is the PRE-gear cruise speed, 400..8000 (BSP/robot.c:17)."""
    for bad in (proto.RPM_MIN - 1, proto.RPM_MAX + 1, 0, -400):
        data = doc7_data()
        joint_of(data, "left_wrist_joint")["default_motor_rpm"] = bad
        text = expect_config_error(data, "default_motor_rpm", "rpm %r is outside 400..8000" % bad)
        assert "left_wrist_joint" in text, text


def test_rpm_defaults_to_crawl_speed():
    data = doc7_data()
    del joint_of(data, "left_wrist_joint")["default_motor_rpm"]
    config = cfg.parse_config(data)
    assert config.joint("left_wrist_joint").default_motor_rpm == proto.RPM_MIN == 400


def test_expected_motor_rpm_is_the_current_name_and_the_legacy_key_still_works():
    """`default_motor_rpm` implied the driver would command that speed. It never did:
    MOVE carries only an angle and the firmware takes rpm from its own stored
    configuration (BSP/robot.c:334). The field is now an expectation the driver
    compares against, and the old key keeps loading so existing files do not break."""
    legacy = cfg.parse_config(doc7_data())
    joint = legacy.joint("head_joint")
    assert joint.expected_motor_rpm == 400
    assert joint.default_motor_rpm == 400, "the deprecated alias must keep working"

    data = doc7_data()
    del joint_of(data, "head_joint")["default_motor_rpm"]
    joint_of(data, "head_joint")["expected_motor_rpm"] = 1500
    assert cfg.parse_config(data).joint("head_joint").expected_motor_rpm == 1500

    # Both keys present: the current name wins.
    data = doc7_data()
    joint_of(data, "head_joint")["expected_motor_rpm"] = 1500
    assert cfg.parse_config(data).joint("head_joint").expected_motor_rpm == 1500

    # The new key is range checked exactly like the old one, and says its own name.
    data = doc7_data()
    joint_of(data, "head_joint")["expected_motor_rpm"] = 100
    text = expect_config_error(data, "expected_motor_rpm", "rpm 100 is outside 400..8000")
    assert "head_joint" in text, text


def test_inverted_must_be_zero_or_one():
    for bad in (2, -1, True):
        data = doc7_data()
        joint_of(data, "head_joint")["inverted"] = bad
        expect_config_error(data, "inverted", "inverted=%r is not a direction flag" % (bad,))


def test_inverted_defaults_to_zero():
    data = doc7_data()
    del joint_of(data, "head_joint")["inverted"]
    assert cfg.parse_config(data).joint("head_joint").inverted == 0


def test_channel_outside_yaml_range_rejected():
    """YAML channels are 1..4; 0 and 5 are protocol numbers leaking into config."""
    for bad in (0, 5, -1):
        data = doc7_data()
        joint_of(data, "head_joint")["channel"] = bad
        text = expect_config_error(data, "channel", "YAML channel %r is outside 1..4" % bad)
        assert "head_joint" in text, text


def test_channel_must_be_an_integer():
    data = doc7_data()
    joint_of(data, "head_joint")["channel"] = "2"
    expect_config_error(data, "channel", "a quoted channel is a string")


def test_joint_board_must_be_defined():
    data = doc7_data()
    joint_of(data, "head_joint")["board"] = "arm_board"
    text = expect_config_error(data, "board", "a joint pointed at a board that does not exist")
    assert "arm_board" in text and "main_board" in text, \
        "the message should name the bad board and the known ones: %s" % text


# --------------------------------------------------------------------------
# limits_rad
# --------------------------------------------------------------------------


def test_limits_rad_null_becomes_none():
    """null means unconfirmed; it must never be turned into a invented limit."""
    data = doc7_data()
    joint_of(data, "head_joint")["limits_rad"] = None
    joint = cfg.parse_config(data).joint("head_joint")
    assert joint.limits_rad is None, joint.limits_rad
    assert joint.limits_confirmed is False, "None limits must not read as confirmed"


def test_limits_rad_absent_becomes_none():
    data = doc7_data()
    del joint_of(data, "head_joint")["limits_rad"]
    assert cfg.parse_config(data).joint("head_joint").limits_rad is None


def test_limits_rad_valid_pair():
    data = doc7_data()
    joint_of(data, "head_joint")["limits_rad"] = [-1.5708, 1.5708]
    joint = cfg.parse_config(data).joint("head_joint")
    assert joint.limits_rad == (-1.5708, 1.5708), joint.limits_rad
    assert joint.limits_confirmed is True
    assert all(isinstance(value, float) for value in joint.limits_rad), joint.limits_rad


def test_limits_rad_accepts_integers_and_full_turn():
    data = doc7_data()
    joint_of(data, "head_joint")["limits_rad"] = [-math.pi, math.pi]
    joint = cfg.parse_config(data).joint("head_joint")
    assert joint.limits_rad == (-math.pi, math.pi), joint.limits_rad
    data = doc7_data()
    joint_of(data, "head_joint")["limits_rad"] = [-6, 6]
    assert cfg.parse_config(data).joint("head_joint").limits_rad == (-6.0, 6.0)


def test_limits_rad_wrong_length_rejected():
    for bad in ([1.0], [1.0, 2.0, 3.0], []):
        data = doc7_data()
        joint_of(data, "head_joint")["limits_rad"] = bad
        text = expect_config_error(data, "limits_rad", "limits_rad %r is not a pair" % (bad,))
        assert "2" in text, "the message should state the expected length: %s" % text


def test_limits_rad_nan_and_inf_rejected():
    for bad in ([float("nan"), 1.0], [0.0, float("inf")], [float("-inf"), 0.0]):
        data = doc7_data()
        joint_of(data, "head_joint")["limits_rad"] = bad
        text = expect_config_error(data, "finite", "limits_rad %r is not usable" % (bad,))
        assert "head_joint" in text, text


def test_limits_rad_min_not_below_max_rejected():
    for bad in ([1.0, 1.0], [1.0, -1.0], [0.0, -0.5]):
        data = doc7_data()
        joint_of(data, "head_joint")["limits_rad"] = bad
        expect_config_error(data, "min", "limits_rad %r is not ordered" % (bad,))


def test_limits_rad_beyond_one_turn_points_at_degrees():
    """A limit wider than +/-2 pi means degrees were written into a radian field."""
    for bad in ([-180.0, 180.0], [-7.0, 7.0], [-math.pi * 3, math.pi * 3]):
        data = doc7_data()
        joint_of(data, "head_joint")["limits_rad"] = bad
        text = expect_config_error(data, "degrees", "limits_rad %r looks like degrees" % (bad,))
        assert "head_joint" in text and "limits_rad" in text, text


def test_limits_rad_non_sequence_rejected():
    for bad in (1.5, "[-1.0, 1.0]", {"min": -1.0, "max": 1.0}):
        data = doc7_data()
        joint_of(data, "head_joint")["limits_rad"] = bad
        expect_config_error(data, "limits_rad", "limits_rad=%r is not a sequence" % (bad,))


def test_limits_rad_non_numeric_element_rejected():
    data = doc7_data()
    joint_of(data, "head_joint")["limits_rad"] = ["-1.0", 1.0]
    text = expect_config_error(data, "limits_rad", "a quoted number is a string")
    assert "min" in text, text


# --------------------------------------------------------------------------
# driver section
# --------------------------------------------------------------------------


def test_allow_motion_as_string_is_rejected():
    """"yes"/"true" as a STRING must not be coerced into permission to move."""
    for bad in ("yes", "true", "True", 1, 0, "false"):
        data = doc7_data()
        data["driver"]["allow_motion"] = bad
        text = expect_config_error(data, "allow_motion",
                                   "allow_motion=%r is not a real boolean" % (bad,))
        assert "boolean" in text, text


def test_allow_motion_defaults_to_false():
    """Read-only is the default: a config that omits the flag may not move."""
    data = doc7_data()
    del data["driver"]["allow_motion"]
    del data["driver"]["allow_unreferenced_relative"]
    config = cfg.parse_config(data)
    assert config.allow_motion is False
    assert config.allow_unreferenced_relative is False


def test_allow_motion_true_is_accepted_as_a_bool():
    data = doc7_data()
    data["driver"]["allow_motion"] = True
    assert cfg.parse_config(data).allow_motion is True


def test_allow_unreferenced_relative_as_string_is_rejected():
    data = doc7_data()
    data["driver"]["allow_unreferenced_relative"] = "yes"
    expect_config_error(data, "allow_unreferenced_relative",
                        "a quoted flag must not enable relative moves without a reference")


def test_polling_hz_must_be_positive_and_bounded():
    for bad in (0, 0.0, -5.0, 100.1, 1000):
        data = doc7_data()
        data["driver"]["polling_hz"] = bad
        text = expect_config_error(data, "polling_hz", "polling_hz=%r is out of range" % (bad,))
        assert "100" in text, "the message should state the upper bound: %s" % text


def test_polling_hz_boundaries_accepted():
    for good in (0.5, 5.0, 100, 100.0):
        data = doc7_data()
        data["driver"]["polling_hz"] = good
        assert cfg.parse_config(data).polling_hz == float(good), good


def test_polling_hz_defaults_to_five():
    data = doc7_data()
    del data["driver"]["polling_hz"]
    assert cfg.parse_config(data).polling_hz == 5.0


def test_polling_hz_must_be_a_number():
    data = doc7_data()
    data["driver"]["polling_hz"] = "5.0"
    expect_config_error(data, "polling_hz", "a quoted rate is a string")


def test_board_motion_policy_parallel_rejected():
    """The first version drives one board at a time; parallel is not implemented."""
    data = doc7_data()
    data["driver"]["board_motion_policy"] = "parallel"
    text = expect_config_error(data, "board_motion_policy", "parallel motion is not supported")
    assert "serial" in text, text


def test_board_motion_policy_defaults_to_serial():
    data = doc7_data()
    del data["driver"]["board_motion_policy"]
    assert cfg.parse_config(data).board_motion_policy == "serial"


def test_speed_policy_domain():
    for good in ("keep_requested", "restore_previous"):
        data = doc7_data()
        data["driver"]["speed_policy"] = good
        assert cfg.parse_config(data).speed_policy == good, good
    for bad in ("fastest", "keep", "", "restore"):
        data = doc7_data()
        data["driver"]["speed_policy"] = bad
        expect_config_error(data, "speed_policy", "speed_policy=%r is not a known policy" % (bad,))


def test_speed_policy_defaults_to_keep_requested():
    data = doc7_data()
    del data["driver"]["speed_policy"]
    assert cfg.parse_config(data).speed_policy == "keep_requested"


def test_driver_section_may_be_empty_or_null():
    """An empty driver section means 'all defaults', which is the safe read-only set."""
    for value in ({}, None):
        data = doc7_data()
        data["driver"] = value
        config = cfg.parse_config(data)
        assert config.allow_motion is False, value
        assert config.polling_hz == 5.0, value
        assert config.speed_policy == "keep_requested", value


def test_driver_section_must_be_a_mapping():
    data = doc7_data()
    data["driver"] = ["allow_motion"]
    expect_config_error(data, "driver", "a list is not a driver section")


# --------------------------------------------------------------------------
# boards section
# --------------------------------------------------------------------------


def test_uid_must_be_24_hex_characters():
    for bad in ("C23B378115B092083830364", "C23B378115B092083830364AB", "not-a-uid",
                "C23B378115B092083830364G", 1234, None):
        data = doc7_data()
        board_of(data)["uid"] = bad
        text = expect_config_error(data, "uid", "uid=%r is not a 24 hex UID" % (bad,))
        assert "main_board" in text, text


def test_uid_is_normalised_to_upper_case():
    data = doc7_data()
    board_of(data)["uid"] = UID_MAIN.lower()
    assert cfg.parse_config(data).boards["main_board"].uid == UID_MAIN


def test_uid_missing_is_rejected():
    data = doc7_data()
    del board_of(data)["uid"]
    text = expect_config_error(data, "uid", "UID binding is mandatory for every board")
    assert "required" in text, text


def test_duplicate_uid_across_boards_rejected():
    """One UID is one physical board; two entries would write to the wrong hardware."""
    data = doc7_data()
    data["boards"]["copy_board"] = {"uid": UID_MAIN, "transport": "fake"}
    text = expect_config_error(data, "uid", "the same UID was declared twice")
    assert "main_board" in text and "copy_board" in text, text


def test_transport_domain():
    for bad in ("modbus_tcp", "serial", "MODBUS", "rtu"):
        data = doc7_data()
        board_of(data)["transport"] = bad
        text = expect_config_error(data, "transport", "transport=%r is unknown" % (bad,))
        assert "modbus_rtu" in text, \
            "the message should list the accepted transports: %s" % text
    data = doc7_data()
    board_of(data)["transport"] = ""
    expect_config_error(data, "transport", "an empty transport is not a choice")


def test_transport_is_case_insensitive():
    for name in ("modbus_rtu", "MODBUS_RTU", "Modbus_RTU"):
        data = doc7_data()
        board_of(data)["transport"] = name
        assert cfg.parse_config(data).boards["main_board"].transport == "modbus_rtu", name


def test_transport_fake_and_can_need_no_port():
    for transport in ("fake", "can"):
        data = doc7_data()
        board_of(data)["transport"] = transport
        del board_of(data)["port"]
        board = cfg.parse_config(data).boards["main_board"]
        assert board.transport == transport, transport
        assert board.port == "", board.port


def test_modbus_rtu_requires_a_port():
    data = doc7_data()
    del board_of(data)["port"]
    text = expect_config_error(data, "port", "modbus_rtu without a port cannot be opened")
    assert "main_board" in text, text


def test_modbus_rtu_rejects_an_empty_port():
    for bad in ("", "   "):
        data = doc7_data()
        board_of(data)["port"] = bad
        expect_config_error(data, "port", "port=%r is not a device path" % (bad,))


def test_port_is_stripped_but_kept_verbatim_otherwise():
    data = doc7_data()
    board_of(data)["port"] = "  %s  " % BOARD_PORT
    assert cfg.parse_config(data).boards["main_board"].port == BOARD_PORT


def test_modbus_defaults_and_domains():
    data = doc7_data()
    del board_of(data)["address"]
    del board_of(data)["baudrate"]
    board = cfg.parse_config(data).boards["main_board"]
    assert board.address == 1 and board.baudrate == 115200, board
    assert board.node == proto.NODE_MIN and board.pwm_hz == proto.PWM_HZ_DEFAULT, board

    for bad in (0, 248, -1):
        data = doc7_data()
        board_of(data)["address"] = bad
        expect_config_error(data, "address", "modbus address %r is outside 1..247" % (bad,))

    for bad in (0, -115200):
        data = doc7_data()
        board_of(data)["baudrate"] = bad
        expect_config_error(data, "baudrate", "baudrate %r cannot open a port" % (bad,))


def test_two_modbus_boards_may_share_address_one():
    """Independent USB serial ports are independent buses (task book section 8.F)."""
    data = doc7_data()
    data["boards"]["arm_board"] = {"uid": UID_SECOND, "transport": "modbus_rtu",
                                   "port": "/dev/ttyUSB9", "address": 1}
    config = cfg.parse_config(data)
    assert config.boards["main_board"].address == 1
    assert config.boards["arm_board"].address == 1


def test_pwm_hz_domain():
    for bad in (proto.PWM_HZ_MIN - 1, proto.PWM_HZ_MAX + 1, 0):
        data = doc7_data()
        board_of(data)["pwm_hz"] = bad
        expect_config_error(data, "pwm_hz", "pwm_hz %r is outside 20000..30000" % (bad,))
    for good in (proto.PWM_HZ_MIN, 25000, proto.PWM_HZ_MAX):
        data = doc7_data()
        board_of(data)["pwm_hz"] = good
        assert cfg.parse_config(data).boards["main_board"].pwm_hz == good, good


def test_can_node_defaults_and_must_be_unique():
    data = doc7_data()
    board_of(data)["transport"] = "can"
    del board_of(data)["port"]
    assert cfg.parse_config(data).boards["main_board"].node == 1

    data = doc7_data()
    board_of(data)["transport"] = "can"
    data["boards"]["arm_board"] = {"uid": UID_SECOND, "transport": "can"}
    text = expect_config_error(data, "node", "two CAN boards defaulted to the same node")
    assert "main_board" in text and "arm_board" in text, text

    data = doc7_data()
    board_of(data)["transport"] = "can"
    data["boards"]["arm_board"] = {"uid": UID_SECOND, "transport": "can", "node": 2}
    config = cfg.parse_config(data)
    assert config.boards["main_board"].node == 1 and config.boards["arm_board"].node == 2


def test_can_node_domain():
    for bad in (0, 128, -1):
        data = doc7_data()
        board_of(data)["transport"] = "can"
        board_of(data)["node"] = bad
        expect_config_error(data, "node", "CAN node %r is outside 1..127" % (bad,))


def test_node_uniqueness_only_applies_to_can():
    """A node id is meaningless on modbus_rtu, so it must not cause a clash there."""
    data = doc7_data()
    data["boards"]["arm_board"] = {"uid": UID_SECOND, "transport": "modbus_rtu",
                                   "port": "/dev/ttyUSB9", "node": 1}
    config = cfg.parse_config(data)
    assert config.boards["arm_board"].node == 1


def test_board_entry_must_be_a_mapping():
    data = doc7_data()
    data["boards"]["main_board"] = UID_MAIN
    expect_config_error(data, "main_board", "a board entry has to be a mapping")


def test_boards_section_empty_rejected():
    data = doc7_data()
    data["boards"] = {}
    expect_config_error(data, "boards", "an empty boards section cannot drive anything")


def test_joints_section_empty_rejected():
    data = doc7_data()
    data["joints"] = {}
    expect_config_error(data, "joints", "a driver with no joints has nothing to publish")


def test_joints_as_list_rejected_with_a_clear_message():
    """A list of joints has no names, so it is refused instead of being guessed at."""
    data = doc7_data()
    data["joints"] = [dict(name="head_joint", **joint_of(data, "head_joint"))]
    text = expect_config_error(data, "joints", "joints must be a mapping keyed by joint name")
    assert "mapping" in text, text


def test_missing_sections_are_reported_by_name():
    for section in ("driver", "boards", "joints"):
        data = doc7_data()
        del data[section]
        text = expect_config_error(data, section, "section %r is mandatory" % section)
        assert "missing" in text, text


def test_root_must_be_a_mapping():
    for bad in ([doc7_data()], None, "driver:", 42):
        try:
            cfg.parse_config(bad)
        except cfg.ConfigError as exc:
            assert "mapping" in str(exc), exc
        else:
            raise AssertionError("parse_config accepted a non-mapping root: %r" % (bad,))


def test_parse_config_does_not_mutate_its_input():
    """motor_node may keep the raw dict around; parsing must not edit it."""
    data = doc7_data()
    before = copy.deepcopy(data)
    cfg.parse_config(data)
    assert data == before, "parse_config modified the caller's mapping"


# --------------------------------------------------------------------------
# load_config / load_config_from_text / summarise
# --------------------------------------------------------------------------


def test_load_config_missing_file_names_the_path():
    """No pyyaml needed: the existence check happens before any parsing."""
    missing = os.path.join(tempfile.gettempdir(), "dsh_no_such_robot_yaml_9182.yaml")
    assert not os.path.exists(missing), missing
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_config(missing)
    text = str(excinfo.value)
    assert missing in text, "the operator must see WHICH file is missing: %s" % text
    assert "not found" in text, text


def test_load_config_rejects_a_directory():
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_config(tempfile.gettempdir())
    assert "not a regular file" in str(excinfo.value), excinfo.value


def test_load_config_rejects_a_non_path():
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_config(42)
    assert "path" in str(excinfo.value), excinfo.value


@needs_yaml
def test_load_config_reads_the_documented_file():
    handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8")
    try:
        handle.write(DOC7_YAML)
        handle.close()
        config = cfg.load_config(handle.name)
        assert len(config.joints) == 4, config.joints
        assert config.boards["main_board"].uid == UID_MAIN
        assert config.joint("head_joint").channel == 1
    finally:
        os.unlink(handle.name)


@needs_yaml
def test_load_config_invalid_yaml_names_the_path():
    handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8")
    try:
        handle.write("boards: [main_board\n  uid: broken\n")
        handle.close()
        with pytest.raises(cfg.ConfigError) as excinfo:
            cfg.load_config(handle.name)
        text = str(excinfo.value)
        assert handle.name in text, "a YAML syntax error must say which file: %s" % text
        assert "YAML" in text, text
    finally:
        os.unlink(handle.name)


@needs_yaml
def test_load_config_from_text_rejects_multiple_documents():
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_config_from_text(DOC7_YAML + "---\ndriver: {}\n")
    assert "YAML" in str(excinfo.value), excinfo.value


@needs_yaml
def test_duplicate_yaml_key_is_rejected():
    """pyyaml keeps the LAST duplicate silently, which would hide a copy-paste error."""
    text = DOC7_YAML.replace(
        "  left_wrist_joint:\n    board: main_board\n    channel: 4\n",
        "  left_wrist_joint:\n    board: main_board\n    channel: 3\n"
        "  left_wrist_joint:\n    board: main_board\n    channel: 4\n")
    assert text != DOC7_YAML
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_config_from_text(text)
    message = str(excinfo.value)
    assert "duplicate key" in message, message
    assert "left_wrist_joint" in message, message


@needs_yaml
def test_load_config_from_text_reports_a_config_error_not_a_yaml_error():
    text = DOC7_YAML.replace("gear_ratio: 290", "gear_ratio: 291")
    with pytest.raises(cfg.ConfigError) as excinfo:
        cfg.load_config_from_text(text)
    assert "gear_ratio" in str(excinfo.value), excinfo.value


def test_summarise_reports_boards_joints_and_motion_permission():
    config = cfg.parse_config(doc7_data())
    text = cfg.summarise(config)
    assert isinstance(text, str), text
    assert "1 board" in text, text
    assert "4 joint" in text, "the summary must state the joint count: %s" % text
    assert "allow_motion=False" in text, text
    assert "modbus_rtu" in text, text
    assert "\n" not in text, "the summary is meant for one log line: %r" % text


def test_summarise_reports_unconfirmed_limits_and_motion_enabled():
    data = doc7_data()
    data["driver"]["allow_motion"] = True
    joint_of(data, "head_joint")["limits_rad"] = [-1.0, 1.0]
    text = cfg.summarise(cfg.parse_config(data))
    assert "allow_motion=True" in text, text
    assert "3 joint(s)" in text, "three of four joints still have unconfirmed limits: %s" % text


def test_summarise_lists_every_transport():
    data = doc7_data()
    data["boards"]["sim_board"] = {"uid": UID_SECOND, "transport": "fake"}
    text = cfg.summarise(cfg.parse_config(data))
    assert "2 board" in text, text
    assert "fake" in text and "modbus_rtu" in text, text


# --------------------------------------------------------------------------
# Stage B constraint: no ROS, and yaml only inside the loader
# --------------------------------------------------------------------------


def test_config_module_does_not_import_ros():
    """Stage B requires the protocol/config layer to be testable without ROS.

    Checked in a SUBPROCESS on purpose: under `colcon test` the pytest process can
    already have rclpy in sys.modules because of a plugin in the sourced ROS
    environment, which would say nothing about this module's own imports.
    """
    import os
    import subprocess

    parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ)
    env["PYTHONPATH"] = parent + os.pathsep + env.get("PYTHONPATH", "")
    code = (
        "import sys, robot_motor_driver.config;"
        "bad = [m for m in ('rclpy', 'rosidl_runtime_py', 'sensor_msgs.msg', 'launch')"
        "       if m in sys.modules];"
        "print('IMPORTED_ROS:' + ','.join(bad) if bad else 'CLEAN')"
    )
    result = subprocess.run([sys.executable, "-c", code], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            universal_newlines=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "CLEAN", result.stdout


def test_config_module_does_not_import_yaml_at_module_scope():
    """A missing pyyaml must not break `import config`, only the load call."""
    assert "yaml" not in vars(cfg), "yaml must be imported lazily inside the loader"


@needs_yaml
def test_missing_pyyaml_message_is_actionable():
    """The lazy import has to produce a ConfigError naming pyyaml, not ImportError."""
    import builtins
    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "yaml":
            raise ImportError("No module named 'yaml'")
        return real_import(name, *args, **kwargs)

    saved = sys.modules.pop("yaml", None)
    builtins.__import__ = fake_import
    try:
        with pytest.raises(cfg.ConfigError) as excinfo:
            cfg.load_config_from_text(DOC7_YAML)
        assert "pyyaml" in str(excinfo.value), excinfo.value
    finally:
        builtins.__import__ = real_import
        if saved is not None:
            sys.modules["yaml"] = saved


def test_defaults_are_the_safe_read_only_set():
    """The smallest legal configuration must not allow motion."""
    data = {
        "driver": {},
        "boards": {"main_board": {"uid": UID_MAIN, "transport": "fake"}},
        "joints": {"head_joint": {"board": "main_board", "channel": 2,
                                  "role": 1, "gear_ratio": 516}},
    }
    config = cfg.parse_config(data)
    assert config.allow_motion is False
    assert config.allow_unreferenced_relative is False
    assert config.polling_hz == 5.0
    assert config.board_motion_policy == "serial"
    assert config.speed_policy == "keep_requested"
    board = config.boards["main_board"]
    assert (board.address, board.baudrate, board.node, board.pwm_hz) == (1, 115200, 1, 25000), board
    joint = config.joint("head_joint")
    assert (joint.inverted, joint.default_motor_rpm, joint.limits_rad) == (0, 400, None), joint


# --------------------------------------------------------------------------
# Multi-board deployments: shared modbus address, dedicated serial lines
# --------------------------------------------------------------------------


def test_two_ch340_boards_may_share_modbus_address_one():
    """The real two-board deployment: two CH340 TTL adapters on two USB sockets,
    and BOTH boards left at their factory address 1. Independent ports are
    independent buses, so neither the address rule nor the newer port-uniqueness
    rule may reject this (task book section 8.F; the generic form of the address
    rule is test_two_modbus_boards_may_share_address_one)."""
    data = doc7_data()
    data["boards"]["right_board"] = {
        "uid": UID_SECOND,
        "transport": "modbus_rtu",
        "port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        "address": 1,
        "baudrate": 115200,
    }
    config = cfg.parse_config(data)
    assert config.boards["main_board"].address == 1
    assert config.boards["right_board"].address == 1
    assert config.boards["main_board"].port != config.boards["right_board"].port


def test_two_boards_sharing_one_port_are_rejected():
    """Two board entries on one UART would interleave frames that both sides
    attribute to their own board, so a shared non-empty port is a startup error
    that names the port and both boards."""
    data = doc7_data()
    data["boards"]["copy_board"] = {"uid": UID_SECOND, "transport": "modbus_rtu",
                                    "port": BOARD_PORT, "address": 1}
    text = expect_config_error(data, "own serial line",
                               "two boards declared the same non-empty port")
    assert BOARD_PORT in text, text
    assert "main_board" in text and "copy_board" in text, \
        "the message must name both boards: %s" % text
    assert "RS-485" in text, "the message should warn against bus wiring: %s" % text

    # The rule is transport independent: two fake boards claiming the same port
    # string are the same copy-paste error, and empty-vs-empty must not match.
    data = doc7_data()
    board_of(data)["transport"] = "fake"
    board_of(data)["port"] = "/dev/ttyFAKE0"
    data["boards"]["copy_board"] = {"uid": UID_SECOND, "transport": "fake",
                                    "port": "/dev/ttyFAKE0"}
    text = expect_config_error(data, "own serial line", "fake boards are not exempt")
    assert "/dev/ttyFAKE0" in text, text
    assert "main_board" in text and "copy_board" in text, text


def test_two_fake_boards_with_empty_ports_are_accepted():
    """An empty port means "no serial line" (fake, or CAN by interface name
    elsewhere), so two boards without ports must NOT be treated as a clash --
    otherwise no multi-board simulation could ever load."""
    data = doc7_data()
    board_of(data)["transport"] = "fake"
    board_of(data)["port"] = ""                       # explicit empty port
    data["boards"]["sim_board"] = {"uid": UID_SECOND, "transport": "fake"}   # port omitted
    config = cfg.parse_config(data)
    assert config.boards["main_board"].port == ""
    assert config.boards["sim_board"].port == ""
    assert sorted(config.boards) == ["main_board", "sim_board"]


# Duplicate UIDs across two boards are already covered by
# test_duplicate_uid_across_boards_rejected above; the rule is unchanged by the
# multi-board work, so it is deliberately not duplicated here.

@pytest.mark.parametrize("limits", [None, [-90,90], [-360,360]])
def test_degree_limits(limits):
    data = doc7_data()
    name = next(iter(data["joints"]))
    entry = data["joints"][name]
    del entry["limits_rad"]
    entry["limits_deg"] = limits
    parsed = cfg.parse_config(data).joints[name].limits_rad
    assert parsed == (None if limits is None else tuple(math.radians(v) for v in limits))

@pytest.mark.parametrize("limits", [[90,-90], [-361,0], [0,361], [False,1],
                                    [0,float("nan")], [0,float("inf")], [0], "90"])
def test_invalid_degree_limits(limits):
    data = doc7_data()
    entry = next(iter(data["joints"].values()))
    del entry["limits_rad"]
    entry["limits_deg"] = limits
    with pytest.raises(cfg.ConfigError):
        cfg.parse_config(data)

def test_both_limit_units_rejected_even_when_null():
    data = doc7_data()
    next(iter(data["joints"].values()))["limits_deg"] = None
    with pytest.raises(cfg.ConfigError, match="not both"):
        cfg.parse_config(data)
