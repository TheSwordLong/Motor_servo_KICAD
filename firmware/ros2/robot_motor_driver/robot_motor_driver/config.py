"""Driver configuration: robot.yaml -> validated dataclasses, with no ROS import.

Pure Python 3.8 so the same file runs on the developer machine (3.13) and on the
remote build target (Ubuntu 20.04, Python 3.8.10, pyyaml 5.3.1). Nothing here
imports rclpy or any other ROS module, which is what keeps stage B of
docs/ROS2_头部与双臂控制_代码生成执行方案.md testable without a ROS installation.
`yaml` is imported lazily inside the loader so this module stays importable on a
machine without pyyaml; the failure then happens at load time with a clear message.

Design rules that a caller depends on:

  * The YAML channel numbering 1..4 is converted to the protocol numbering 0..3
    exactly once, here, through protocol.yaml_channel_to_protocol. Everything
    downstream (JointConfig.channel, channel_to_joint) speaks protocol numbers,
    and JointConfig.yaml_channel converts back for log messages and CONFIG frames.
  * Every domain check reuses protocol.py (validate_uid, validate_gear,
    validate_rpm, validate_pwm_hz, validate_address, validate_node,
    validate_boolean_flag, validate_roles_unique, Role). This module adds context
    to those errors, it never re-derives a firmware limit.
  * Every ConfigError names the section, the board or joint, and the field, so a
    config can be fixed on site without a debugger.
  * Duplicate YAML keys are rejected. pyyaml silently keeps the LAST value of a
    duplicated mapping key, which would hide a copy-paste error in robot.yaml, so
    the loader used here is a SafeLoader subclass that refuses duplicates.
  * limits_rad: null means "mechanical limits not yet confirmed". It is preserved
    as None, never invented, and the node must then stay read-only or simulated
    (task book section 7).
  * Unknown keys are ignored on purpose: a launch file may add annotations, and
    rejecting them would couple this parser to every future field. Typos in the
    keys that matter are still caught, because those keys are required or have a
    validated domain.
  * gear_ratio 0 is accepted because protocol.validate_gear accepts it; 0 means an
    unconfigured channel, and the firmware then refuses motion with R_UNREADY. A
    joint with gear 0 is therefore loadable but can never move.
"""

import math
import os
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import protocol as proto

# --------------------------------------------------------------------------
# Errors
# --------------------------------------------------------------------------


class ConfigError(Exception):
    """Every configuration problem: missing file, bad YAML, illegal value.

    The message always identifies the offending section, board or joint and the
    field name, because this is read by a human standing next to the robot.
    """


# --------------------------------------------------------------------------
# Accepted values and defaults
# --------------------------------------------------------------------------

SECTION_DRIVER = "driver"
SECTION_BOARDS = "boards"
SECTION_JOINTS = "joints"
REQUIRED_SECTIONS = (SECTION_DRIVER, SECTION_BOARDS, SECTION_JOINTS)

TRANSPORT_MODBUS_RTU = "modbus_rtu"
TRANSPORT_CAN = "can"
TRANSPORT_FAKE = "fake"
TRANSPORTS = (TRANSPORT_MODBUS_RTU, TRANSPORT_CAN, TRANSPORT_FAKE)

#: The first version drives one board at a time (task book section 8.F), so
#: "serial" is the only board motion policy. Parallel motion needs cross-board
#: failure simulation and stop verification before it may be offered.
BOARD_MOTION_POLICIES = ("serial",)

#: keep_requested avoids a CONFIG flash write per move; restore_previous puts the
#: previously applied rpm back, but only once the whole board is quiet.
SPEED_POLICIES = ("keep_requested", "restore_previous")

DEFAULT_ALLOW_MOTION = False
DEFAULT_ALLOW_UNREFERENCED_RELATIVE = False
DEFAULT_POLLING_HZ = 5.0
POLLING_HZ_MAX = 100.0
DEFAULT_BOARD_MOTION_POLICY = "serial"
DEFAULT_SPEED_POLICY = "keep_requested"

DEFAULT_PORT = ""
DEFAULT_BAUDRATE = 115200          # task book section 4.1: 115200 8N1
DEFAULT_MOTOR_RPM = proto.RPM_MIN  # 400, the firmware crawl speed

#: One full output shaft revolution. A limit outside +/-2 pi is a unit mistake:
#: degrees were written where radians belong.
LIMIT_MAX_RAD = 2.0 * math.pi
LIMIT_TOLERANCE_RAD = 1e-9

#: Sentinel for "no default": a key using it is required.
_MISSING = object()


# --------------------------------------------------------------------------
# Data classes
# --------------------------------------------------------------------------


@dataclass
class JointConfig(object):
    """One joint as configured, with the channel already in protocol numbering."""

    name: str
    board: str
    channel: int                       # protocol numbering, 0..3
    role: int
    gear_ratio: int
    inverted: int = 0
    #: The pre-gear rpm this driver EXPECTS the board to have stored for the channel.
    #: It is never written to the board: MOVE carries only an angle and the firmware
    #: takes speed from its own stored configuration (BSP/robot.c:334). A MoveJoint
    #: goal whose motor_rpm is non-zero and differs from the board's value is refused.
    expected_motor_rpm: int = DEFAULT_MOTOR_RPM
    limits_rad: Optional[Tuple[float, float]] = None

    @property
    def default_motor_rpm(self):
        # type: () -> int
        """Deprecated alias for expected_motor_rpm.

        Kept so older callers and configurations keep working. The old name suggested
        the driver would set this speed, which it does not and cannot do per move.
        """
        return self.expected_motor_rpm

    @property
    def yaml_channel(self):
        # type: () -> int
        """Channel number as written in YAML (1..4).

        This is an ATTRIBUTE, not a method: use `joint.yaml_channel`, never
        `joint.yaml_channel()`.
        """
        return self.channel + 1

    @property
    def limits_confirmed(self):
        # type: () -> bool
        """False while limits_rad is None, i.e. mechanical limits unconfirmed."""
        return self.limits_rad is not None


@dataclass
class BoardConfig(object):
    """One physical board and how to reach it.

    `address` only matters for modbus_rtu and `node` only for can; both are always
    populated so a node can log them without branching on transport. Two boards on
    two independent USB serial ports may legitimately share address 1 (task book
    section 8.F), so addresses are NOT required to be unique here.
    """

    name: str
    uid: str
    transport: str
    port: str = DEFAULT_PORT
    address: int = proto.ADDRESS_MIN
    baudrate: int = DEFAULT_BAUDRATE
    node: int = proto.NODE_MIN
    pwm_hz: int = proto.PWM_HZ_DEFAULT


@dataclass
class DriverConfig(object):
    """Whole driver configuration: policies, boards, joints."""

    allow_motion: bool = DEFAULT_ALLOW_MOTION
    allow_unreferenced_relative: bool = DEFAULT_ALLOW_UNREFERENCED_RELATIVE
    polling_hz: float = DEFAULT_POLLING_HZ
    board_motion_policy: str = DEFAULT_BOARD_MOTION_POLICY
    speed_policy: str = DEFAULT_SPEED_POLICY
    boards: Dict[str, BoardConfig] = field(default_factory=dict)
    joints: Dict[str, JointConfig] = field(default_factory=dict)

    def joint(self, name):
        # type: (str) -> JointConfig
        """Look one joint up by name; an unknown name is a ConfigError, not None.

        A goal naming a joint this driver does not own must be rejected, and the
        message lists what IS configured so the typo is obvious.
        """
        try:
            return self.joints[name]
        except (KeyError, TypeError):
            raise ConfigError("unknown joint %r; configured joints: %s"
                              % (name, _names_or_none(self.joints)))

    def board(self, name):
        # type: (str) -> BoardConfig
        """Look one board up by name; an unknown name is a ConfigError."""
        return self._board_or_raise(name)

    def joints_for_board(self, board_name):
        # type: (str) -> List[JointConfig]
        """Joints mapped onto one board, in YAML declaration order."""
        self._board_or_raise(board_name)
        return [joint for joint in self.joints.values() if joint.board == board_name]

    def channel_to_joint(self, board_name, channel):
        # type: (str, int) -> Optional[JointConfig]
        """Joint on one PROTOCOL channel (0..3) of a board, or None if unmapped.

        None is the normal answer for a channel that drives nothing; an unknown
        board or a channel outside 0..3 is a ConfigError, because both mean the
        caller is out of sync with this configuration.
        """
        self._board_or_raise(board_name)
        if isinstance(channel, bool) or not isinstance(channel, int) \
                or not 0 <= channel < proto.CHANNEL_COUNT:
            raise ConfigError(
                "channel must be a protocol channel 0..%d, got %r; YAML uses 1..4, "
                "convert with protocol.yaml_channel_to_protocol"
                % (proto.CHANNEL_COUNT - 1, channel))
        for joint in self.joints.values():
            if joint.board == board_name and joint.channel == channel:
                return joint
        return None

    def _board_or_raise(self, board_name):
        # type: (Any) -> BoardConfig
        try:
            board = self.boards.get(board_name)
        except (AttributeError, TypeError):
            board = None
        if board is None:
            raise ConfigError("unknown board %r; configured boards: %s"
                              % (board_name, _names_or_none(self.boards)))
        return board


# --------------------------------------------------------------------------
# Small helpers: every one of them turns a type problem into a ConfigError
# --------------------------------------------------------------------------


def _type_name(value):
    # type: (Any) -> str
    """YAML-ish type name for error messages (None reads as "null")."""
    if value is None:
        return "null"
    return type(value).__name__


def _names_or_none(mapping):
    # type: (Dict[str, Any]) -> str
    return ", ".join(sorted(mapping)) if mapping else "none"


def _require_mapping(value, where):
    # type: (Any, str) -> Dict[Any, Any]
    if not isinstance(value, dict):
        raise ConfigError("%s must be a mapping, got %s" % (where, _type_name(value)))
    return value


def _require_text(value, where, allow_empty=False):
    # type: (Any, str, bool) -> str
    if not isinstance(value, str):
        raise ConfigError("%s must be a string, got %r (%s)" % (where, value, _type_name(value)))
    text = value.strip()
    if not text and not allow_empty:
        raise ConfigError("%s must be a non-empty string" % where)
    return text


def _require_int(value, where):
    # type: (Any, str) -> int
    # bool is an int subclass in Python, and `inverted: true` is not a channel flag.
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError("%s must be an integer, got %r (%s)" % (where, value, _type_name(value)))
    return int(value)


def _require_number(value, where):
    # type: (Any, str) -> float
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError("%s must be a number, got %r (%s)" % (where, value, _type_name(value)))
    return float(value)


def _require_bool(value, where):
    # type: (Any, str) -> bool
    """Accept only a real YAML boolean.

    pyyaml already maps unquoted yes/no/true/false/on/off to bool, so a value that
    arrives as the STRING "yes" was quoted on purpose or came from a launch
    argument. Coercing it would let quoting decide whether the robot may move, so
    it is rejected instead.
    """
    if not isinstance(value, bool):
        raise ConfigError("%s must be a boolean (unquoted true/false), got %r (%s)"
                          % (where, value, _type_name(value)))
    return bool(value)


def _field(entry, key, where, default=_MISSING):
    # type: (Dict[Any, Any], str, str, Any) -> Any
    """Value of one key; a missing key and an explicit null both yield the default.

    With no default the key is required, and the message says so instead of
    letting a later type check produce something vague.
    """
    if key in entry and entry[key] is not None:
        return entry[key]
    if default is _MISSING:
        raise ConfigError("%s.%s is required and must not be null" % (where, key))
    return default


def _validated(func, value, where):
    # type: (Callable[[Any], Any], Any, str) -> Any
    """Run one protocol.py domain validator, adding config context to its error."""
    try:
        return func(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError("%s: %s" % (where, exc))


def _role_choices():
    # type: () -> str
    return ", ".join("%d=%s" % (int(role), role.name.lower()) for role in proto.Role)


def _parse_limits(value, where):
    # type: (Any, str) -> Optional[Tuple[float, float]]
    """Parse limits_rad into (min, max) radians, or None when unconfirmed.

    None is a legitimate answer meaning "limits not measured yet", and the node
    must then refuse absolute motion and stay read-only or simulated. Anything
    else has to be a two element numeric sequence that is finite, ordered, and
    inside one output revolution.
    """
    if value is None:
        return None
    if not isinstance(value, (list, tuple)):
        raise ConfigError("%s must be null or a two element [min, max] sequence, got %r (%s)"
                          % (where, value, _type_name(value)))
    if len(value) != 2:
        raise ConfigError("%s needs exactly 2 values [min, max], got %d: %r"
                          % (where, len(value), list(value)))
    numbers = []  # type: List[float]
    for label, item in zip(("min", "max"), value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ConfigError("%s %s must be a finite number in radians, got %r (%s)"
                              % (where, label, item, _type_name(item)))
        number = float(item)
        if not math.isfinite(number):
            raise ConfigError("%s %s must be finite, got %r; NaN and infinity are not "
                              "usable as a mechanical limit" % (where, label, item))
        numbers.append(number)
    low, high = numbers[0], numbers[1]
    if not low < high:
        raise ConfigError("%s min must be strictly less than max, got min=%r max=%r"
                          % (where, low, high))
    if low < -LIMIT_MAX_RAD - LIMIT_TOLERANCE_RAD or high > LIMIT_MAX_RAD + LIMIT_TOLERANCE_RAD:
        raise ConfigError(
            "%s must stay inside +/-%.6f rad (+/-360 deg output shaft), got [%.6f, %.6f]; "
            "a range this wide usually means the limits were written in DEGREES instead "
            "of radians" % (where, LIMIT_MAX_RAD, low, high))
    return (low, high)



def _parse_degree_limits(value, where):
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ConfigError("%s must be null or [min, max] in degrees" % where)
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
           for x in value):
        raise ConfigError("%s must contain finite degree numbers" % where)
    low, high = map(float, value)
    if not -360.0 <= low < high <= 360.0:
        raise ConfigError("%s must be ordered within [-360, 360] degrees" % where)
    # Retain the existing internal limit representation for both API adapters.
    return (math.radians(low), math.radians(high))


# --------------------------------------------------------------------------
# Section parsers
# --------------------------------------------------------------------------


def _parse_driver(raw):
    # type: (Any) -> Tuple[bool, bool, float, str, str]
    """Validate the driver section and return its five policy values."""
    where = SECTION_DRIVER
    entry = {} if raw is None else _require_mapping(raw, where)

    allow_motion = _require_bool(_field(entry, "allow_motion", where, DEFAULT_ALLOW_MOTION),
                                 where + ".allow_motion")
    allow_unreferenced = _require_bool(
        _field(entry, "allow_unreferenced_relative", where, DEFAULT_ALLOW_UNREFERENCED_RELATIVE),
        where + ".allow_unreferenced_relative")

    polling_hz = _require_number(_field(entry, "polling_hz", where, DEFAULT_POLLING_HZ),
                                 where + ".polling_hz")
    if not 0.0 < polling_hz <= POLLING_HZ_MAX:
        raise ConfigError("%s.polling_hz must be > 0 and <= %.1f Hz, got %r"
                          % (where, POLLING_HZ_MAX, polling_hz))

    board_motion_policy = _require_text(
        _field(entry, "board_motion_policy", where, DEFAULT_BOARD_MOTION_POLICY),
        where + ".board_motion_policy").lower()
    if board_motion_policy not in BOARD_MOTION_POLICIES:
        raise ConfigError("%s.board_motion_policy must be one of %s (the first version drives "
                          "one board at a time), got %r"
                          % (where, list(BOARD_MOTION_POLICIES), board_motion_policy))

    speed_policy = _require_text(_field(entry, "speed_policy", where, DEFAULT_SPEED_POLICY),
                                 where + ".speed_policy").lower()
    if speed_policy not in SPEED_POLICIES:
        raise ConfigError("%s.speed_policy must be one of %s, got %r"
                          % (where, list(SPEED_POLICIES), speed_policy))

    return (allow_motion, allow_unreferenced, polling_hz, board_motion_policy, speed_policy)


def _parse_boards(raw):
    # type: (Any) -> Dict[str, BoardConfig]
    """Validate the boards section; insertion order is preserved for serial polling."""
    where = SECTION_BOARDS
    entries = _require_mapping(raw, where)
    if not entries:
        raise ConfigError("%s must declare at least one board; an empty boards section "
                          "cannot drive anything" % where)

    boards = {}       # type: Dict[str, BoardConfig]
    uid_owner = {}    # type: Dict[str, str]
    node_owner = {}   # type: Dict[int, str]
    port_owner = {}   # type: Dict[str, str]

    for key, raw_board in entries.items():
        board_where = "%s.%s" % (where, key)
        name = _require_text(key, where + " key")
        entry = _require_mapping(raw_board, board_where)

        uid = _validated(proto.validate_uid, _field(entry, "uid", board_where),
                         board_where + ".uid")
        clash = uid_owner.get(uid)
        if clash is not None:
            raise ConfigError("%s.uid %s is already used by boards.%s; each entry must be a "
                              "distinct physical board, because the UID is what binds every "
                              "write to the right hardware" % (board_where, uid, clash))
        uid_owner[uid] = name

        transport = _require_text(_field(entry, "transport", board_where),
                                  board_where + ".transport").lower()
        if transport not in TRANSPORTS:
            raise ConfigError("%s.transport must be one of %s, got %r"
                              % (board_where, list(TRANSPORTS), transport))

        if transport == TRANSPORT_MODBUS_RTU:
            port = _require_text(_field(entry, "port", board_where), board_where + ".port")
        else:
            port = _require_text(_field(entry, "port", board_where, DEFAULT_PORT),
                                 board_where + ".port", allow_empty=True)

        baudrate = _require_int(_field(entry, "baudrate", board_where, DEFAULT_BAUDRATE),
                                board_where + ".baudrate")
        if baudrate <= 0:
            raise ConfigError("%s.baudrate must be positive, got %d" % (board_where, baudrate))

        # Two board entries on one port would mean two processes (or two transports in
        # one process) fighting over the same UART, with interleaved frames that both
        # sides would attribute to their own board. An empty port is allowed here for
        # the fake/CAN cases and is checked where it is actually used.
        if port:
            clash = port_owner.get(port)
            if clash is not None:
                raise ConfigError("%s.port %r is already used by boards.%s; each board needs its "
                                  "own serial line. Two boards must never share one port, and two "
                                  "TTL adapters must never be wired onto one RS-485 bus."
                                  % (board_where, port, clash))
            port_owner[port] = name

        address = _validated(
            proto.validate_address,
            _require_int(_field(entry, "address", board_where, proto.ADDRESS_MIN),
                         board_where + ".address"),
            board_where + ".address")

        node = _validated(
            proto.validate_node,
            _require_int(_field(entry, "node", board_where, proto.NODE_MIN),
                         board_where + ".node"),
            board_where + ".node")
        if transport == TRANSPORT_CAN:
            clash = node_owner.get(node)
            if clash is not None:
                raise ConfigError("%s.node %d is already used by boards.%s; CAN node ids must "
                                  "be unique on one bus" % (board_where, node, clash))
            node_owner[node] = name

        pwm_hz = _validated(
            proto.validate_pwm_hz,
            _require_int(_field(entry, "pwm_hz", board_where, proto.PWM_HZ_DEFAULT),
                         board_where + ".pwm_hz"),
            board_where + ".pwm_hz")

        boards[name] = BoardConfig(name=name, uid=uid, transport=transport, port=port,
                                   address=address, baudrate=baudrate, node=node, pwm_hz=pwm_hz)
    return boards


def _parse_joints(raw, boards):
    # type: (Any, Dict[str, BoardConfig]) -> Dict[str, JointConfig]
    """Validate the joints section against the already parsed boards."""
    where = SECTION_JOINTS
    entries = _require_mapping(raw, where)
    if not entries:
        raise ConfigError("%s must declare at least one joint; a driver with no joints has "
                          "nothing to publish" % where)

    joints = {}          # type: Dict[str, JointConfig]
    channel_owner = {}   # type: Dict[Tuple[str, int], str]
    role_owner = {}      # type: Dict[Tuple[str, int], str]
    board_roles = {}     # type: Dict[str, List[int]]

    for key, raw_joint in entries.items():
        joint_where = "%s.%s" % (where, key)
        name = _require_text(key, where + " key")
        entry = _require_mapping(raw_joint, joint_where)

        board_name = _require_text(_field(entry, "board", joint_where), joint_where + ".board")
        if board_name not in boards:
            raise ConfigError("%s.board %r is not defined under boards (known boards: %s)"
                              % (joint_where, board_name, _names_or_none(boards)))

        yaml_channel = _require_int(_field(entry, "channel", joint_where), joint_where + ".channel")
        channel = _validated(proto.yaml_channel_to_protocol, yaml_channel, joint_where + ".channel")
        clash = channel_owner.get((board_name, channel))
        if clash is not None:
            raise ConfigError("%s.channel %d (protocol channel %d) on board %s is already used by "
                              "joints.%s; one channel drives exactly one joint"
                              % (joint_where, yaml_channel, channel, board_name, clash))
        channel_owner[(board_name, channel)] = name

        role = _require_int(_field(entry, "role", joint_where), joint_where + ".role")
        try:
            role = int(proto.Role(role))
        except ValueError:
            raise ConfigError("%s.role must be one of %s, got %r"
                              % (joint_where, _role_choices(), role))
        roles = board_roles.setdefault(board_name, [])
        roles.append(role)
        try:
            # protocol.validate_roles_unique owns this rule (BSP/robot.c:19); the
            # loop below only adds the joint names to its message.
            proto.validate_roles_unique(roles)
        except ValueError as exc:
            clash = role_owner.get((board_name, role))
            if clash is not None:
                raise ConfigError("%s.role %d (%s) on board %s is already assigned to joints.%s; "
                                  "assigned roles must be unique per board"
                                  % (joint_where, role, proto.role_name(role), board_name, clash))
            raise ConfigError("%s.role: %s (board %s)" % (joint_where, exc, board_name))
        if role != int(proto.Role.UNASSIGNED):
            role_owner[(board_name, role)] = name

        gear_ratio = _validated(
            proto.validate_gear,
            _require_int(_field(entry, "gear_ratio", joint_where), joint_where + ".gear_ratio"),
            joint_where + ".gear_ratio")

        inverted = _validated(
            lambda value: proto.validate_boolean_flag(value, "inverted"),
            _require_int(_field(entry, "inverted", joint_where, 0), joint_where + ".inverted"),
            joint_where + ".inverted")

        # `expected_motor_rpm` is the current name; `default_motor_rpm` is still
        # accepted so existing YAML files keep loading. Both mean "the rpm this driver
        # expects the board to hold", never "the rpm to command".
        rpm_key = "expected_motor_rpm"
        if rpm_key not in entry and "default_motor_rpm" in entry:
            rpm_key = "default_motor_rpm"
        expected_motor_rpm = _validated(
            proto.validate_rpm,
            _require_int(_field(entry, rpm_key, joint_where, DEFAULT_MOTOR_RPM),
                         joint_where + "." + rpm_key),
            joint_where + "." + rpm_key)

        if "limits_deg" in entry and "limits_rad" in entry:
            raise ConfigError("%s: supply only limits_deg or limits_rad, not both" % joint_where)
        if "limits_deg" in entry:
            limits_rad = _parse_degree_limits(entry["limits_deg"], joint_where + ".limits_deg")
        else:
            limits_rad = _parse_limits(_field(entry, "limits_rad", joint_where, None),
                                       joint_where + ".limits_rad")

        joints[name] = JointConfig(name=name, board=board_name, channel=channel, role=role,
                                   gear_ratio=gear_ratio, inverted=inverted,
                                   expected_motor_rpm=expected_motor_rpm, limits_rad=limits_rad)
    return joints


# --------------------------------------------------------------------------
# Public entry points
# --------------------------------------------------------------------------


def parse_config(data):
    # type: (Dict[str, Any]) -> DriverConfig
    """Validate an already parsed YAML mapping and return a DriverConfig.

    `data` is what yaml.safe_load produced, so this function is testable without
    touching the filesystem or pyyaml at all.
    """
    if not isinstance(data, dict):
        detail = ("got a list; boards and joints must be mappings keyed by name, not lists"
                  if isinstance(data, list) else "got %s" % _type_name(data))
        raise ConfigError("configuration must be a mapping with the sections %s, %s"
                          % ("/".join(REQUIRED_SECTIONS), detail))
    missing = [name for name in REQUIRED_SECTIONS if name not in data]
    if missing:
        raise ConfigError("configuration is missing required section(s): %s (all of %s are "
                          "required)" % (", ".join(missing), ", ".join(REQUIRED_SECTIONS)))

    policies = _parse_driver(data[SECTION_DRIVER])
    boards = _parse_boards(data[SECTION_BOARDS])
    joints = _parse_joints(data[SECTION_JOINTS], boards)
    return DriverConfig(allow_motion=policies[0],
                        allow_unreferenced_relative=policies[1],
                        polling_hz=policies[2],
                        board_motion_policy=policies[3],
                        speed_policy=policies[4],
                        boards=boards,
                        joints=joints)


def _parse_yaml_text(text, source):
    # type: (str, str) -> Any
    """Parse YAML text with a SafeLoader that also rejects duplicate keys.

    pyyaml is imported here, not at module scope, so `import config` still works
    on a machine without it and the missing dependency is reported as a ConfigError
    that names the file being loaded.
    """
    try:
        import yaml
    except ImportError:
        raise ConfigError("%s needs pyyaml, which is not installed for this interpreter "
                          "(the remote target ships pyyaml 5.3.1); install it with "
                          "`python3 -m pip install pyyaml`" % source)

    class _NoDuplicateKeys(yaml.SafeLoader):
        """SafeLoader plus a duplicate key check, which pyyaml does not do itself."""

    def _construct_mapping(loader, node, deep=False):
        pairs = loader.construct_pairs(node, deep=deep)
        seen = set()
        for key, _ in pairs:
            if key in seen:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping", node.start_mark,
                    "found duplicate key %r; a repeated key silently overrides the earlier "
                    "value, so it is refused" % (key,), node.start_mark)
            seen.add(key)
        return dict(pairs)

    _NoDuplicateKeys.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
                                     _construct_mapping)
    try:
        return yaml.load(text, Loader=_NoDuplicateKeys)
    except yaml.YAMLError as exc:
        raise ConfigError("%s is not valid YAML: %s" % (source, exc))


def load_config_from_text(text):
    # type: (str) -> DriverConfig
    """Parse and validate a configuration given as a string (used by tests)."""
    return parse_config(_parse_yaml_text(text, "configuration text"))


def load_config(path):
    # type: (Any) -> DriverConfig
    """Read, parse and validate one YAML file. Every failure names the path."""
    try:
        text_path = os.fspath(path)
    except TypeError:
        raise ConfigError("config path must be a string or os.PathLike, got %s"
                          % _type_name(path))
    if not os.path.exists(text_path):
        raise ConfigError("config file not found: %s" % text_path)
    if not os.path.isfile(text_path):
        raise ConfigError("config path is not a regular file: %s" % text_path)
    try:
        with open(text_path, "r", encoding="utf-8") as handle:
            text = handle.read()
    except (OSError, UnicodeDecodeError) as exc:
        raise ConfigError("cannot read config file %s: %s" % (text_path, exc))

    data = _parse_yaml_text(text, text_path)
    try:
        return parse_config(data)
    except ConfigError as exc:
        # Prefixing the path keeps `ros2 launch` logs actionable: the operator sees
        # which of several config files is wrong, not only which field.
        raise ConfigError("%s: %s" % (text_path, exc))


def summarise(config):
    # type: (DriverConfig) -> str
    """One human readable line for the node's startup log.

    Reports what the driver is allowed to do (allow_motion) and how it is wired
    (transports), plus how many joints still have unconfirmed mechanical limits,
    because that count is what keeps the node read-only.
    """
    transports = sorted(set(board.transport for board in config.boards.values()))
    unconfirmed = sum(1 for joint in config.joints.values() if joint.limits_rad is None)
    return ("%d board(s) [%s], %d joint(s), allow_motion=%s, allow_unreferenced_relative=%s, "
            "polling_hz=%.1f, board_motion_policy=%s, speed_policy=%s, "
            "limits unconfirmed on %d joint(s)"
            % (len(config.boards), ",".join(transports) if transports else "none",
               len(config.joints), config.allow_motion, config.allow_unreferenced_relative,
               config.polling_hz, config.board_motion_policy, config.speed_policy, unconfirmed))


__all__ = [
    "ConfigError",
    "JointConfig",
    "BoardConfig",
    "DriverConfig",
    "parse_config",
    "load_config",
    "load_config_from_text",
    "summarise",
]
