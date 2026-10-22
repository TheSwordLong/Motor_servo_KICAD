"""Bring-up tests: launch arguments, safety defaults, and the shipped robot.yaml.

These need the installed ament index (they resolve FindPackageShare), so they run
under colcon test rather than from a bare checkout.
"""

import os

import pytest

launch = pytest.importorskip("launch")
from launch import LaunchContext                                          # noqa: E402
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable   # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from launch_ros.actions import Node                                        # noqa: E402

from ament_index_python.packages import get_package_share_directory         # noqa: E402


def load_description():
    """Return (entities, context) for the installed bring-up launch file."""
    share = get_package_share_directory("robot_motor_bringup")
    path = os.path.join(share, "launch", "bringup.launch.py")
    assert os.path.exists(path), "installed launch file missing: %s" % path
    source = PythonLaunchDescriptionSource(path)
    context = LaunchContext()
    description = source.get_launch_description(context)
    return list(description.entities), context


def perform(value, context):
    """Resolve a launch value to text.

    Humble returns LIST-of-substitution from DeclareLaunchArgument.default_value and
    from SetEnvironmentVariable.name/value, a bare string from Node package, and a
    single Substitution elsewhere, so all three shapes are handled here.
    """
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if isinstance(value, (list, tuple)):
        return "".join(perform(item, context) for item in value)
    if hasattr(value, "perform"):
        return value.perform(context)
    return str(value)


def node_package(node, context):
    """Node exposes its package as a public property on some distros and only as a
    name-mangled attribute on others (Humble), so try both."""
    for attribute in ("package", "_Node__package"):
        value = getattr(node, attribute, None)
        if value is not None:
            return perform(value, context)
    return ""


def test_launch_declares_the_arguments_the_plan_requires():
    """Section 8C of the execution plan invokes transport, allow_motion and
    config_file; those must exist as launch arguments."""
    entities, context = load_description()
    declared = {entity.name: entity for entity in entities
                if isinstance(entity, DeclareLaunchArgument)}
    for required in ("transport", "allow_motion", "config_file"):
        assert required in declared, "missing launch argument %r (have %s)" % (
            required, sorted(declared))
    for extra in ("maintenance", "polling_hz", "publish_joint_states", "require_uid",
                  "ros_domain_id", "start_driver", "log_level",
                  "allow_unreferenced_relative"):
        assert extra in declared, "missing launch argument %r" % extra


def test_unreferenced_relative_defaults_to_false():
    """Enabling relative motion without a zero reference must be an explicit act."""
    entities, context = load_description()
    defaults = {entity.name: perform(entity.default_value, context)
                for entity in entities if isinstance(entity, DeclareLaunchArgument)}
    assert defaults["allow_unreferenced_relative"] == "false"


def test_node_receives_the_policy_parameters():
    """Every policy switch must actually be forwarded to the node; an argument that is
    declared but never passed silently does nothing (that bug shipped once already).

    launch_ros normalises the parameters mapping into ((name subs), (value subs))
    tuples, so both the plain dict form and the normalised form are handled.
    """
    entities, context = load_description()
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    assert nodes
    node = nodes[0]
    raw = getattr(node, "_Node__parameters", None) or getattr(node, "parameters", None)
    assert raw, "could not introspect the declared node parameters"
    forwarded = set()
    for item in raw:
        if isinstance(item, dict):
            # Normalised form: {(name substitutions,): (value substitutions,)}
            for key in item.keys():
                try:
                    forwarded.add(perform(key, context))
                except Exception:      # pragma: no cover - unexpected shape
                    continue
            continue
        try:
            forwarded.add(perform(item[0], context))
        except Exception:      # pragma: no cover - unexpected normalised shape
            continue
    for required in ("config_file", "transport", "allow_motion",
                     "allow_unreferenced_relative", "maintenance", "polling_hz",
                     "publish_joint_states", "require_uid", "commissioning",
                     "max_commissioning_step_deg"):
        assert required in forwarded, "launch does not forward %r to the node (got %s)" % (
            required, sorted(str(name) for name in forwarded))


def test_motion_is_off_by_default():
    entities, context = load_description()
    defaults = {entity.name: perform(entity.default_value, context)
                for entity in entities if isinstance(entity, DeclareLaunchArgument)}
    assert defaults["allow_motion"] == "false", "bring-up must default to read-only"
    assert defaults["maintenance"] == "false"
    assert defaults["commissioning"] == "false", \
        "unbounded motion must never be the default"
    assert defaults["publish_joint_states"] == "false"
    assert defaults["require_uid"] == "true"
    assert defaults["start_driver"] == "true"


def test_config_file_defaults_to_the_installed_robot_yaml():
    entities, context = load_description()
    defaults = {entity.name: perform(entity.default_value, context)
                for entity in entities if isinstance(entity, DeclareLaunchArgument)}
    assert defaults["config_file"].endswith(os.path.join("robot_motor_bringup", "config",
                                                         "robot.yaml"))
    assert os.path.exists(defaults["config_file"])


def test_dedicated_ros_domain_is_exported():
    """The target host runs a live control stack on domain 0; this driver must not
    quietly join it.

    The declaration actions are visited first so the launch configurations hold
    their defaults; otherwise resolving the value would just echo whatever the test
    happened to set, proving nothing.
    """
    entities, context = load_description()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            try:
                list(entity.visit(context))
            except Exception:      # pragma: no cover - distro variance
                pass
    exports = [entity for entity in entities if isinstance(entity, SetEnvironmentVariable)]
    assert exports, "bring-up must set ROS_DOMAIN_ID"
    named = {perform(entity.name, context): perform(entity.value, context) for entity in exports}
    assert "ROS_DOMAIN_ID" in named
    assert named["ROS_DOMAIN_ID"] == "71"


def test_driver_node_is_declared():
    entities, context = load_description()
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    assert nodes, "bring-up must declare the driver node"
    packages = [node_package(node, context) for node in nodes]
    assert "robot_motor_driver" in packages


def test_shipped_robot_yaml_matches_the_audited_board():
    """The default configuration must describe the board that was actually probed:
    UID, stable serial path, and the four joints with their gear ratios and roles."""
    from robot_motor_driver import protocol as proto
    from robot_motor_driver.config import load_config

    share = get_package_share_directory("robot_motor_bringup")
    config = load_config(os.path.join(share, "config", "robot.yaml"))

    assert config.allow_motion is False
    assert config.allow_unreferenced_relative is False
    assert config.polling_hz == pytest.approx(5.0)
    assert config.board_motion_policy == "serial"
    assert config.speed_policy == "keep_requested"

    assert len(config.boards) == 1
    board = config.boards["main_board"]
    assert board.uid == "C23B378115B092083830364A"
    assert board.transport == "modbus_rtu"
    assert board.baudrate == 115200
    assert board.address == 1
    # A stable by-path link, never a bare ttyUSBn: this port re-enumerated between
    # sessions on the same physical socket.
    assert board.port.startswith("/dev/serial/by-path/")
    assert "platform-fc800000.usb-usb-0:1.2:1.0-port0" in board.port

    expected = {
        "head_joint": (1, int(proto.Role.HEAD), 516),
        "left_upper_arm_joint": (0, int(proto.Role.LEFT_UPPER), 516),
        "left_forearm_joint": (2, int(proto.Role.LEFT_FOREARM), 290),
        "left_wrist_joint": (3, int(proto.Role.LEFT_PALM), 82),
    }
    assert set(config.joints) == set(expected)
    for name, (channel, role, gear) in expected.items():
        joint = config.joint(name)
        assert joint.board == "main_board"
        assert joint.channel == channel, name
        assert joint.role == role, name
        assert joint.gear_ratio == gear, name
        assert joint.inverted == 0, name
        # Head updated to 400 on 2026-10-08; arms remain 8000. EXPECTATION the
        # driver compares against, never a speed it commands: MOVE carries only an
        # angle and the firmware reads rpm from its own stored configuration.
        assert joint.expected_motor_rpm == (400 if name == "head_joint" else 8000), name
        assert joint.default_motor_rpm == joint.expected_motor_rpm, name
        assert joint.limits_rad is None, "%s limits are not confirmed yet" % name


def test_shipped_robot_yaml_channels_are_unique_per_board():
    from robot_motor_driver.config import load_config

    share = get_package_share_directory("robot_motor_bringup")
    config = load_config(os.path.join(share, "config", "robot.yaml"))
    channels = [joint.channel for joint in config.joints.values()]
    assert sorted(channels) == [0, 1, 2, 3]


# --------------------------------------------------------------------------
# Multi-board: board selection and namespacing
# --------------------------------------------------------------------------


def node_namespace(node):
    """Raw namespace substitution of a Node action.

    launch_ros stores it as a name-mangled private attribute on Humble, the same
    situation node_package() handles for the package name, so try the known
    spellings in turn.
    """
    for attribute in ("_Node__node_namespace", "node_namespace", "namespace"):
        value = getattr(node, attribute, None)
        if value is not None:
            return value
    return None


def forwarded_parameter_names(node, context):
    """Names of the parameters a Node action forwards, in both the plain dict and
    the normalised ((name subs), (value subs)) shapes (see
    test_node_receives_the_policy_parameters for why both exist)."""
    raw = getattr(node, "_Node__parameters", None) or getattr(node, "parameters", None)
    assert raw, "could not introspect the declared node parameters"
    forwarded = set()
    for item in raw:
        if isinstance(item, dict):
            for key in item.keys():
                try:
                    forwarded.add(perform(key, context))
                except Exception:      # pragma: no cover - unexpected shape
                    continue
            continue
        try:
            forwarded.add(perform(item[0], context))
        except Exception:      # pragma: no cover - unexpected normalised shape
            continue
    return forwarded


def load_dual_description():
    """(entities, context) for the installed dual_board.launch.py.

    Loaded exactly like bringup.launch.py: robot_motor_bringup installs its whole
    launch/ directory, so the file resolves from the ament index under colcon
    test. The skip (rather than a failure) covers a workspace installed before
    dual_board.launch.py was added.
    """
    share = get_package_share_directory("robot_motor_bringup")
    path = os.path.join(share, "launch", "dual_board.launch.py")
    if not os.path.exists(path):
        pytest.skip("dual_board.launch.py is missing from the installed share directory "
                    "(%s); re-install robot_motor_bringup built from this checkout" % path)
    source = PythonLaunchDescriptionSource(path)
    context = LaunchContext()
    description = source.get_launch_description(context)
    return list(description.entities), context


def test_board_name_and_board_namespace_arguments_default_to_empty():
    """Empty board_name means "the only board" to the node, and empty
    board_namespace keeps the historical /robot/... names, so a single-board
    bring-up is unchanged by the multi-board work."""
    entities, context = load_description()
    declared = {entity.name: entity for entity in entities
                if isinstance(entity, DeclareLaunchArgument)}
    for required in ("board_name", "board_namespace"):
        assert required in declared, "missing launch argument %r (have %s)" % (
            required, sorted(declared))
    defaults = {entity.name: perform(entity.default_value, context)
                for entity in entities if isinstance(entity, DeclareLaunchArgument)}
    assert defaults["board_name"] == "", defaults["board_name"]
    assert defaults["board_namespace"] == "", defaults["board_namespace"]


def test_board_name_is_a_parameter_and_board_namespace_is_the_node_namespace():
    """board_name must reach the node as a PARAMETER (that is how it selects its
    board), while board_namespace must become the Node's namespace, NOT a
    parameter: the node declares no board_namespace parameter, so forwarding it
    would be an undeclared override, and the relative interface names would keep
    colliding between two boards in one domain."""
    entities, context = load_description()
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    assert len(nodes) == 1, "bringup must declare exactly one driver node"
    node = nodes[0]

    forwarded = forwarded_parameter_names(node, context)
    assert "board_name" in forwarded, "launch does not forward board_name (got %s)" % (
        sorted(str(name) for name in forwarded))
    assert "board_namespace" not in forwarded, \
        "board_namespace is the Node's namespace; the node has no such parameter"

    raw_namespace = node_namespace(node)
    assert raw_namespace is not None, "could not introspect the node namespace"
    # Prove the substitution really is the board_namespace argument by giving that
    # argument a probe value and watching it come back out; comparing against the
    # '' default alone would also pass for a hard-coded empty namespace.
    context.launch_configurations["board_namespace"] = "probe_ns"
    assert perform(raw_namespace, context) == "probe_ns"


def test_dual_board_declares_per_board_arguments_with_safe_defaults():
    """Every per-board knob must exist for BOTH sides, the second board must be
    opt-in (board_b_start false), and no board may move by default."""
    entities, context = load_dual_description()
    declared = {entity.name: entity for entity in entities
                if isinstance(entity, DeclareLaunchArgument)}
    for side in ("a", "b"):
        for key in ("name", "namespace", "start", "allow_motion",
                    "allow_unreferenced_relative", "maintenance", "commissioning",
                    "max_commissioning_step_deg"):
            argument = "board_%s_%s" % (side, key)
            assert argument in declared, "missing launch argument %r (have %s)" % (
                argument, sorted(declared))
    defaults = {entity.name: perform(entity.default_value, context)
                for entity in entities if isinstance(entity, DeclareLaunchArgument)}
    assert defaults["board_a_start"] == "true"
    assert defaults["board_b_start"] == "false", \
        "the second board must be opt-in: this file may not start driving a board " \
        "nobody asked about"
    assert defaults["board_a_allow_motion"] == "false"
    assert defaults["board_b_allow_motion"] == "false"
    assert defaults["board_a_allow_unreferenced_relative"] == "false"
    assert defaults["board_b_allow_unreferenced_relative"] == "false"
    assert defaults["board_a_maintenance"] == "false"
    assert defaults["board_b_maintenance"] == "false"
    assert defaults["board_a_commissioning"] == "false"
    assert defaults["board_b_commissioning"] == "false"
    assert defaults["board_a_namespace"] == "board_a"
    assert defaults["board_b_namespace"] == "board_b"
    assert defaults["board_a_name"] == "main_board"
    assert defaults["board_b_name"] == "right_board"


def test_dual_board_declares_exactly_two_driver_nodes_under_different_namespaces():
    """One Node per board, each with its OWN namespace substitution: a shared
    namespace would put both boards' services on the same names, which is exactly
    the collision the relative naming exists to prevent."""
    entities, context = load_dual_description()
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    assert len(nodes) == 2, "dual_board must declare one node per board, got %d" % len(nodes)
    for node in nodes:
        assert node_package(node, context) == "robot_motor_driver"

    raw_namespaces = []
    for node in nodes:
        raw = node_namespace(node)
        assert raw is not None, "could not introspect a node's namespace"
        raw_namespaces.append(raw)
    # Probe values prove each node reads its OWN argument, without depending on the
    # order the two Node actions appear in.
    context.launch_configurations["board_a_namespace"] = "probe_ns_a"
    context.launch_configurations["board_b_namespace"] = "probe_ns_b"
    resolved = sorted(perform(raw, context) for raw in raw_namespaces)
    assert resolved == ["probe_ns_a", "probe_ns_b"], \
        "the two nodes do not use the two distinct namespace arguments: %s" % resolved


def test_dual_board_forwards_board_name_to_each_node_but_never_a_namespace_parameter():
    """Each node must select its board through the board_name PARAMETER, and no
    namespace may be forwarded as a parameter (the node declares none, and
    namespaces come from the Node action itself)."""
    entities, context = load_dual_description()
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    assert len(nodes) == 2
    for node in nodes:
        forwarded = forwarded_parameter_names(node, context)
        assert "board_name" in forwarded, "a node does not forward board_name (got %s)" % (
            sorted(str(name) for name in forwarded))
        for forbidden in ("board_namespace", "board_a_namespace", "board_b_namespace",
                          "namespace"):
            assert forbidden not in forwarded, \
                "%r was forwarded as a parameter; namespaces belong to the Node action" % forbidden
        for required in ("config_file", "transport", "allow_motion",
                         "allow_unreferenced_relative", "maintenance", "commissioning",
                         "max_commissioning_step_deg", "polling_hz", "require_uid",
                         "publish_joint_states"):
            assert required in forwarded, "a node does not forward %r (got %s)" % (
                required, sorted(str(name) for name in forwarded))
