"""Node level tests against the simulated board.

These need rclpy and the generated interfaces, so they skip cleanly when either is
missing (for example when the offline protocol suite runs on a machine without
ROS). Nothing here opens a real serial port: transport is always "fake".

The point of these tests is the safety contract, not the plumbing:
  * starting the node puts nothing on the bus beyond reads;
  * read-only refuses motion goals AND the stop service, with a message that says
    why;
  * goal validation rejects NaN, unknown joints, bad rpm, bad timeouts, oversized
    angles and absolute moves without a zero;
  * a goal that quantises to zero steps returns NO_OP instead of waiting for a DONE
    that would mean nothing;
  * shutting down does not sneak a motion or a self test out of the door.
"""

import math
import os

# Set BEFORE rclpy is imported/initialised: these tests create real nodes and
# publishers, and the target host runs a live control stack on domain 0
# (controller_manager, joint_state_broadcaster, several robot_state_publisher).
# Test traffic must not join that graph, and the discovery load from it must not
# destabilise the tests either.
os.environ.setdefault("ROS_DOMAIN_ID", "71")

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import GoalResponse                                    # noqa: E402
from robot_motor_interfaces.action import MoveJoint, RunSelfTest          # noqa: E402
from robot_motor_interfaces.msg import MotorStates                        # noqa: E402
from robot_motor_interfaces.srv import Stop                               # noqa: E402

from robot_motor_driver import protocol as proto                          # noqa: E402
from robot_motor_driver.config import ConfigError, load_config_from_text  # noqa: E402
from robot_motor_driver.motor_node import (RESULT_GOAL_INVALID, RESULT_LIMIT,
                                           RESULT_LIMITS_UNCONFIRMED, RESULT_NO_OP,
                                           RESULT_RPM_MISMATCH, RESULT_VALID_REQUIRED,
                                           RobotMotorNode)
from robot_motor_driver.scheduler import TaskRequest                       # noqa: E402

CONFIG_YAML = """
driver:
  allow_motion: false
  allow_unreferenced_relative: false
  polling_hz: 20.0
  board_motion_policy: serial
  speed_policy: keep_requested
boards:
  main_board:
    uid: "C23B378115B092083830364A"
    transport: fake
    port: ""
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
  left_wrist_joint:
    board: main_board
    channel: 4
    role: 6
    gear_ratio: 82
    inverted: 0
    default_motor_rpm: 400
    limits_rad: [-1.5707963267948966, 1.5707963267948966]
"""


@pytest.fixture()
def context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def make_node(context, allow_motion=False, maintenance=False,
              allow_unreferenced_relative=False, publish_joint_states=False,
              commissioning=False, max_commissioning_step_deg=10.0):
    """Build a node with its parameters applied BEFORE the constructor reads them."""
    from rclpy.parameter import Parameter
    config = load_config_from_text(CONFIG_YAML)
    overrides = [
        Parameter("transport", Parameter.Type.STRING, "fake"),
        Parameter("allow_motion", Parameter.Type.BOOL, bool(allow_motion)),
        Parameter("maintenance", Parameter.Type.BOOL, bool(maintenance)),
        Parameter("allow_unreferenced_relative", Parameter.Type.BOOL,
                  bool(allow_unreferenced_relative)),
        Parameter("publish_joint_states", Parameter.Type.BOOL, bool(publish_joint_states)),
        Parameter("polling_hz", Parameter.Type.DOUBLE, 20.0),
        Parameter("commissioning", Parameter.Type.BOOL, bool(commissioning)),
        Parameter("max_commissioning_step_deg", Parameter.Type.DOUBLE,
                  float(max_commissioning_step_deg)),
    ]
    return RobotMotorNode(config=config, parameter_overrides=overrides)


def move_goal(joint="head_joint", mode=0, angle_rad=0.1, rpm=0, timeout=10.0):
    goal = MoveJoint.Goal()
    goal.joint_name = joint
    goal.mode = mode
    goal.angle_rad = float(angle_rad)
    goal.motor_rpm = int(rpm)
    goal.timeout_sec = float(timeout)
    return goal


# --------------------------------------------------------------------------
# Startup and shutdown hygiene
# --------------------------------------------------------------------------


def test_startup_only_reads_the_board(context):
    node = make_node(context)
    try:
        board = node._fake_board
        assert board.commands_received == 0, "startup must not send a single command frame"
        assert board.flash_writes == 0, "startup must not write flash"
        assert node.session.identified is True
        assert node.session.uid == "C23B378115B092083830364A"
        assert node.session.read_only is True
    finally:
        node.destroy_node()


def test_shutdown_does_not_send_motion_or_selftest(context):
    node = make_node(context)
    board = node._fake_board
    node.destroy_node()
    assert board.commands_received == 0


def test_shutdown_stop_is_suppressed_in_read_only_mode(context):
    """Best-effort stop is only attempted when writes are allowed at all."""
    node = make_node(context, allow_motion=False)
    board = node._fake_board
    node.destroy_node()
    assert board.commands_received == 0


def test_publishing_states_does_not_write(context):
    import time

    node = make_node(context)
    try:
        # Wait for the worker's first poll; the snapshot starts empty by design.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not node.worker.snapshot.uid:
            time.sleep(0.05)
        for _ in range(3):
            node._publish_states()
        assert node._fake_board.commands_received == 0
        snapshot = node.worker.snapshot
        assert snapshot.uid == node.session.uid == "C23B378115B092083830364A"
    finally:
        node.destroy_node()


def test_states_topic_carries_uid_freshness_and_read_only(context):
    """End-to-end check that the published snapshot is honest about identity,
    read-only mode and per-channel configuration."""
    import time

    from rclpy.executors import MultiThreadedExecutor
    node = make_node(context)
    probe = rclpy.create_node("states_probe")
    received = []
    try:
        probe.create_subscription(MotorStates, "/robot/motor_states",
                                  lambda message: received.append(message), 10)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        deadline = time.monotonic() + 15.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        executor.shutdown()
        assert received, "no /robot/motor_states message arrived"
        message = received[-1]
        assert message.uid == "C23B378115B092083830364A"
        assert message.command_protocol == proto.COMMAND_PROTOCOL_REVISION
        assert message.identity_ok is True
        assert message.read_only is True
        assert message.unknown_command_state is False
        assert message.communication_ok is True
        assert len(message.channels) == proto.CHANNEL_COUNT
        head = message.channels[1]          # head_joint is YAML channel 2 -> protocol 1
        assert head.channel == 1
        assert head.gear_ratio == 516
        assert head.role == int(proto.Role.HEAD)
        assert head.pulses_per_output_rev == 516 * 6
        assert message.age_sec >= 0.0
    finally:
        probe.destroy_node()
        node.destroy_node()


# --------------------------------------------------------------------------
# Goal gating
# --------------------------------------------------------------------------


def test_motion_goals_are_rejected_when_allow_motion_is_false(context):
    node = make_node(context, allow_motion=False)
    try:
        assert node._on_move_goal(move_goal()) == GoalResponse.REJECT
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_motion_goals_are_accepted_when_allow_motion_is_true(context):
    node = make_node(context, allow_motion=True)
    try:
        assert node._on_move_goal(move_goal()) == GoalResponse.ACCEPT
        # Accepting a goal must not itself move anything.
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_goals_are_rejected_when_identity_is_unconfirmed(context):
    node = make_node(context, allow_motion=True)
    try:
        node.identity_error = "board UID does not match"
        assert node._on_move_goal(move_goal()) == GoalResponse.REJECT
    finally:
        node.destroy_node()


def test_goals_are_rejected_while_the_outcome_is_unknown(context):
    node = make_node(context, allow_motion=True)
    try:
        node.session.mark_unknown("test")
        assert node._on_move_goal(move_goal()) == GoalResponse.REJECT
        node.session.acknowledge_unknown()
        assert node._on_move_goal(move_goal()) == GoalResponse.ACCEPT
    finally:
        node.destroy_node()


def test_cancel_is_always_accepted(context):
    """Cancelling must never be refused: the worker then sends STOP and reports
    honestly whether the board confirmed it."""
    from rclpy.action import CancelResponse
    node = make_node(context, allow_motion=True)
    try:
        assert node._on_move_cancel(object()) == CancelResponse.ACCEPT
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# Goal validation
# --------------------------------------------------------------------------


@pytest.mark.parametrize("angle", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_angle_is_rejected(context, angle):
    node = make_node(context, allow_motion=True)
    try:
        code, message, joint = node._validate_move_goal(move_goal(angle_rad=angle))
        assert code == RESULT_GOAL_INVALID
        assert "finite" in message
    finally:
        node.destroy_node()


def test_unknown_joint_is_rejected(context):
    node = make_node(context, allow_motion=True)
    try:
        code, message, joint = node._validate_move_goal(move_goal(joint="right_wrist_joint"))
        assert code == RESULT_GOAL_INVALID
        assert joint is None
        assert "unknown joint" in message
    finally:
        node.destroy_node()


def test_bad_mode_is_rejected(context):
    node = make_node(context, allow_motion=True)
    try:
        code, _, _ = node._validate_move_goal(move_goal(mode=7))
        assert code == RESULT_GOAL_INVALID
    finally:
        node.destroy_node()


@pytest.mark.parametrize("rpm", [1, 399, 8001, 65535])
def test_rpm_outside_the_firmware_domain_is_rejected(context, rpm):
    node = make_node(context, allow_motion=True)
    try:
        code, message, _ = node._validate_move_goal(move_goal(rpm=rpm))
        assert code == RESULT_GOAL_INVALID
        assert "motor_rpm" in message
    finally:
        node.destroy_node()


def test_rpm_zero_means_use_the_board_value(context):
    """motor_rpm is an assertion about the board's stored speed, not a command. Zero
    means "whatever the board has", so it can never disagree."""
    node = make_node(context, allow_motion=True)
    try:
        code, _, joint = node._validate_move_goal(move_goal(rpm=0))
        assert code == 0
        # The YAML value is what the driver EXPECTS the board to hold; the old
        # attribute name still works so existing callers do not break.
        assert joint.expected_motor_rpm == 400
        assert joint.default_motor_rpm == joint.expected_motor_rpm
    finally:
        node.destroy_node()


@pytest.mark.parametrize("timeout", [0.0, 0.1, -5.0, 1e6])
def test_timeout_outside_the_driver_window_is_rejected(context, timeout):
    node = make_node(context, allow_motion=True)
    try:
        code, message, _ = node._validate_move_goal(move_goal(timeout=timeout))
        assert code == RESULT_GOAL_INVALID
        assert "timeout_sec" in message
    finally:
        node.destroy_node()


def test_angle_beyond_one_turn_is_rejected_not_split(context):
    node = make_node(context, allow_motion=True)
    try:
        code, message, _ = node._validate_move_goal(move_goal(angle_rad=math.pi * 2.5))
        assert code == RESULT_LIMIT
        assert "360" in message
    finally:
        node.destroy_node()


def test_absolute_goal_outside_configured_limits_is_rejected(context):
    node = make_node(context, allow_motion=True)
    try:
        code, message, _ = node._validate_move_goal(
            move_goal(joint="left_wrist_joint", mode=1, angle_rad=3.0))
        assert code == RESULT_LIMIT
        assert "limits" in message
    finally:
        node.destroy_node()


def test_joint_without_limits_does_not_invent_them(context):
    """limits_rad: null means unconfirmed. The static goal check still passes, but the
    motion itself is refused: without a limit the driver cannot say where the joint
    would end up, and inventing one would be worse than refusing."""
    node = make_node(context, allow_motion=True)
    try:
        joint = node.config.joint("head_joint")
        assert joint.limits_rad is None
        code, _, _ = node._validate_move_goal(move_goal(joint="head_joint", mode=1,
                                                        angle_rad=3.0))
        assert code == 0
        quiet_fake_board(node)
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0, angle_rad=0.1,
                                          timeout=30.0))
        result = node._on_move(handle)
        assert result.error_code == RESULT_LIMITS_UNCONFIRMED, result.message
        assert "commissioning" in result.message
        assert handle.outcome == "abort"
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_commissioning_allows_unbounded_motion_but_caps_each_command(context):
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True, max_commissioning_step_deg=10.0)
    try:
        assert node.commissioning is True
        assert node.soft_limits_enforced is False, "head_joint has limits_rad null"
        quiet_fake_board(node)
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0,
                                          angle_rad=math.radians(5.0), timeout=30.0))
        result = node._on_move(handle)
        assert result.error_code == 0, result.message
        assert result.motion_completed is True

        big = FakeGoalHandle(move_goal(joint="head_joint", mode=0,
                                       angle_rad=math.radians(45.0), timeout=30.0))
        refused = node._on_move(big)
        assert refused.error_code == RESULT_LIMIT
        assert "commissioning mode caps" in refused.message
        assert big.outcome == "abort"
    finally:
        node.destroy_node()


def test_move_refuses_an_rpm_the_board_does_not_have(context):
    """The goal asked for 400 rpm while the board is configured for 8000. Silently
    running at the board's speed would let a caller believe it had set the speed."""
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True)
    try:
        quiet_fake_board(node)
        board_rpm = node._fake_board.channels[1].rpm
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0, angle_rad=0.1,
                                          rpm=400, timeout=30.0))
        result = node._on_move(handle)
        assert result.error_code == RESULT_RPM_MISMATCH, result.message
        assert str(board_rpm) in result.message
        assert "400" in result.message
        assert "apply_config" in result.message
        assert handle.outcome == "abort"
        assert node._fake_board.commands_received == 0

        # The same goal with the board's own rpm goes through.
        matching = FakeGoalHandle(move_goal(joint="head_joint", mode=0, angle_rad=0.1,
                                            rpm=board_rpm, timeout=30.0))
        accepted = node._on_move(matching)
        assert accepted.error_code == 0, accepted.message
        assert accepted.motion_completed is True
    finally:
        node.destroy_node()


def test_states_topic_reports_the_soft_limit_posture(context):
    """A consumer must be able to see that no soft limits are protecting this robot."""
    node = make_node(context, commissioning=True)
    try:
        message = node._states_message(node.worker.snapshot)
        assert message.commissioning is True
        assert message.soft_limits_enforced is False
    finally:
        node.destroy_node()

    plain = make_node(context)
    try:
        message = plain._states_message(plain.worker.snapshot)
        assert message.commissioning is False
        assert message.soft_limits_enforced is False
    finally:
        plain.destroy_node()


# --------------------------------------------------------------------------
# Read-only service behaviour
# --------------------------------------------------------------------------


def test_stop_service_refuses_in_read_only_mode_and_says_why(context):
    node = make_node(context, allow_motion=False)
    try:
        request = Stop.Request()
        request.joint_name = "head_joint"
        response = node._on_stop(request, Stop.Response())
        assert response.accepted is False
        assert response.stop_confirmed is False
        assert "hardware e-stop" in response.message
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_stop_service_with_unknown_joint_is_rejected_before_anything_else(context):
    node = make_node(context, allow_motion=True)
    try:
        request = Stop.Request()
        request.joint_name = "no_such_joint"
        response = node._on_stop(request, Stop.Response())
        assert response.accepted is False
        assert "unknown joint" in response.message
    finally:
        node.destroy_node()


def test_maintenance_services_refuse_in_read_only_mode(context):
    node = make_node(context, allow_motion=False, maintenance=False)
    try:
        from robot_motor_interfaces.srv import ClearFault, SetBrake, SetZero
        for service_type, handler, name in (
                (SetZero, node._on_set_zero, "zero"),
                (ClearFault, node._on_clear_fault, "clear"),
                (SetBrake, node._on_set_brake, "brake")):
            request = service_type.Request()
            request.joint_name = "head_joint"
            response = handler(request, service_type.Response())
            assert response.accepted is False, name
            assert "read-only" in response.message, name
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_maintenance_mode_allows_stop_but_not_motion(context):
    node = make_node(context, allow_motion=False, maintenance=True)
    try:
        assert node.session.read_only is False
        assert node.worker.allow_motion is False
        assert node._on_move_goal(move_goal()) == GoalResponse.REJECT
        request = Stop.Request()
        request.joint_name = "head_joint"
        response = node._on_stop(request, Stop.Response())
        assert response.accepted is True
        assert response.stop_confirmed is True
        assert node._fake_board.commands_received >= 1
    finally:
        node.destroy_node()


def test_acknowledge_unknown_service_reports_state(context):
    node = make_node(context, allow_motion=True)
    try:
        from std_srvs.srv import Trigger
        response = node._on_acknowledge_unknown(Trigger.Request(), Trigger.Response())
        assert response.success is True
        assert "no UNKNOWN" in response.message
        node.session.mark_unknown("simulated loss")
        response = node._on_acknowledge_unknown(Trigger.Request(), Trigger.Response())
        assert response.success is True
        assert "simulated loss" in response.message
        assert node.session.unknown_command_state is False
    finally:
        node.destroy_node()


def test_board_identity_service_reports_uid(context):
    node = make_node(context)
    try:
        from std_srvs.srv import Trigger
        node.worker.refresh_now()
        response = node._on_board_identity(Trigger.Request(), Trigger.Response())
        assert response.success is True
        assert "C23B378115B092083830364A" in response.message
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# Quantisation and self test gating
# --------------------------------------------------------------------------


def test_selftest_is_gated_by_allow_motion(context):
    """The self test MOVES the joint, so it is gated exactly like a motion goal and
    is never started automatically."""
    node = make_node(context, allow_motion=False)
    try:
        goal = RunSelfTest.Goal()
        goal.joint_name = "head_joint"
        goal.scope = RunSelfTest.Goal.SINGLE
        goal.direction = 0
        goal.timeout_sec = 10.0
        assert node._on_move_goal(goal) == GoalResponse.REJECT
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_selftest_scope_constants():
    assert RunSelfTest.Goal.SINGLE == 0
    assert RunSelfTest.Goal.ALL == 1


def test_no_op_detection_uses_the_gear_ratio_not_the_argument(context):
    """At gear 82, 0.1 deg quantises to zero steps; the node must not send it."""
    node = make_node(context, allow_motion=True)
    try:
        joint = node.config.joint("left_wrist_joint")
        assert joint.gear_ratio == 82
        assert proto.quantises_to_zero(1, joint.gear_ratio) is True
        assert proto.quantises_to_zero(1, 516) is False
        assert RESULT_NO_OP != 0
    finally:
        node.destroy_node()


def test_yaml_policy_provides_defaults_and_overrides_still_win(context):
    """The YAML driver section must actually drive the node policy. A parameter that
    ignores the file makes the configuration a lie, and a file that ignores the
    override makes launch arguments useless; both directions are checked here."""
    from rclpy.parameter import Parameter

    permissive = CONFIG_YAML.replace("allow_unreferenced_relative: false",
                                     "allow_unreferenced_relative: true")
    config = load_config_from_text(permissive)
    assert config.allow_unreferenced_relative is True

    node = RobotMotorNode(config=config, parameter_overrides=[
        Parameter("transport", Parameter.Type.STRING, "fake")])
    try:
        assert node.get_parameter("allow_unreferenced_relative").value is True
        assert node.get_parameter("allow_motion").value is False
        assert node.get_parameter("polling_hz").value == pytest.approx(20.0)
    finally:
        node.destroy_node()

    stricter = RobotMotorNode(config=config, parameter_overrides=[
        Parameter("transport", Parameter.Type.STRING, "fake"),
        Parameter("allow_unreferenced_relative", Parameter.Type.BOOL, False)])
    try:
        assert stricter.get_parameter("allow_unreferenced_relative").value is False
    finally:
        stricter.destroy_node()


def test_unreferenced_relative_policy_is_enforced(context):
    """With the policy on, a relative move proceeds while position_valid is false;
    with it off the same goal is refused and nothing reaches the bus."""
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True)
    try:
        quiet_fake_board(node)
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0, angle_rad=0.1,
                                          timeout=30.0))
        result = node._on_move(handle)
        assert result.error_code != RESULT_VALID_REQUIRED, result.message
        assert node._fake_board.commands_received >= 1
    finally:
        node.destroy_node()

    strict = make_node(context, allow_motion=True, allow_unreferenced_relative=False,
                       commissioning=True)
    try:
        quiet_fake_board(strict)
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0, angle_rad=0.1,
                                          timeout=30.0))
        result = strict._on_move(handle)
        assert result.error_code == RESULT_VALID_REQUIRED
        assert strict._fake_board.commands_received == 0, \
            "a refused goal must not put a command frame on the bus"
    finally:
        strict.destroy_node()


def test_published_role_and_enabled_come_from_the_board_not_the_yaml(context):
    """A consumer must be able to tell "configured on the board" from "planned in the
    YAML". Substituting the intended role when the board reports 0 made an entirely
    unconfigured right-arm board look configured on /robot/motor_states."""
    node = make_node(context)
    try:
        joint = node.config.joint("head_joint")
        assert joint.role == int(proto.Role.HEAD), "the YAML intends role 1 for this joint"
        board_channel = node._fake_board.channels[joint.channel]
        board_channel.role = 0            # the board says: unassigned
        board_channel.enabled = False
        snapshot = node.worker.refresh_now()
        message = node._channel_message(snapshot, joint.channel)
        assert message.role == 0, "published role must be the board's value, not the YAML's"
        assert message.enabled is False, "published enabled must be the board's value"
        assert message.channel == joint.channel
    finally:
        node.destroy_node()


def test_diagnostics_reports_configuration_divergence(context):
    """The divergence must still be visible somewhere: diagnostics, not the state msg."""
    import time

    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String

    node = make_node(context)
    if getattr(node, "_diagnostics_available", False):
        node.destroy_node()
        pytest.skip("diagnostic_msgs is installed here, so the JSON fallback is not used")
    probe = rclpy.create_node("diag_probe")
    got = []
    try:
        joint = node.config.joint("head_joint")
        node._fake_board.channels[joint.channel].role = 0
        probe.create_subscription(String, "/robot/diagnostics_json",
                                  lambda message: got.append(message.data), 10)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not any("configuration differs" in g for g in got):
            executor.spin_once(timeout_sec=0.1)
        executor.shutdown()
        assert any("configuration differs" in g for g in got), \
            "diagnostics never reported the divergence; last message: %s" % (got[-1:] or "none")
    finally:
        probe.destroy_node()
        node.destroy_node()


def test_valid_required_codes_are_distinct_from_firmware_results(context):
    """Driver codes must never collide with firmware business results 0..6."""
    for code in (RESULT_GOAL_INVALID, RESULT_LIMIT, RESULT_NO_OP, RESULT_VALID_REQUIRED):
        assert code >= 100
    node = make_node(context, allow_motion=True)
    try:
        assert node.session.read_only is False
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# End-to-end execute callbacks
# --------------------------------------------------------------------------


def quiet_fake_board(node):
    """Make the simulated board accept commands.

    The firmware requires 500 ms without any FG edge on ANY channel before it takes
    MOVE/ZERO/CONFIG/SELFTEST (BSP/robot.c:167, 382, 387, 424, 430), and the driver
    correctly reports busy(2) instead of retrying. A freshly built fake board has
    edges "now", so tests that intend to move must settle it first.
    """
    import time as _time

    settled = _time.monotonic() - 2.0
    for channel in node._fake_board.channels:
        channel.last_edge = settled


class FakeGoalHandle(object):
    """The slice of rclpy's ServerGoalHandle that the node touches.

    It deliberately defines NEITHER `is_canceling` NOR `is_cancelling`: a
    source-built Humble was observed without that attribute, and referencing it
    inside the execute callback raised AttributeError, which made the action server
    abort the goal with an EMPTY result *after* the command frame had already gone
    out. On real hardware that is "the joint keeps moving while ROS reports
    failure", so the node must not depend on the attribute existing.
    """

    def __init__(self, request, status=1):
        from types import SimpleNamespace
        self.request = request
        self.is_active = True
        self.status = status
        self.goal_id = SimpleNamespace(uuid=bytes(range(16)))
        self.outcome = None
        self.feedback = []

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.outcome = "succeed"

    def abort(self):
        self.outcome = "abort"

    def canceled(self):
        self.outcome = "canceled"


def test_relative_move_completes_through_the_execute_callback(context):
    """Full path: goal -> validation -> worker -> board -> result, with a goal handle
    that lacks is_canceling.

    head_joint has limits_rad null, so commissioning has to be on for any motion at
    all; that is the point of the switch, not a test convenience.
    """
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True, max_commissioning_step_deg=15.0)
    try:
        quiet_fake_board(node)
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0,
                                          angle_rad=0.1745329252, rpm=0, timeout=30.0))
        result = node._on_move(handle)
        assert handle.outcome == "succeed", (handle.outcome, result.message)
        assert result.motion_completed is True
        assert result.persisted is True
        assert result.error_code == 0
        assert result.message, "an empty message means the execute callback died early"
        # head_joint is gear 516: 10.0 deg -> 86 steps -> exactly 100 deci-deg back.
        assert result.final_angle_rad == pytest.approx(0.1745329252, abs=0.02)
        assert handle.feedback, "no feedback was published during the motion"
        assert handle.feedback[-1].command_sequence >= 1
        assert node._fake_board.commands_received >= 1
    finally:
        node.destroy_node()


def test_execute_callback_never_returns_an_empty_result(context):
    """Regression guard: whatever the outcome, error_code and message must be
    populated. The is_canceling AttributeError produced error_code=0 with an empty
    message and an aborted goal."""
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=False)
    try:
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0,
                                          angle_rad=0.1745329252, timeout=20.0))
        result = node._on_move(handle)
        assert result.message, "empty message with error_code %d" % result.error_code
        assert handle.outcome in ("succeed", "abort", "canceled")
        if not result.motion_completed:
            assert result.error_code != 0 or "refused" in result.message
    finally:
        node.destroy_node()


def test_cancellation_is_detected_through_the_status_fallback(context):
    """With no is_canceling attribute, cancellation must still be seen via the goal
    status, so the driver stops the joint instead of running the motion out."""
    from action_msgs.msg import GoalStatus
    # commissioning is required here only because head_joint has limits_rad null; the
    # step cap is raised so the 6.0 rad goal is not rejected for being too large.
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True, max_commissioning_step_deg=400.0)
    try:
        quiet_fake_board(node)
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0,
                                          angle_rad=6.0, rpm=0, timeout=60.0),
                                status=GoalStatus.STATUS_CANCELING)
        result = node._on_move(handle)
        assert "cancel" in result.message.lower(), result.message
        assert handle.outcome in ("canceled", "abort")
    finally:
        node.destroy_node()


def test_maintenance_zero_reports_a_fresh_valid_bit(context):
    """SetZero must read the board AFTER the command; a stale snapshot used to report
    position_valid=false even though ZERO had succeeded."""
    from robot_motor_interfaces.srv import SetZero
    node = make_node(context, allow_motion=False, maintenance=True)
    try:
        quiet_fake_board(node)
        request = SetZero.Request()
        request.joint_name = "head_joint"
        response = node._on_set_zero(request, SetZero.Response())
        assert response.accepted is True, response.message
        assert response.position_valid is True, response.message
        assert "did NOT report position_valid" not in response.message
        # head_joint is YAML channel 2, i.e. protocol channel 1.
        assert node._fake_board.channels[1].valid is True
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# Configuration transaction semantics
# --------------------------------------------------------------------------


def _config_entry(channel, enabled, role, gear, inverted, rpm):
    from robot_motor_interfaces.msg import ChannelConfig
    entry = ChannelConfig()
    entry.channel = int(channel)
    entry.enabled = bool(enabled)
    entry.role = int(role)
    entry.gear_ratio = int(gear)
    entry.inverted = int(inverted)
    entry.rpm = int(rpm)
    return entry


def _board_config(node):
    return [(c.enabled, c.role, c.gear, c.inverted, c.rpm) for c in node._fake_board.channels]


def test_apply_config_with_an_empty_list_changes_nothing(context):
    """An empty list means "keep what the board has". It must NOT fall back to the YAML:
    silently replacing a board's stored role, gear or rpm with a file default is how a
    working arm gets mis-configured."""
    from robot_motor_interfaces.srv import ApplyConfig
    node = make_node(context, allow_motion=False, maintenance=True)
    try:
        quiet_fake_board(node)
        before = _board_config(node)
        request = ApplyConfig.Request()
        request.commit = True
        response = node._on_apply_config(request, ApplyConfig.Response())
        assert response.accepted is True, response.message
        assert response.changed is False
        assert "CONFIG was NOT sent" in response.message
        assert _board_config(node) == before
        assert node._fake_board.commands_received == 0, "no command frame may be sent"
        # The read-back still reports all four channels.
        assert len(response.applied) == 4 and len(response.staged) == 4
        assert [(c.enabled, c.role, c.gear_ratio, c.inverted, c.rpm)
                for c in response.applied] == before
    finally:
        node.destroy_node()


def test_apply_config_partial_list_leaves_the_other_channels_alone(context):
    from robot_motor_interfaces.srv import ApplyConfig
    node = make_node(context, allow_motion=False, maintenance=True)
    try:
        quiet_fake_board(node)
        before = _board_config(node)
        channel0 = node._fake_board.channels[0]
        request = ApplyConfig.Request()
        request.commit = True
        request.channels = [_config_entry(0, channel0.enabled, channel0.role, channel0.gear,
                                          channel0.inverted, 1500)]
        response = node._on_apply_config(request, ApplyConfig.Response())
        assert response.accepted is True, response.message
        assert response.changed is True
        after = _board_config(node)
        assert after[0] == (before[0][0], before[0][1], before[0][2], before[0][3], 1500)
        assert after[1:] == before[1:], "untouched channels must keep their board values"
    finally:
        node.destroy_node()


def test_apply_config_rejects_a_duplicate_role_on_a_disabled_channel(context):
    """robot_config_valid checks role uniqueness for every non-zero role, enabled or not
    (BSP/robot.c:20). A driver that filters by `enabled` would stage an image the board
    then rejects with R_VALUE, after a flash write was already prepared."""
    from robot_motor_interfaces.srv import ApplyConfig
    node = make_node(context, allow_motion=False, maintenance=True)
    try:
        quiet_fake_board(node)
        before = _board_config(node)
        other_role = node._fake_board.channels[1].role
        request = ApplyConfig.Request()
        request.commit = True
        request.channels = [_config_entry(0, False, other_role, 0, 0, 8000)]
        response = node._on_apply_config(request, ApplyConfig.Response())
        assert response.accepted is False
        assert response.changed is False
        assert "duplicates" in response.message
        assert "robot_config_valid" in response.message
        assert _board_config(node) == before
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_apply_config_rejects_an_rpm_outside_the_firmware_range_on_a_disabled_channel(context):
    from robot_motor_interfaces.srv import ApplyConfig
    node = make_node(context, allow_motion=False, maintenance=True)
    try:
        quiet_fake_board(node)
        request = ApplyConfig.Request()
        request.commit = True
        request.channels = [_config_entry(3, False, 0, 0, 0, 100)]
        response = node._on_apply_config(request, ApplyConfig.Response())
        assert response.accepted is False
        assert "rpm must be 400..8000 even when disabled" in response.message
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()


def test_apply_config_stage_only_commits_nothing(context):
    from robot_motor_interfaces.srv import ApplyConfig
    node = make_node(context, allow_motion=False, maintenance=True)
    try:
        quiet_fake_board(node)
        before = _board_config(node)
        channel0 = node._fake_board.channels[0]
        request = ApplyConfig.Request()
        request.commit = False
        request.channels = [_config_entry(0, channel0.enabled, channel0.role, channel0.gear,
                                          channel0.inverted, 1500)]
        response = node._on_apply_config(request, ApplyConfig.Response())
        assert response.accepted is True, response.message
        assert response.changed is True
        assert "staged only" in response.message
        assert _board_config(node) == before, "stage-only must not change the applied area"
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# Batch self test through the action
# --------------------------------------------------------------------------


def test_selftest_batch_result_carries_every_channel(context):
    from robot_motor_interfaces.action import RunSelfTest
    node = make_node(context, allow_motion=True)
    try:
        node._fake_board.probe_time_scale = 25.0
        quiet_fake_board(node)
        goal = RunSelfTest.Goal()
        goal.scope = RunSelfTest.Goal.ALL
        goal.direction = 0
        goal.timeout_sec = 60.0
        handle = FakeGoalHandle(goal)
        result = node._on_selftest(handle)
        assert handle.outcome == "succeed", (handle.outcome, result.message)
        assert result.batch_completed is True
        assert result.all_passed is True
        assert [c.channel for c in result.channels] == [0, 1, 2, 3]
        assert all(c.executed for c in result.channels)
        assert all(c.state == int(proto.SelftestState.PASSED) for c in result.channels)
        assert all(c.sequence >= 1 for c in result.channels)
        assert "does NOT prove" in result.message
        assert handle.feedback, "a batch must publish feedback while it runs"
        assert handle.feedback[-1].active_channel != 255
    finally:
        node.destroy_node()


def test_selftest_single_scope_still_reports_one_channel(context):
    from robot_motor_interfaces.action import RunSelfTest
    node = make_node(context, allow_motion=True)
    try:
        node._fake_board.probe_time_scale = 25.0
        quiet_fake_board(node)
        goal = RunSelfTest.Goal()
        goal.scope = RunSelfTest.Goal.SINGLE
        goal.joint_name = "head_joint"
        goal.direction = 0
        goal.timeout_sec = 60.0
        handle = FakeGoalHandle(goal)
        result = node._on_selftest(handle)
        assert handle.outcome == "succeed", (handle.outcome, result.message)
        assert result.completed is True
        assert result.passed is True
        assert result.batch_completed is False
        assert [c.channel for c in result.channels] == [1], "head_joint is protocol channel 1"
        assert result.channels[0].state == int(proto.SelftestState.PASSED)
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# Multi-board: one node per board, each under its own namespace
# --------------------------------------------------------------------------
#
# These tests pin the structural guarantees of a two-board deployment: one node
# process drives exactly ONE board (chosen by the board_name parameter), resolves
# only the joints mapped onto that board, and every interface name is relative so
# the node's namespace decides where it lands. Two nodes in one DDS domain must
# not be able to answer each other's service calls, and nothing that happens on
# one board (a move, an UNKNOWN halt, an identity mismatch) may touch the other.

UID_BOARD_A = "C23B378115B092083830364A"
UID_BOARD_B = "C23B37C715B092083830364A"

#: Two simulated boards that mirror the real deployment shape: two TTL adapters,
#: BOTH boards at modbus address 1 (independent ports are independent buses), and
#: empty ports because transport is fake. Each board carries one joint with
#: limits_rad null and one with confirmed limits, so both motion paths are
#: exercised. Roles and channels are unique per board: board A maps the protocol
#: channels {1, 3}, board B maps {0, 2}, and no joint name exists on both boards,
#: which is what makes the cross-board routing checks below meaningful.
TWO_BOARD_YAML = """
driver:
  allow_motion: false
  allow_unreferenced_relative: false
  polling_hz: 20.0
  board_motion_policy: serial
  speed_policy: keep_requested
boards:
  board_a:
    uid: "C23B378115B092083830364A"
    transport: fake
    port: ""
    address: 1
    baudrate: 115200
  board_b:
    uid: "C23B37C715B092083830364A"
    transport: fake
    port: ""
    address: 1
    baudrate: 115200
joints:
  head_joint:
    board: board_a
    channel: 2
    role: 1
    gear_ratio: 516
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  left_wrist_joint:
    board: board_a
    channel: 4
    role: 6
    gear_ratio: 82
    inverted: 0
    default_motor_rpm: 400
    limits_rad: [-1.5707963267948966, 1.5707963267948966]
  right_upper_arm_joint:
    board: board_b
    channel: 1
    role: 3
    gear_ratio: 516
    inverted: 0
    default_motor_rpm: 400
    limits_rad: null
  right_wrist_joint:
    board: board_b
    channel: 3
    role: 7
    gear_ratio: 82
    inverted: 0
    default_motor_rpm: 400
    limits_rad: [-1.5707963267948966, 1.5707963267948966]
"""


def make_node_two(context, board_name, namespace=None, allow_motion=False, maintenance=False,
                  allow_unreferenced_relative=False, publish_joint_states=False,
                  commissioning=False, max_commissioning_step_deg=10.0):
    """make_node() against TWO_BOARD_YAML: selects one board and one namespace.

    board_name and namespace go in BEFORE the constructor reads them, exactly like
    the other overrides: the constructor picks the board and creates the interfaces
    while running, so setting either afterwards would prove nothing.
    """
    from rclpy.parameter import Parameter
    config = load_config_from_text(TWO_BOARD_YAML)
    overrides = [
        Parameter("transport", Parameter.Type.STRING, "fake"),
        Parameter("board_name", Parameter.Type.STRING, board_name),
        Parameter("allow_motion", Parameter.Type.BOOL, bool(allow_motion)),
        Parameter("maintenance", Parameter.Type.BOOL, bool(maintenance)),
        Parameter("allow_unreferenced_relative", Parameter.Type.BOOL,
                  bool(allow_unreferenced_relative)),
        Parameter("publish_joint_states", Parameter.Type.BOOL, bool(publish_joint_states)),
        Parameter("polling_hz", Parameter.Type.DOUBLE, 20.0),
        Parameter("commissioning", Parameter.Type.BOOL, bool(commissioning)),
        Parameter("max_commissioning_step_deg", Parameter.Type.DOUBLE,
                  float(max_commissioning_step_deg)),
    ]
    node = RobotMotorNode(config=config, parameter_overrides=overrides, namespace=namespace)
    # Model a commissioned board for isolation tests. A default FakeBoard carries
    # left-arm roles; silently using those for right-arm tests hid mismatches.
    with node._fake_board._lock:
        for joint in config.joints_for_board(board_name):
            channel = node._fake_board.channels[joint.channel]
            channel.role = joint.role
            channel.gear = joint.gear_ratio
            channel.inverted = joint.inverted
    return node


def absolute(node, relative):
    """Resolve a relative interface name the way rclpy resolves it.

    Deliberately computed HERE and not read from RobotMotorNode._absolute_name: a
    naming test that reuses the implementation's own resolver would restate the
    code under test instead of checking it. This rclpy's Service objects carry
    only the unresolved relative name (srv_name), so the namespace has to come
    from the node.
    """
    ns = (node.get_namespace() or "/").rstrip("/")
    return "%s/%s" % (ns, str(relative).lstrip("/"))


def resolved_service_names(node):
    """Absolute names of every service the node offers.

    node.services is a GENERATOR on this rclpy, so it is drained into a list
    exactly once; graph discovery is not used because it is unreliable in tests.
    """
    return set(absolute(node, service.srv_name) for service in list(node.services))


def test_empty_board_name_with_two_boards_refuses_to_guess(context):
    """With two boards declared, an empty board_name must fail at startup.

    Guessing (e.g. taking the first board) would silently bind motion commands to
    hardware nobody chose; the message has to name the parameter and the boards.
    """
    with pytest.raises(ConfigError) as excinfo:
        make_node_two(context, "")
    text = str(excinfo.value)
    assert "board_name" in text, text
    assert "board_name is empty" in text, text
    assert "board_a" in text and "board_b" in text, \
        "the message should list the declared boards: %s" % text


def test_unknown_board_name_names_the_declared_boards(context):
    """A typo in board_name must be a startup error that shows the way out."""
    with pytest.raises(ConfigError) as excinfo:
        make_node_two(context, "board_c")
    text = str(excinfo.value)
    assert "board_c" in text, text
    assert "not in the configuration" in text, text
    assert "board_a" in text and "board_b" in text, \
        "the message should list the declared boards: %s" % text


def test_board_selection_binds_uid_joints_and_channels_to_one_board(context):
    """board_name='board_b' means: board B's UID, board B's joints, nothing else.

    The joint table is what _resolve_joint searches, so a joint of board A being
    present here would be the cross-board routing bug in its purest form.
    """
    node = make_node_two(context, "board_b", namespace="board_b")
    try:
        assert node.board_name == "board_b"
        assert sorted(node._joints_by_name) == ["right_upper_arm_joint", "right_wrist_joint"]
        assert "head_joint" not in node._joints_by_name, \
            "a board-A joint must not resolve on a board-B node"
        assert "left_wrist_joint" not in node._joints_by_name
        # right_upper_arm_joint is YAML channel 1, right_wrist_joint YAML channel 3,
        # so the PROTOCOL channel map of this node is {0, 2} and nothing else.
        assert sorted(node._joint_by_channel) == [0, 2]
        assert node._joint_by_channel[0].name == "right_upper_arm_joint"
        assert node._joint_by_channel[2].name == "right_wrist_joint"
        # Identification ran in the constructor against board B's own fake board.
        assert node.identity_error == ""
        assert node.session.uid == UID_BOARD_B
        assert node._fake_board.uid == UID_BOARD_B
        assert node.board.uid == UID_BOARD_B
    finally:
        node.destroy_node()


def test_joint_of_another_board_does_not_resolve_here(context):
    """_resolve_joint searches ONLY this node's board; the refusal text names the
    board that does own the joint and says why routing it here would be wrong."""
    node = make_node_two(context, "board_b", namespace="board_b")
    try:
        assert node._resolve_joint("right_upper_arm_joint") is not None
        assert node._resolve_joint("head_joint") is None, \
            "head_joint exists in the configuration but not on THIS board"
        text = node._unknown_joint_text("head_joint")
        assert "board_a" in text, "the message must name the owning board: %s" % text
        assert "board_b" in text, "the message must name this node's board: %s" % text
        assert "connector" in text or "different board" in text, text
        # A joint nobody declared anywhere takes the generic path instead.
        missing = node._unknown_joint_text("no_such_joint")
        assert "unknown joint" in missing, missing
        assert "board_b" in missing, missing
    finally:
        node.destroy_node()


def test_move_goal_naming_another_boards_joint_is_rejected(context):
    """End-to-end guard: a MoveJoint goal for a board-A joint sent to a board-B node
    must abort with GOAL_INVALID and an explanation, and send nothing anywhere."""
    node = make_node_two(context, "board_b", namespace="board_b", allow_motion=True)
    try:
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0, angle_rad=0.1,
                                          timeout=30.0))
        result = node._on_move(handle)
        assert result.error_code == RESULT_GOAL_INVALID, result.message
        assert "board_a" in result.message, \
            "the refusal must say where the joint actually lives: %s" % result.message
        assert result.motion_completed is False
        assert handle.outcome == "abort"
        assert node._fake_board.commands_received == 0, \
            "a rejected goal must not put a command frame on the bus"
    finally:
        node.destroy_node()


def test_without_a_namespace_the_relative_names_resolve_to_the_historical_roots(context):
    """namespace=None must keep the historical names: /robot/stop, /robot/motor_states.

    The mechanism is that every interface is created with a RELATIVE name, so the
    relativity itself is asserted here, not only the resolved result.
    """
    node = make_node_two(context, "board_a")
    try:
        services = list(node.services)
        assert services, "the node created no services at all"
        for service in services:
            assert not str(service.srv_name).startswith("/"), \
                "service %r was created with an absolute name, which would ignore the " \
                "namespace and collide with a second board node" % service.srv_name
        names = resolved_service_names(node)
        assert absolute(node, "robot/stop") == "/robot/stop"
        assert "/robot/stop" in names, sorted(names)
        assert "/robot/set_zero" in names, sorted(names)
        assert node.states_pub.topic_name == "/robot/motor_states"
    finally:
        node.destroy_node()


def test_namespace_moves_every_relative_interface_under_it(context):
    """Under namespace 'board_b' the same relative names become /board_b/robot/...,
    and the un-namespaced names must NOT exist on this node -- that is what stops
    two boards in one domain from answering each other's service calls."""
    node = make_node_two(context, "board_b", namespace="board_b")
    try:
        names = resolved_service_names(node)
        assert "/board_b/robot/stop" in names, sorted(names)
        assert "/robot/stop" not in names, \
            "a board-B node must not offer the un-namespaced service: %s" % sorted(names)
        assert node.states_pub.topic_name == "/board_b/robot/motor_states"
        if node._diagnostics_available:
            # /diagnostics is the one deliberately ABSOLUTE name: a system-wide
            # aggregation topic where several publishers are normal.
            assert node.diagnostics_pub.topic_name == "/diagnostics"
        else:
            # Without diagnostic_msgs the JSON fallback is relative like everything
            # else, so it moves under the namespace too.
            assert node.diagnostics_pub.topic_name == "/board_b/robot/diagnostics_json"
    finally:
        node.destroy_node()


def test_two_board_nodes_have_disjoint_services_and_their_own_action_servers(context):
    """Two nodes in ONE context: no service name may exist on both, and each board's
    move_joint action must live under its own namespace. The action servers are
    checked through the services they create (send_goal/cancel_goal/get_result),
    which is stable across rclpy versions.
    """
    node_a = make_node_two(context, "board_a", namespace="board_a")
    node_b = make_node_two(context, "board_b", namespace="board_b")
    try:
        a_names = resolved_service_names(node_a)
        b_names = resolved_service_names(node_b)
        assert not (a_names & b_names), \
            "shared service names would let either node answer the other's calls: %s" \
            % sorted(a_names & b_names)
        # Same interface shape on both boards, just under different namespaces.
        assert set(name.replace("/board_a/", "/") for name in a_names) \
            == set(name.replace("/board_b/", "/") for name in b_names)
        for prefix, names in (("/board_a/", a_names), ("/board_b/", b_names)):
            assert prefix + "robot/stop" in names, sorted(names)
            from rclpy.action import get_action_names_and_types
            import time
            deadline = time.monotonic() + 3.0
            expected = {prefix + "robot/move_joint", prefix + "robot/selftest"}
            while time.monotonic() < deadline:
                actions = {name for name, _ in get_action_names_and_types(node_a)}
                if expected.issubset(actions):
                    break
                time.sleep(0.02)
            assert expected.issubset(actions), actions

    finally:
        node_a.destroy_node()
        node_b.destroy_node()


def test_a_move_on_one_board_never_reaches_the_other(context):
    """Isolation: a completed motion on board A must leave board B's simulated bus
    untouched, and vice versa. Each node owns its own transport, worker and fake
    board; anything arriving on the wrong simulator would mean shared state.
    """
    node_a = make_node_two(context, "board_a", namespace="board_a", allow_motion=True,
                           commissioning=True, allow_unreferenced_relative=True)
    node_b = make_node_two(context, "board_b", namespace="board_b", allow_motion=True,
                           commissioning=True, allow_unreferenced_relative=True)
    try:
        quiet_fake_board(node_a)
        quiet_fake_board(node_b)
        handle = FakeGoalHandle(move_goal(joint="head_joint", mode=0, angle_rad=0.1,
                                          timeout=30.0))
        result = node_a._on_move(handle)
        assert result.error_code == 0, result.message
        assert result.motion_completed is True
        assert handle.outcome == "succeed"
        assert node_a._fake_board.commands_received >= 1
        assert node_b._fake_board.commands_received == 0, \
            "a command meant for board A reached board B's bus"

        a_before = node_a._fake_board.commands_received
        handle_b = FakeGoalHandle(move_goal(joint="right_upper_arm_joint", mode=0,
                                            angle_rad=0.1, timeout=30.0))
        result_b = node_b._on_move(handle_b)
        assert result_b.error_code == 0, result_b.message
        assert result_b.motion_completed is True
        assert handle_b.outcome == "succeed"
        assert node_b._fake_board.commands_received >= 1
        assert node_a._fake_board.commands_received == a_before, \
            "a command meant for board B reached board A's bus"
    finally:
        node_a.destroy_node()
        node_b.destroy_node()


def test_unknown_state_on_one_board_does_not_halt_the_other(context):
    """An UNKNOWN outcome halts the board whose command outcome is unknown, and
    only that board: the whole point of one process per board is that a fault on
    one side cannot stop the other from being polled, moved or stopped.
    """
    node_a = make_node_two(context, "board_a", namespace="board_a", allow_motion=True,
                           commissioning=True, allow_unreferenced_relative=True)
    node_b = make_node_two(context, "board_b", namespace="board_b", allow_motion=True,
                           commissioning=True, allow_unreferenced_relative=True)
    try:
        quiet_fake_board(node_a)
        quiet_fake_board(node_b)
        node_a.session.mark_unknown("test")
        assert node_a.session.unknown_command_state is True
        assert node_b.session.unknown_command_state is False, \
            "board B has its own session; A's UNKNOWN must not propagate"
        assert node_a._on_move_goal(move_goal(joint="head_joint")) == GoalResponse.REJECT
        # Board B must still complete a real motion while A is halted.
        handle = FakeGoalHandle(move_goal(joint="right_upper_arm_joint", mode=0,
                                          angle_rad=0.1, timeout=30.0))
        result = node_b._on_move(handle)
        assert result.error_code == 0, result.message
        assert handle.outcome == "succeed"
        assert node_a._fake_board.commands_received == 0, "halted board A sent a command"
        assert node_a.session.unknown_command_state is True, \
            "activity on board B must not clear board A's halt"
    finally:
        node_a.destroy_node()
        node_b.destroy_node()


def test_uid_mismatch_on_one_board_leaves_the_other_driving(context):
    """UID binding is per board: when the hardware behind board B's connector
    reports a different UID than configured, node B records an identity error and
    refuses every write, while node A -- talking to its own, matching board --
    keeps working. This is the swapped-cable case the binding exists for.

    The mismatch is created at the transport boundary, not in the YAML: with
    transport=fake the simulated board is BUILT with the declared UID, so a
    "wrong" declared UID alone cannot produce a mismatch. Mutating the fake
    board's uid and re-running _identify() exercises the same bind-or-reject
    path (BoardSession.identify) the constructor used.
    """
    from robot_motor_interfaces.srv import SetZero
    node_a = make_node_two(context, "board_a", namespace="board_a", allow_motion=True)
    node_b = make_node_two(context, "board_b", namespace="board_b", allow_motion=True)
    try:
        assert node_a.identity_error == ""
        assert node_b.identity_error == ""
        assert node_a.session.uid == UID_BOARD_A
        assert node_b.session.uid == UID_BOARD_B

        # Swap the hardware behind board B's connector: it now reports board A's UID.
        node_b._fake_board.uid = UID_BOARD_A
        node_b._identify()

        assert node_b.identity_error, "a UID mismatch must be recorded as an identity error"
        assert "does not match" in node_b.identity_error, node_b.identity_error
        assert node_b.session.identified is False
        # Every write on B stays refused: goals, and the maintenance services too.
        assert node_b._on_move_goal(
            move_goal(joint="right_upper_arm_joint")) == GoalResponse.REJECT
        request = SetZero.Request()
        request.joint_name = "right_wrist_joint"
        response = node_b._on_set_zero(request, SetZero.Response())
        assert response.accepted is False
        assert "refused" in response.message, response.message
        assert node_b._fake_board.commands_received == 0, \
            "no command frame may reach a board whose identity is unconfirmed"
        # Node A never talked to the swapped board and is untouched.
        assert node_a.identity_error == ""
        assert node_a.session.identified is True
        assert node_a.session.uid == UID_BOARD_A
        assert node_a._on_move_goal(move_goal(joint="head_joint")) == GoalResponse.ACCEPT
    finally:
        node_a.destroy_node()
        node_b.destroy_node()


@pytest.mark.parametrize("target_deg", [0.0, 35.0, 370.0])
def test_review_absolute_targets_reach_worker(context, target_deg):
    # The worker tests independently verify current-position subtraction and
    # delta limits. The ROS callback must not reject absolute coordinates.
    from robot_motor_driver.scheduler import TaskHandle, TaskResult
    node = make_node(context, allow_motion=True, commissioning=True)
    seen = []
    original = node.worker.submit
    def capture(request):
        if request.kind != "move":
            return original(request)
        seen.append(request)
        handle = TaskHandle(request)
        handle._resolve(TaskResult(completed=True, persisted=True, position_valid=True))
        return handle
    node.worker.submit = capture
    try:
        goal = FakeGoalHandle(move_goal(mode=1, angle_rad=math.radians(target_deg)))
        node._on_move(goal)
        assert len(seen) == 1
        assert seen[0].target_deci_deg == round(target_deg * 10)
        assert seen[0].max_step_deci_deg == 100
    finally:
        node.destroy_node()

# Degree facade regression tests. All nodes below use fake transport only.
def degree_goal(angle=5.0, mode=0, joint="head_joint"):
    from robot_motor_interfaces.action import MoveJointDeg
    goal = MoveJointDeg.Goal()
    goal.joint_name = joint
    goal.mode = mode
    goal.angle_deg = float(angle)
    goal.motor_rpm = 0
    goal.timeout_sec = 30.0
    return goal

@pytest.mark.parametrize("angle", [5.0,-5.0,0.0,0.05,-0.05,360.0,-360.0])
def test_degree_and_radian_facades_submit_same_parameters(context, angle):
    from robot_motor_driver.scheduler import TaskHandle, TaskResult
    node = make_node(context, allow_motion=True, commissioning=True,
                     max_commissioning_step_deg=360.0)
    seen = []
    original = node.worker.submit
    def capture(request):
        if request.kind != "move":
            return original(request)
        seen.append(request)
        handle = TaskHandle(request)
        handle._resolve(TaskResult(completed=True, persisted=True, position_valid=True,
                                  final_angle_deci_deg=50, final_angle_rad=math.radians(5)))
        return handle
    node.worker.submit = capture
    try:
        rad = node._on_move(FakeGoalHandle(move_goal(angle_rad=math.radians(angle))))
        deg = node._on_move_deg(FakeGoalHandle(degree_goal(angle)))
        assert rad.error_code == deg.error_code
        if angle == 0:
            assert not seen
        else:
            assert len(seen) == 2
            for key in ("argument","mode","target_deci_deg","channel","expect_rpm",
                        "max_step_deci_deg","limits_rad"):
                assert getattr(seen[0],key) == getattr(seen[1],key)
            assert deg.final_angle_deg == 5.0
    finally:
        node.destroy_node()

@pytest.mark.parametrize("angle", [float("nan"),float("inf"),-float("inf"),360.1,-360.1])
def test_degree_invalid_goal_never_writes(context, angle):
    node = make_node(context, allow_motion=True, commissioning=True)
    try:
        result = node._on_move_deg(FakeGoalHandle(degree_goal(angle)))
        assert result.error_code in (RESULT_GOAL_INVALID, RESULT_LIMIT)
        assert node._fake_board.commands_received == 0
    finally:
        node.destroy_node()

def test_degree_absolute_zero_goes_to_same_worker(context):
    from robot_motor_driver.scheduler import TaskHandle, TaskResult
    node = make_node(context, allow_motion=True, commissioning=True)
    seen = []
    original = node.worker.submit
    def capture(request):
        if request.kind != "move":
            return original(request)
        seen.append(request)
        handle = TaskHandle(request)
        handle._resolve(TaskResult(completed=True))
        return handle
    node.worker.submit = capture
    try:
        node._on_move_deg(FakeGoalHandle(degree_goal(0,mode=1)))
        assert len(seen) == 1
        assert seen[0].target_deci_deg == 0
        assert seen[0].mode == 1
    finally:
        node.destroy_node()

def test_degree_states_share_timestamp_raw_counts_and_invalid_flags(context):
    node = make_node(context)
    try:
        source = node._states_message(node.worker.snapshot)
        source.channels[0].position_valid = True
        source.channels[0].angle_deci_deg = -123
        source.channels[0].target_deci_deg = 456
        source.channels[1].position_valid = False
        source.channels[1].angle_deci_deg = 999
        converted = node._states_message_deg(source)
        assert converted.header == source.header
        assert converted.channels[0].angle_deg == -12.3
        assert converted.channels[0].target_deg == 45.6
        assert converted.channels[1].angle_deg == 0.0
        assert converted.channels[1].angle_deci_deg == 999
        assert converted.channels[1].position_valid is False
        for old,new in zip(source.channels, converted.channels):
            for field in old.get_fields_and_field_types():
                if field not in ("angle_rad","target_rad"):
                    assert getattr(new,field) == getattr(old,field)
    finally:
        node.destroy_node()

def test_degree_simulated_move_and_cancel(context):
    from action_msgs.msg import GoalStatus
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True, max_commissioning_step_deg=360.0)
    try:
        quiet_fake_board(node)
        handle = FakeGoalHandle(degree_goal(5))
        result = node._on_move_deg(handle)
        assert result.motion_completed and result.error_code == 0, result.message
        assert handle.outcome == "succeed"
        assert not result.position_valid
        assert result.final_angle_deg == 0.0
        assert abs(node._fake_board.channels[1].angle_deci_deg - 50) < 2
        quiet_fake_board(node)
        canceled = FakeGoalHandle(degree_goal(300), status=GoalStatus.STATUS_CANCELING)
        result = node._on_move_deg(canceled)
        assert "cancel" in result.message.lower()
        assert canceled.outcome in ("canceled","abort")
    finally:
        node.destroy_node()

@pytest.mark.parametrize("failure", ["fault","storage"])
def test_degree_fault_and_save_failure(context, failure):
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True)
    try:
        quiet_fake_board(node)
        if failure == "fault":
            node._fake_board.inject_fault(1,7)
        else:
            node._fake_board.storage_failure = True
        result = node._on_move_deg(FakeGoalHandle(degree_goal(5)))
        assert not result.motion_completed
        assert result.error_code != 0
        assert not result.persisted
    finally:
        node.destroy_node()

def test_mixed_units_queue_and_stop_priority(context):
    import threading
    import time
    node = make_node(context, allow_motion=True, allow_unreferenced_relative=True,
                     commissioning=True, max_commissioning_step_deg=360.0)
    results = []
    captured = []
    original = node.worker.submit
    def capture(request):
        if request.kind == "move":
            captured.append(request)
        return original(request)
    node.worker.submit = capture
    threads = []
    try:
        quiet_fake_board(node)
        node._fake_board.motion_duration_scale = 100.0
        first = FakeGoalHandle(degree_goal(30))
        second = FakeGoalHandle(move_goal(angle_rad=math.radians(-5),timeout=30.0))
        threads = [
            threading.Thread(target=lambda: results.append(node._on_move_deg(first))),
            threading.Thread(target=lambda: results.append(node._on_move(second)))]
        threads[0].start()
        deadline = time.monotonic() + 3
        while len(captured) < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        threads[1].start()
        deadline = time.monotonic() + 3
        while len(captured) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(captured) == 2
        assert [r.argument for r in captured] == [300,-50]
        stop = node.worker.request_stop(1)
        assert stop.wait(10)
        assert stop.result.stop_confirmed
        for thread in threads:
            thread.join(15)
            assert not thread.is_alive()
        assert len(results) == 2
        assert not any(r.motion_completed for r in results)
        assert node._fake_board.channels[1].brake
    finally:
        for thread in threads:
            thread.join(5)
        node.destroy_node()

def test_degree_simulated_absolute_return_zero(context):
    from robot_motor_interfaces.srv import SetZero
    node = make_node(context, allow_motion=True, maintenance=True,
                     commissioning=True)
    try:
        quiet_fake_board(node)
        request = SetZero.Request()
        request.joint_name = "head_joint"
        response = node._on_set_zero(request, SetZero.Response())
        assert response.accepted, response.message
        quiet_fake_board(node)
        forward = node._on_move_deg(FakeGoalHandle(degree_goal(5)))
        assert forward.motion_completed, forward.message
        quiet_fake_board(node)
        back = node._on_move_deg(FakeGoalHandle(degree_goal(0,mode=1)))
        assert back.motion_completed, back.message
        assert abs(back.final_angle_deg) < 0.2
    finally:
        node.destroy_node()

def test_diagnostic_degree_view_masks_invalid_position(context):
    import json
    node = make_node(context)
    captured = []
    class Publisher:
        def publish(self, message):
            captured.append(message)
    node.diagnostics_pub = Publisher()
    try:
        message = node._states_message(node.worker.snapshot)
        message.channels[0].position_valid = True
        message.channels[0].angle_deci_deg = 123
        message.channels[1].position_valid = False
        message.channels[1].angle_deci_deg = 999
        node._publish_diagnostics(node.worker.snapshot, message)
        if node._diagnostics_available:
            value = next(v.value for v in captured[0].status[0].values if v.key == "angles_deg")
            angles = json.loads(value)
        else:
            angles = json.loads(captured[0].data)["angles_deg"]
        assert angles[0]["angle_deg"] == 12.3
        assert angles[1]["angle_deg"] == 0.0
        assert angles[1]["position_valid"] is False
    finally:
        node.destroy_node()


@pytest.mark.parametrize("board_name,namespace", [("board_a", "board_a"),
                                                  ("board_b", "board_b")])
def test_named_joint_topics_reuse_board_snapshot_without_writes(context, board_name, namespace):
    """YAML names/channels and namespaces select aliases, including invalid positions."""
    import time

    class Recorder:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node = make_node_two(context, board_name, namespace=namespace)
    try:
        deadline = time.monotonic() + 10.0
        while not node.worker.snapshot.uid and time.monotonic() < deadline:
            time.sleep(0.05)
        assert node.worker.snapshot.uid
        mapped = node.config.joints_for_board(board_name)
        assert set(node.joint_state_deg_pubs) == {j.channel for j in mapped}
        for joint in mapped:
            assert node.joint_state_deg_pubs[joint.channel].topic_name == (
                "/%s/robot/joints/%s/state" % (namespace, joint.name))
        node.states_deg_pub = Recorder()
        recorders = {j.channel: Recorder() for j in mapped}
        node.joint_state_deg_pubs = recorders
        for _ in range(2):
            node._publish_states()
            whole = node.states_deg_pub.messages[-1]
            for channel, recorder in recorders.items():
                alias = recorder.messages[-1]
                expected = next(state for state in whole.channels if state.channel == channel)
                assert alias is expected, "aliases must not repoll or rebuild channel values"
                assert alias == expected
                if not alias.position_valid:
                    assert alias.angle_deg == alias.target_deg == 0.0
        assert all(len(rec.messages) == 2 for rec in recorders.values())
        assert node._fake_board.commands_received == 0
        assert node._fake_board.flash_writes == 0
    finally:
        node.destroy_node()


def test_manual_service_and_named_topic(context):
    from robot_motor_interfaces.srv import SetManualAdjust
    node=make_node(context,maintenance=True)
    class Recorder:
        def __init__(self):self.messages=[]
        def publish(self,message):self.messages.append(message)
    try:
        quiet_fake_board(node)
        joint=node.config.joint("head_joint")
        request=SetManualAdjust.Request();request.joint_name="head_joint";request.enable=True
        response=node._on_set_manual_adjust(request,SetManualAdjust.Response())
        assert response.accepted and response.active and not response.position_valid and not response.persisted
        recorder=Recorder();node.manual_adjust_pubs[joint.channel]=recorder
        node._publish_states();m=recorder.messages[-1]
        assert m.joint_name=="head_joint" and m.channel==joint.channel and m.supported and m.mode==1
        assert m.active and not m.position_valid and m.dirty
        request.enable=False
        response=node._on_set_manual_adjust(request,SetManualAdjust.Response())
        assert response.accepted and not response.active and not response.exiting and response.persisted
    finally:node.destroy_node()

def test_manual_readonly_service_no_write(context):
    from robot_motor_interfaces.srv import SetManualAdjust
    node=make_node(context)
    try:
        request=SetManualAdjust.Request();request.joint_name="head_joint";request.enable=True
        response=node._on_set_manual_adjust(request,SetManualAdjust.Response())
        assert not response.accepted and node._fake_board.commands_received==0
    finally:node.destroy_node()
