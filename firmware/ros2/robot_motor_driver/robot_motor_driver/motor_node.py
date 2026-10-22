"""ROS 2 node: the adapter between ROS interfaces and the board worker.

Threading model, which is the main structural difference from the legacy node in
/opt/ros/motor_control_ws (that one ran a blocking read-compare-move loop inside a
service callback on a single-threaded executor):

* ALL serial I/O happens on the BoardWorker thread. The node never touches the
  transport.
* ROS timers and callbacks only read the published snapshot or submit tasks, so no
  callback can stall the executor for the duration of a motion.
* Long-running goals wait on their own thread and report through the action API.

Safety defaults required by the execution plan:
* allow_motion is false, and read-only mode intercepts every write, including STOP.
* Nothing is sent at startup or shutdown beyond reads, unless motion is allowed;
  the shutdown STOP is explicitly best effort and is logged as unconfirmed when the
  board does not verify it.
* An absolute move is refused while position_valid is false; a relative move with
  position_valid false additionally needs allow_unreferenced_relative.
* No automatic zeroing, fault clearing, self test, or motion resumption anywhere.
"""

import math
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_motor_interfaces.action import MoveJoint, MoveJointDeg, RunSelfTest
from robot_motor_interfaces.msg import ChannelConfig, MotorState, MotorStates, MotorStateDeg, MotorStatesDeg, SelftestChannel, ManualAdjustState
from robot_motor_interfaces.srv import ApplyConfig, ClearFault, SetBrake, SetZero, Stop, SetManualAdjust, RetrySave, SetDirection

from . import protocol as proto
from .board_session import (DRIVER_LIMIT, DRIVER_LIMITS_UNCONFIRMED, DRIVER_NO_OP, DRIVER_OK,
                            DRIVER_READONLY, DRIVER_RESULT_NAMES, DRIVER_RPM_MISMATCH,
                            DRIVER_VALID_REQUIRED, BoardSession)
from .config import ConfigError, load_config
from .fake_board import FakeBoard, SerialStub
from .scheduler import BoardWorker, TaskRequest
from .transport_modbus import ModbusRtuTransport, TransportNotAvailable

#: Action result codes above the firmware's 0..6 business results.
RESULT_NO_OP = DRIVER_NO_OP
RESULT_LIMIT = DRIVER_LIMIT
RESULT_VALID_REQUIRED = DRIVER_VALID_REQUIRED
RESULT_LIMITS_UNCONFIRMED = DRIVER_LIMITS_UNCONFIRMED
RESULT_RPM_MISMATCH = DRIVER_RPM_MISMATCH
RESULT_REJECTED = 120
RESULT_GOAL_INVALID = 121

#: Default commissioning step ceiling: with mechanical limits unconfirmed, one command
#: is capped so a debugging session cannot sweep a joint through an unknown range.
DEFAULT_COMMISSIONING_STEP_DEG = 10.0

MOVE_MODE_RELATIVE = 0
MOVE_MODE_ABSOLUTE = 1


def _goal_is_active(goal_handle):
    # type: (object) -> bool
    try:
        return bool(goal_handle.is_active)
    except Exception:      # pragma: no cover - distro variance
        return True


def _goal_is_canceling(goal_handle):
    # type: (object) -> bool
    """Cancellation check that survives rclpy API spelling differences.

    A source-built Humble was observed WITHOUT `ServerGoalHandle.is_canceling`.
    Touching a missing attribute inside the execute callback aborts the goal with an
    EMPTY result AFTER the command frame has already gone out, i.e. the mechanism
    keeps moving while ROS reports a failure. That is why this never accesses the
    attribute directly.
    """
    for name in ("is_canceling", "is_cancelling"):
        value = getattr(goal_handle, name, None)
        if value is not None:
            return bool(value)
    try:
        from action_msgs.msg import GoalStatus
        return int(goal_handle.status) == int(GoalStatus.STATUS_CANCELING)
    except Exception:      # pragma: no cover - defensive
        return False

DEFAULT_POLLING_HZ = 5.0
MAX_POLLING_HZ = 100.0
MIN_TIMEOUT_SEC = 0.5
MAX_TIMEOUT_SEC = 300.0
#: Freshness limit: a snapshot older than this is published as stale.
STALE_AFTER_SEC = 1.0


class RobotMotorNode(Node):
    """Driver node for ONE GD32 robot motor board.

    Several boards are driven by several node processes, one per board, each selecting
    its board with the `board_name` parameter and each usually under its own namespace.
    See dual_board.launch.py for why that is safer than one process driving both.
    """

    def __init__(self, config=None, node_name="robot_motor_driver", parameter_overrides=None,
                 namespace=None):
        # type: (Optional[object], str, Optional[list], Optional[str]) -> None
        # parameter_overrides is how tests (and any programmatic caller) set
        # allow_motion/maintenance BEFORE the constructor reads them; setting ROS
        # parameters afterwards would have no effect on the wiring below.
        # namespace decides where the RELATIVE interface names land: "" or None keeps
        # the historical /robot/... names, "board_b" moves them to /board_b/robot/...
        super().__init__(node_name, namespace=namespace or '',
                         parameter_overrides=parameter_overrides or [])
        self._callback_group = ReentrantCallbackGroup()

        # config_file has to be known BEFORE the policy parameters are declared, so
        # that their defaults come from the YAML file and a launch argument still
        # overrides them. Otherwise the file and the parameter disagree silently and
        # the node honours only one of them.
        self.declare_parameter("config_file", "")
        self.config = config
        if self.config is None:
            path = self.get_parameter("config_file").value
            try:
                self.config = load_config(path)
            except ConfigError as error:
                self.get_logger().error("configuration rejected: %s" % error)
                raise
        self._declare_parameters()
        self.get_logger().info("configuration: %s" % _summarise(self.config))

        transport_override = self.get_parameter("transport").value
        allow_motion = bool(self.get_parameter("allow_motion").value)
        maintenance = bool(self.get_parameter("maintenance").value)
        self.save_recovery_only = bool(self.get_parameter("save_recovery_only").value)
        if self.save_recovery_only:
            allow_motion = False
            maintenance = True
            self.get_logger().warn("SAVE recovery only: all writes except STOP and RetrySave are refused")
        polling_hz = float(self.get_parameter("polling_hz").value)
        if not 0.0 < polling_hz <= MAX_POLLING_HZ:
            raise ConfigError("polling_hz must be in (0, %.1f], got %r" % (MAX_POLLING_HZ, polling_hz))

        # Read-only intercepts every write, including STOP. Maintenance mode allows
        # stop/zero/clear/brake/save/config but still refuses motion; motion needs
        # allow_motion. This is the explicit maintenance entry the plan requires.
        read_only = not (allow_motion or maintenance)
        self.allow_motion = allow_motion
        self.maintenance = maintenance
        self.commissioning = bool(self.get_parameter("commissioning").value)
        self.max_commissioning_step_deg = float(
            self.get_parameter("max_commissioning_step_deg").value)
        if self.commissioning:
            self.get_logger().warn(
                "commissioning mode is ON: joints whose limits_rad is null may be moved WITHOUT any "
                "soft-limit protection, capped at %.1f deg per command. This is a debugging mode, "
                "not a production setting." % self.max_commissioning_step_deg)
        #: True only when every mapped joint has confirmed mechanical limits.
        self.soft_limits_enforced = all(
            joint.limits_rad is not None for joint in self.config.joints.values())
        self.publish_joint_states = bool(self.get_parameter("publish_joint_states").value)

        self.board_name, self.board = self._select_board()
        mapped = self.config.joints_for_board(self.board_name)
        names = {joint.name for joint in mapped}
        raw_allowed = str(self.get_parameter("allowed_motion_joints").value).strip()
        selected = names if raw_allowed == "*" else {name.strip() for name in raw_allowed.split(",") if name.strip()}
        if not selected.issubset(names):
            raise ConfigError("allowed_motion_joints contains unknown joints: %s" % sorted(selected - names))
        self.allowed_motion_joints = selected
        allowed_channels = None if raw_allowed == "*" else {joint.channel for joint in mapped if joint.name in selected}
        expected_configs = {joint.channel: (True, int(joint.role), int(joint.gear_ratio), int(joint.inverted)) for joint in mapped}
        self.transport = self._build_transport(transport_override)
        response_timeout = float(self.get_parameter("response_timeout").value)
        self.transport.response_timeout = response_timeout

        expected_uid = self.board.uid if bool(self.get_parameter("require_uid").value) else None
        self.session = BoardSession(self.transport, expected_uid=expected_uid,
                                    read_only=read_only,
                                    logger=lambda level, message: self._log(level, message))
        self.worker = BoardWorker(self.session, poll_interval=1.0 / polling_hz,
                                  allow_motion=allow_motion, allowed_channels=allowed_channels,
                                  expected_configs=expected_configs,
                                  save_recovery_only=self.save_recovery_only,
                                  logger=lambda level, message: self._log(level, message))
        self.polling_hz = polling_hz
        self.stale_after_sec = float(self.get_parameter("stale_after_sec").value)
        self.identity_error = ""

        self._joint_by_channel = {}    # type: Dict[int, object]
        self._joints_by_name = {}      # type: Dict[str, object]
        for joint in self.config.joints_for_board(self.board_name):
            self._joint_by_channel[joint.channel] = joint
            self._joints_by_name[joint.name] = joint

        self._lock = threading.RLock()
        self._create_interfaces()

        # Identity is established before anything else; a mismatch is not fatal for
        # the node (it stays up and keeps reporting), but it blocks every write.
        self._identify()

        # Identification ran on THIS thread, but from here on the worker thread is the
        # only owner of the port. Without the hand-over, strict ownership would make
        # the worker's first transaction fail.
        release = getattr(self.transport, "release_ownership", None)
        if release is not None:
            release()
        self.worker.start()
        self._timer = self.create_timer(1.0 / polling_hz, self._publish_states,
                                        callback_group=self._callback_group)
        # Keep "ready: transport=" as the leading substring: stage_c_realboard_readonly.sh
        # and the test procedures grep for exactly that to prove which backend is live.
        self.get_logger().info(
            "ready: transport=%s port=%s board=%s namespace=%s allow_motion=%s maintenance=%s "
            "read_only=%s commissioning=%s polling=%.1f Hz joints=%s"
            % (self._effective_transport, self._effective_port or "(none)",
               self.board_name, self.get_namespace() or "/",
               allow_motion, maintenance, read_only, self.commissioning, polling_hz,
               sorted(self._joints_by_name)))
        if self._effective_transport != (self.board.transport or "").strip():
            self.get_logger().warn(
                "the configuration says transport=%r but this node is actually running "
                "transport=%r (a launch/parameter override won). Judge fake-vs-real by THIS line "
                "and by the port occupancy, never by the YAML or by the UID -- a simulated board "
                "adopts the UID from the configuration."
                % (self.board.transport, self._effective_transport))

    # ---------------------------------------------------------------- parameters

    def _declare_parameters(self):
        # type: () -> None
        """Declare everything except config_file, defaulting to the YAML policy.

        A launch argument or a programmatic override still wins, because overrides
        are applied at declaration time. `allow_motion` and
        `allow_unreferenced_relative` therefore cannot be enabled by accident: both
        the file and the override default to false.
        """
        config = self.config
        self.declare_parameter("transport", "")
        self.declare_parameter("allow_motion", bool(config.allow_motion))
        self.declare_parameter("allow_unreferenced_relative",
                               bool(config.allow_unreferenced_relative))
        self.declare_parameter("maintenance", False)
        self.declare_parameter("save_recovery_only", False)
        self.declare_parameter("allowed_motion_joints", "*")
        self.declare_parameter("commissioning", False)
        self.declare_parameter("max_commissioning_step_deg", DEFAULT_COMMISSIONING_STEP_DEG)
        self.declare_parameter("polling_hz", float(config.polling_hz))
        self.declare_parameter("stale_after_sec", STALE_AFTER_SEC)
        self.declare_parameter("response_timeout", proto.DEFAULT_RESPONSE_TIMEOUT)
        self.declare_parameter("require_uid", True)
        self.declare_parameter("publish_joint_states", False)
        #: Relative names on purpose: they resolve against this node's namespace, so a
        #: single board with no namespace still gets /joint_states and
        #: /robot/motor_states, while a second board launched under namespace "board_b"
        #: gets /board_b/robot/motor_states. Passing an ABSOLUTE name here opts out of
        #: that and will collide with another board in the same domain.
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("motor_states_topic", "robot/motor_states")
        self.declare_parameter("motor_states_deg_topic", "robot/motor_states_deg")
        self.declare_parameter("publish_diagnostics", True)
        #: Which board of the configuration this process drives. Empty means "the only
        #: one"; with several boards declared it must be set, so that one process can
        #: never end up driving a board nobody chose.
        self.declare_parameter("board_name", "")

    def _select_board(self):
        # type: () -> tuple
        """Pick the ONE board this process drives.

        A process never drives two boards: separate processes give each board its own
        serial-ownership thread, its own sequence allocator and dedupe cache, and its
        own UNKNOWN state, so one board hanging or losing its port cannot stop the
        other from being polled or stopped. It also lets each board carry its own
        policy (one read-only while the other moves), which node-level parameters could
        not express.
        """
        boards = self.config.boards
        wanted = str(self.get_parameter("board_name").value or "").strip()
        if not boards:
            raise ConfigError("the configuration declares no boards")
        if wanted:
            if wanted not in boards:
                raise ConfigError(
                    "board_name %r is not in the configuration; declared boards: %s"
                    % (wanted, sorted(boards)))
            return wanted, boards[wanted]
        if len(boards) == 1:
            name = sorted(boards)[0]
            return name, boards[name]
        raise ConfigError(
            "the configuration declares %d boards (%s) and board_name is empty. This node drives "
            "exactly one board per process: set board_name:=<name> and launch one node per board, "
            "each under its own namespace (see bringup.launch.py / dual_board.launch.py). Refusing "
            "to guess which board should get the motion commands." % (len(boards), sorted(boards)))

    def _build_transport(self, override):
        # type: (str) -> object
        transport_name = (override or self.board.transport or "modbus_rtu").strip()
        # The EFFECTIVE transport, not the one written in the YAML: a launch override
        # wins, and logging the config value would let an operator believe a simulated
        # run was talking to real hardware (or the other way round). That confusion is
        # exactly what the identification procedure is supposed to prevent.
        self._effective_transport = transport_name
        self._effective_port = ""
        if transport_name == "fake":
            board = FakeBoard(uid=self.board.uid or "000000000000000000000000")
            stub = SerialStub(board)
            self._effective_port = "(simulated: no serial port is opened)"
            self.get_logger().warn(
                "transport=fake: talking to a SIMULATED board. No serial port is opened and no "
                "result from this run is evidence about real hardware.")
            transport = ModbusRtuTransport(port=stub.port, address=self.board.address or 1,
                                           serial_factory=lambda *a, **k: stub,
                                           strict_ownership=False)
            transport.open()
            self._fake_board = board
            self._fake_stub = stub
            return transport
        if transport_name == "modbus_rtu":
            if not self.board.port:
                raise ConfigError("board %s has transport modbus_rtu but no port" % self.board_name)
            transport = ModbusRtuTransport(port=self.board.port, address=self.board.address,
                                           baudrate=self.board.baudrate,
                                           strict_ownership=True)
            transport.open()
            self._effective_port = self.board.port
            self.get_logger().info("opened %s at %d baud, address %d"
                                   % (self.board.port, self.board.baudrate, self.board.address))
            return transport
        if transport_name == "can":
            from .transport_can import CanTransport
            transport = CanTransport(interface=self.board.port, channel=self.board.port,
                                     node=self.board.node)
            # Raises TransportNotAvailable until python-can exists AND somebody has
            # produced hardware evidence for the CAN path.
            transport.open()
            self._effective_port = self.board.port
            return transport
        raise ConfigError("unknown transport %r (expected fake, modbus_rtu or can)" % transport_name)

    def _identify(self):
        # type: () -> None
        try:
            uid = self.session.identify()
            self.identity_error = ""
            self.get_logger().info("board identified: uid=%s command_protocol=%d"
                                   % (uid, self.session.command_protocol))
        except proto.IdentityError as error:
            self.identity_error = str(error)
            self.get_logger().error(
                "IDENTITY REJECTED: %s. Every write stays refused; check that the configured UID "
                "belongs to the board that is actually connected." % error)
        except (proto.ProtocolError, TransportNotAvailable, OSError) as error:
            self.identity_error = "board not reachable: %s" % error
            self.get_logger().error("%s (node stays up and keeps reporting)" % self.identity_error)

    # -------------------------------------------------------------- interfaces

    def _absolute_name(self, relative):
        # type: (str) -> str
        """Resolve a relative interface name the same way rclpy will.

        Computed here rather than read back from the created Service/Publisher object:
        this source-built Humble's rclpy `Service` has no `service_name` attribute, so
        relying on entity attributes would break on exactly the host we ship to.
        """
        namespace = (self.get_namespace() or "/").rstrip("/")
        return "%s/%s" % (namespace, str(relative).lstrip("/"))

    def _create_interfaces(self):
        # type: () -> None
        # Every name below is RELATIVE, so it resolves against this node's namespace.
        # With no namespace (the single-board case) "robot/stop" becomes "/robot/stop",
        # exactly as before. With namespace "board_b" it becomes "/board_b/robot/stop".
        # That is what lets two boards coexist in one DDS domain without fighting over
        # the same service: a request can only reach the node that owns the name.
        # "/diagnostics" stays absolute on purpose -- it is a system-wide aggregation
        # topic and several publishers on it are normal.
        group = self._callback_group
        self.states_pub = self.create_publisher(
            MotorStates, str(self.get_parameter("motor_states_topic").value), 10)
        self.states_deg_pub = self.create_publisher(
            MotorStatesDeg, str(self.get_parameter("motor_states_deg_topic").value), 10)
        # Per-joint aliases follow the YAML binding, never a fixed array index.
        # They carry the same MotorStateDeg objects as the whole-board snapshot.
        self.joint_state_deg_pubs = {
            joint.channel: self.create_publisher(
                MotorStateDeg, "robot/joints/%s/state" % joint.name, 10)
            for joint in self.config.joints_for_board(self.board_name)
        }
        self.manual_adjust_pubs = {
            joint.channel: self.create_publisher(
                ManualAdjustState, "robot/joints/%s/manual_adjust" % joint.name, 10)
            for joint in self.config.joints_for_board(self.board_name)
        }
        if self.publish_joint_states:
            self.joint_pub = self.create_publisher(
                JointState, str(self.get_parameter("joint_states_topic").value), 10)
            self.get_logger().warn(
                "publishing %s: a live ros2_control stack on the same DDS domain will consume "
                "these positions. Keep the dedicated ROS_DOMAIN_ID unless that is intended."
                % self.get_parameter("joint_states_topic").value)
        else:
            self.joint_pub = None

        self._diagnostics_available = False
        if bool(self.get_parameter("publish_diagnostics").value):
            try:
                from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
                self._diagnostic_types = (DiagnosticArray, DiagnosticStatus, KeyValue)
                self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
                self._diagnostics_available = True
            except ImportError:
                self.diagnostics_pub = self.create_publisher(String, "robot/diagnostics_json", 10)
                self.get_logger().warn(
                    "diagnostic_msgs is not installed on this host, so /diagnostics cannot be "
                    "published. Falling back to robot/diagnostics_json (std_msgs/String). "
                    "Install the diagnostic_msgs package to get the standard topic.")

        self.create_service(Stop, "robot/stop", self._on_stop, callback_group=group)
        self.create_service(ClearFault, "robot/clear_fault", self._on_clear_fault,
                            callback_group=group)
        self.create_service(SetZero, "robot/set_zero", self._on_set_zero, callback_group=group)
        self.create_service(RetrySave, "robot/retry_save", self._on_retry_save, callback_group=group)
        self.create_service(SetBrake, "robot/set_brake", self._on_set_brake, callback_group=group)
        self.create_service(SetDirection, "robot/set_direction", self._on_set_direction, callback_group=group)
        self.create_service(SetManualAdjust, "robot/set_manual_adjust", self._on_set_manual_adjust,
                            callback_group=group)
        self.create_service(ApplyConfig, "robot/apply_config", self._on_apply_config,
                            callback_group=group)
        self.create_service(Trigger, "robot/acknowledge_unknown", self._on_acknowledge_unknown,
                            callback_group=group)
        self.create_service(Trigger, "robot/board_identity", self._on_board_identity,
                            callback_group=group)

        self._move_server = ActionServer(
            self, MoveJoint, "robot/move_joint", execute_callback=self._on_move,
            goal_callback=self._on_move_goal, cancel_callback=self._on_move_cancel,
            callback_group=group)
        self._move_deg_server = ActionServer(
            self, MoveJointDeg, "robot/move_joint_deg", execute_callback=self._on_move_deg,
            goal_callback=self._on_move_goal, cancel_callback=self._on_move_cancel,
            callback_group=group)
        self._selftest_server = ActionServer(
            self, RunSelfTest, "robot/selftest", execute_callback=self._on_selftest,
            goal_callback=self._on_move_goal, cancel_callback=self._on_move_cancel,
            callback_group=group)

    # ------------------------------------------------------------- publishing

    def _publish_states(self):
        # type: () -> None
        snapshot = self.worker.snapshot
        message = self._states_message(snapshot)
        self.states_pub.publish(message)
        degrees = self._states_message_deg(message)
        self.states_deg_pub.publish(degrees)
        for state in degrees.channels:
            publisher = self.joint_state_deg_pubs.get(state.channel)
            if publisher is not None:
                publisher.publish(state)
            manual_pub = self.manual_adjust_pubs.get(state.channel)
            if manual_pub is not None:
                manual = ManualAdjustState()
                manual.header = message.header
                manual.joint_name = self._joint_by_channel[state.channel].name
                manual.channel = state.channel
                manual.supported = bool(snapshot.capabilities & proto.CAP_MANUAL_ADJUST)
                manual.mode = snapshot.manual_modes[state.channel] if manual.supported else 0
                manual.active = manual.mode == 1
                manual.exiting = manual.mode == 2
                manual.position_valid = state.position_valid
                for field in ("identity_ok", "communication_ok", "stale", "age_sec", "dirty", "storage_error", "pending_save"):
                    setattr(manual, field, getattr(message, field))
                manual_pub.publish(manual)

        if self.joint_pub is not None:
            self._publish_joint_states(snapshot, message.stale)
        self._publish_diagnostics(snapshot, message)

    def _states_message(self, snapshot):
        # type: (object) -> MotorStates
        """Build the published snapshot. Split out from publishing so the exact bytes a
        consumer would receive can be asserted in tests."""
        message = MotorStates()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.board_name
        message.uid = snapshot.uid or ""
        message.command_protocol = int(snapshot.command_protocol)
        message.identity_ok = bool(snapshot.uid) and not self.identity_error
        message.communication_ok = bool(snapshot.communication_ok) and self.worker.connected
        age = self.worker.snapshot_age()
        message.age_sec = float(age) if age != float("inf") else -1.0
        message.stale = (not message.communication_ok) or age > self.stale_after_sec
        message.last_sequence = int(snapshot.last_sequence)
        message.last_result = int(snapshot.last_result)
        if snapshot.info is not None:
            message.dirty = bool(snapshot.info.dirty)
            message.storage_error = bool(snapshot.info.storage_error)
            message.pending_save = bool(snapshot.info.pending_save)
        message.read_only = bool(self.session.read_only)
        message.unknown_command_state = bool(self.session.unknown_command_state)
        message.unknown_detail = self.session.unknown_detail or self.identity_error
        # Published so a consumer can SEE that soft limits are not protecting this
        # robot, instead of having to read the configuration file to work it out.
        message.commissioning = bool(self.commissioning)
        message.soft_limits_enforced = bool(self.soft_limits_enforced)

        for channel in range(proto.CHANNEL_COUNT):
            message.channels.append(self._channel_message(snapshot, channel))
        return message

    def _states_message_deg(self, source):
        """Derive both unit views from the very same snapshot and timestamp."""
        message = MotorStatesDeg()
        for field in MotorStates.get_fields_and_field_types():
            if field != "channels":
                setattr(message, field, getattr(source, field))
        for old in source.channels:
            state = MotorStateDeg()
            for field in MotorState.get_fields_and_field_types():
                if field not in ("angle_rad", "target_rad"):
                    setattr(state, field, getattr(old, field))
            if old.position_valid:
                state.angle_deg = proto.deci_deg_to_degrees(old.angle_deci_deg)
                state.target_deg = proto.deci_deg_to_degrees(old.target_deci_deg)
            message.channels.append(state)
        return message

    def _channel_message(self, snapshot, channel):
        # type: (object, int) -> MotorState
        # Every field here is what the BOARD reported. The values the YAML intends
        # (role, gear, enabled) are deliberately NOT substituted when the board
        # disagrees, so a consumer can always tell "configured on the board" from
        # "planned in the file". Divergence is reported through diagnostics instead.
        state = MotorState()
        state.channel = int(channel)
        if channel < len(snapshot.configs):
            config = snapshot.configs[channel]
            state.enabled = bool(config.enabled)
            state.role = int(config.role)
            state.gear_ratio = int(config.gear)
            state.inverted = int(config.inverted)
            state.configured_rpm = int(config.rpm)
        if channel < len(snapshot.statuses):
            status = snapshot.statuses[channel]
            state.state = int(status.state)
            state.fault = int(status.fault)
            state.position_valid = bool(status.valid)
            state.brake = bool(status.brake)
            state.direction = bool(status.direction)
            state.duty_permille = int(status.duty_permille)
            state.measured_rpm = int(status.rpm)
        if channel < len(snapshot.positions):
            position = snapshot.positions[channel]
            state.angle_deci_deg = int(position.angle_deci_deg)
            state.fg_count = int(position.fg_count)
            state.target_deci_deg = int(position.target_deci_deg)
            state.angle_rad = float(proto.deci_deg_to_rad(position.angle_deci_deg))
            state.target_rad = float(proto.deci_deg_to_rad(position.target_deci_deg))
        if channel < len(snapshot.selftests):
            record = snapshot.selftests[channel]
            state.selftest_state = int(record.state)
            state.selftest_reason = int(record.reason)
            state.selftest_phase = int(record.phase)
            state.selftest_outbound_pulses = int(record.outbound_pulses)
            state.selftest_return_pulses = int(record.return_pulses)
            state.selftest_residual_pulses = int(record.residual_pulses)
            state.selftest_powered_ms = int(record.powered_ms)
            state.selftest_return_ms = int(record.return_ms)
            state.selftest_sequence = int(record.sequence)
        gear = state.gear_ratio
        state.pulses_per_output_rev = int(proto.pulses_per_output_rev(gear))
        if not state.position_valid:
            # Without a trusted zero the angle is a diagnostic conversion of a pulse
            # count, not a pose. Publishing it as a position would be a lie, so the
            # radian fields are zeroed and consumers must read position_valid.
            state.angle_rad = 0.0
            state.target_rad = 0.0
        return state

    def _publish_joint_states(self, snapshot, stale):
        # type: (object, bool) -> None
        """Publish JointState ONLY for joints with a fresh, valid position."""
        if stale:
            return
        names = []     # type: List[str]
        positions = []  # type: List[float]
        for channel, joint in sorted(self._joint_by_channel.items()):
            if channel >= len(snapshot.statuses) or channel >= len(snapshot.positions):
                continue
            status = snapshot.statuses[channel]
            position = snapshot.positions[channel]
            if not status.valid:
                continue
            names.append(joint.name)
            positions.append(float(proto.deci_deg_to_rad(position.angle_deci_deg)))
        if not names:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = names
        message.position = positions
        # effort is deliberately left empty: the board reports no torque or current,
        # and inventing one would be fabricated evidence.
        self.joint_pub.publish(message)

    def _publish_diagnostics(self, snapshot, message):
        # type: (object, MotorStates) -> None
        if not bool(self.get_parameter("publish_diagnostics").value):
            return
        problems = []    # type: List[str]
        if self.identity_error:
            problems.append("identity: %s" % self.identity_error)
        if not message.communication_ok:
            problems.append("communication: %s" % (snapshot.error or "no valid poll"))
        if message.stale:
            problems.append("data stale (age %.2f s)" % message.age_sec)
        if message.unknown_command_state:
            problems.append("UNKNOWN command state: %s" % message.unknown_detail)
        if snapshot.info is not None:
            if snapshot.info.storage_error:
                problems.append("storage_error set: MOVE and SELFTEST are blocked until a save "
                                "succeeds")
            if snapshot.info.dirty:
                problems.append("dirty set: a command was accepted but its final state is not "
                                "persisted yet")
            if snapshot.info.pending_save:
                problems.append("pending_save set")
        for channel in range(len(snapshot.statuses)):
            status = snapshot.statuses[channel]
            if status.fault:
                problems.append("channel %d fault %d (%s)" % (channel, status.fault, status.fault_name))
            if channel < len(snapshot.configs):
                joint = self._joint_by_channel.get(channel)
                if joint is not None:
                    expected = (True, int(joint.role), int(joint.gear_ratio), int(joint.inverted),
                                int(joint.expected_motor_rpm))
                    actual = (snapshot.configs[channel].enabled, int(snapshot.configs[channel].role),
                              int(snapshot.configs[channel].gear),
                              int(snapshot.configs[channel].inverted),
                              int(snapshot.configs[channel].rpm))
                    if expected != actual:
                        problems.append(
                            "channel %d (%s) configuration differs from YAML: expected "
                            "(enabled, role, gear, inverted, rpm)=%s, board has %s. The board's "
                            "values are what actually runs: rpm in particular comes from the board "
                            "(BSP/robot.c:334), this driver never writes it per move. Use this "
                            "board's %s service to change it."
                            % (channel, joint.name, expected, actual,
                               self._absolute_name("robot/apply_config")))
                    if joint.limits_rad is None:
                        problems.append(
                            "channel %d (%s) has limits_rad null: mechanical limits are "
                            "UNCONFIRMED, so no soft-limit protection applies and motion is "
                            "refused unless commissioning:=true"
                            % (channel, joint.name))
        if not self.allow_motion:
            problems.append("allow_motion is false: every motion request is refused")
        if self.commissioning:
            problems.append(
                "commissioning mode is ON: motion is allowed although limits are unconfirmed, "
                "capped at %.1f deg per command, with NO soft-limit protection"
                % self.max_commissioning_step_deg)

        import json
        angles_deg = [{"channel": int(state.channel),
                       "position_valid": bool(state.position_valid),
                       "angle_deg": proto.deci_deg_to_degrees(state.angle_deci_deg) if state.position_valid else 0.0,
                       "target_deg": proto.deci_deg_to_degrees(state.target_deci_deg) if state.position_valid else 0.0}
                      for state in message.channels]
        if self._diagnostics_available:
            array, status_type, key_value = self._diagnostic_types
            report = array()
            report.header.stamp = self.get_clock().now().to_msg()
            entry = status_type()
            entry.hardware_id = message.uid or "unknown"
            entry.name = "robot_motor_driver/%s" % self.board_name
            entry.level = status_type.OK if not problems else status_type.WARN
            entry.message = "ok" if not problems else "%d issue(s)" % len(problems)
            entry.values = [key_value(key="issue %d" % index, value=text)
                            for index, text in enumerate(problems)]
            entry.values += [key_value(key="angles_deg", value=json.dumps(angles_deg))]
            report.status.append(entry)
            self.diagnostics_pub.publish(report)
        else:
            import json
            payload = {"level": "OK" if not problems else "WARN",
                       "uid": message.uid,
                       "read_only": message.read_only,
                       "unknown": message.unknown_command_state,
                       "angles_deg": angles_deg,
                       "problems": problems}
            self.diagnostics_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # -------------------------------------------------------------- validation

    def _resolve_joint(self, name):
        # type: (str) -> Optional[object]
        """Resolve a joint name WITHIN THE BOARD THIS PROCESS DRIVES.

        Deliberately not `self.config.joint(name)`: the configuration may describe
        several boards, and resolving against the global table would let a goal sent to
        board A name a joint of board B. The channel number would then be applied to
        board A's own connector -- a different physical joint on a different board, with
        board A's gear ratio. That is the worst kind of failure: it looks like success.
        """
        return self._joints_by_name.get(name)

    def _unknown_joint_text(self, name):
        # type: (str) -> str
        """Explain a failed lookup, naming the board that does own the joint."""
        joint = self._joints_by_name.get(name)
        if joint is not None:                       # pragma: no cover - defensive
            return ""
        try:
            elsewhere = self.config.joint(name)
        except ConfigError:
            return ("unknown joint %r; this node drives board %r with joints %s"
                    % (name, self.board_name, sorted(self._joints_by_name)))
        return ("joint %r belongs to board %r, which this node (board %r) does not drive. "
                "Send it to that board's own interface instead -- routing it here would apply "
                "channel %d to a different board's connector."
                % (name, elsewhere.board, self.board_name, elsewhere.channel))

    def _validate_move_goal(self, goal, degrees=False):
        # type: (object) -> tuple
        """Return (error_code, message, joint) for a MoveJoint goal."""
        joint = self._resolve_joint(goal.joint_name)
        if joint is None:
            return RESULT_GOAL_INVALID, self._unknown_joint_text(goal.joint_name), None
        if joint.name not in self.allowed_motion_joints:
            return RESULT_GOAL_INVALID, "joint is outside allowed_motion_joints", joint
        angle = goal.angle_deg if degrees else goal.angle_rad
        unit = "deg" if degrees else "rad"
        if not math.isfinite(angle):
            return RESULT_GOAL_INVALID, "angle_%s must be finite, got %r" % (unit, angle), joint
        if degrees and goal.mode == MOVE_MODE_RELATIVE and abs(angle) > 360.0:
            return RESULT_LIMIT, "single MOVE is limited to +/-360 deg", joint
        if goal.mode not in (MOVE_MODE_RELATIVE, MOVE_MODE_ABSOLUTE):
            return RESULT_GOAL_INVALID, "mode must be 0 (relative) or 1 (absolute)", joint
        if goal.motor_rpm and not proto.RPM_MIN <= int(goal.motor_rpm) <= proto.RPM_MAX:
            return RESULT_GOAL_INVALID, "motor_rpm must be 0 (use default) or %d..%d, got %d" \
                   % (proto.RPM_MIN, proto.RPM_MAX, goal.motor_rpm), joint
        if not MIN_TIMEOUT_SEC <= float(goal.timeout_sec) <= MAX_TIMEOUT_SEC:
            return RESULT_GOAL_INVALID, "timeout_sec must be %.1f..%.1f, got %r" \
                   % (MIN_TIMEOUT_SEC, MAX_TIMEOUT_SEC, goal.timeout_sec), joint
        if degrees and abs(angle) > 214748364.7:
            return RESULT_LIMIT, "angle_deg exceeds signed 32-bit position range", joint
        deci = (proto.degrees_to_deci_deg(angle) if degrees else proto.rad_to_deci_deg(angle))
        try:
            if goal.mode == MOVE_MODE_RELATIVE:
                proto.validate_move_argument(deci)
        except ValueError as error:
            return RESULT_LIMIT, "single MOVE is limited to +/-360 deg and is not split: %s" % error, joint
        if joint.limits_rad is not None and goal.mode == MOVE_MODE_ABSOLUTE:
            low, high = joint.limits_rad
            if degrees:
                low, high = math.degrees(low), math.degrees(high)
            if not low <= angle <= high:
                return RESULT_LIMIT, "goal %.4f %s outside configured limits [%.4f, %.4f]" \
                       % (angle, unit, low, high), joint
        return DRIVER_OK, "", joint

    # ------------------------------------------------------------------ actions

    def _on_move_goal(self, goal_request):
        # type: (object) -> GoalResponse
        if not self.allow_motion:
            self.get_logger().warn("move goal rejected: allow_motion is false")
            return GoalResponse.REJECT
        if self.identity_error:
            self.get_logger().error("move goal rejected: %s" % self.identity_error)
            return GoalResponse.REJECT
        if self.session.unknown_command_state:
            self.get_logger().error("move goal rejected: UNKNOWN command state (%s)"
                                    % self.session.unknown_detail)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_move_cancel(self, goal_handle):
        # type: (object) -> CancelResponse
        # Cancelling is always accepted: the worker then sends STOP and reports
        # honestly whether the board confirmed it.
        return CancelResponse.ACCEPT

    def _on_move_deg(self, goal_handle):
        return self._on_move(goal_handle, degrees=True)

    def _on_move(self, goal_handle, degrees=False):
        # type: (object) -> MoveJoint.Result
        goal = goal_handle.request
        action_type = MoveJointDeg if degrees else MoveJoint
        angle = goal.angle_deg if degrees else goal.angle_rad
        unit = "deg" if degrees else "rad"
        result = action_type.Result()
        code, message, joint = self._validate_move_goal(goal, degrees=degrees)
        if joint is None or code != DRIVER_OK:
            result.motion_completed = False
            result.persisted = False
            result.position_valid = False
            result.error_code = int(code)
            result.message = message
            goal_handle.abort()
            return result

        deci = (proto.degrees_to_deci_deg(angle) if degrees else proto.rad_to_deci_deg(angle))
        commissioning = bool(self.get_parameter("commissioning").value)
        max_step_deg = float(self.get_parameter("max_commissioning_step_deg").value)

        # Only STATIC checks happen here. Everything that depends on the board's
        # current position, its stored rpm or its gear is decided by the worker from a
        # fresh reading while it holds this joint's slot, so no goal can act on a
        # position another goal has already moved away from.
        if joint.limits_rad is None and not commissioning:
            result.error_code = int(RESULT_LIMITS_UNCONFIRMED)
            result.message = (
                "%s has limits_rad: null, meaning the mechanical limits are NOT confirmed, so the "
                "driver cannot verify where this motion ends and refuses by default. Record the "
                "limits in the configuration, or start the node with commissioning:=true to accept "
                "explicitly unbounded motion (then every command is capped at %.1f deg and the log, "
                "status and diagnostics all say there is no soft-limit protection)."
                % (joint.name, max_step_deg))
            if degrees:
                result.message = result.message.replace("limits_rad", "limits_deg")
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        if commissioning and goal.mode == MOVE_MODE_RELATIVE and (abs(angle) if degrees else math.degrees(abs(angle))) > max_step_deg + 1e-9:
            result.error_code = int(RESULT_LIMIT)
            result.message = (
                "commissioning mode caps one command at %.1f deg; this goal asks for %.1f deg. "
                "Nothing was sent." % (max_step_deg, (abs(angle) if degrees else math.degrees(abs(angle)))))
            self.get_logger().warn(result.message)
            goal_handle.abort()
            return result

        if deci == 0 and goal.mode == MOVE_MODE_RELATIVE:
            # Only the trivially empty goal is caught here. The authoritative
            # quantisation check needs the gear the BOARD has stored, so it runs in the
            # worker; the YAML gear is not trusted for that decision.
            result.error_code = int(RESULT_NO_OP)
            result.motion_completed = False
            result.persisted = False
            result.position_valid = False
            setattr(result, "final_angle_deg" if degrees else "final_angle_rad", 0.0)
            result.message = ("goal is 0.0 %s, which quantises to 0 steps; nothing was sent and "
                              "no new DONE is expected") % unit
            self.get_logger().info(result.message)
            goal_handle.succeed()
            return result

        feedback_lock = threading.Lock()

        def publish_feedback(payload):
            with feedback_lock:
                if not _goal_is_active(goal_handle):
                    return
                feedback = action_type.Feedback()
                feedback.command_sequence = int(payload["sequence"])
                feedback.state = int(payload["state"])
                feedback.fault = int(payload["fault"])
                feedback.position_valid = bool(payload["position_valid"])
                feedback.brake = bool(payload["brake"])
                feedback.communication_ok = bool(payload["communication_ok"])
                if degrees:
                    feedback.current_angle_deg = proto.deci_deg_to_degrees(payload["current_angle_deci_deg"]) if payload["position_valid"] else 0.0
                    feedback.target_angle_deg = proto.deci_deg_to_degrees(payload["target_angle_deci_deg"]) if payload["position_valid"] else 0.0
                else:
                    feedback.current_angle_rad = float(payload["current_angle_rad"])
                    feedback.target_angle_rad = float(payload["target_angle_rad"])
                feedback.measured_motor_rpm = int(payload["measured_motor_rpm"])
                try:
                    goal_handle.publish_feedback(feedback)
                except Exception as error:      # pragma: no cover - defensive
                    self.get_logger().warn("feedback publish failed: %s" % error)

        request = TaskRequest(
            kind="move", channel=int(joint.channel), opcode=int(proto.Opcode.MOVE),
            argument=int(deci) if goal.mode == MOVE_MODE_RELATIVE else 0,
            mode=int(goal.mode),
            target_deci_deg=int(deci) if goal.mode == MOVE_MODE_ABSOLUTE else None,
            limits_rad=joint.limits_rad,
            allow_without_limits=commissioning,
            max_step_deci_deg=(proto.degrees_to_deci_deg(max_step_deg) if commissioning else None),
            expect_rpm=int(goal.motor_rpm),
            allow_unreferenced=bool(self.get_parameter("allow_unreferenced_relative").value),
            timeout_sec=float(goal.timeout_sec),
            ros_target_id=_goal_id(goal_handle),
            joint=joint.name, feedback=publish_feedback)
        self.get_logger().info(
            "MOVE %s channel=%d mode=%s goal=%.4f %s (%d x 0.1 deg) expected_rpm=%s "
            "timeout=%.1f s commissioning=%s%s"
            % (joint.name, joint.channel,
               "absolute" if goal.mode == MOVE_MODE_ABSOLUTE else "relative",
               angle, unit, deci,
               ("%d (must match the board)" % goal.motor_rpm) if goal.motor_rpm
               else "whatever the board has stored",
               goal.timeout_sec, commissioning,
               "" if joint.limits_rad is not None
               else " -- NO SOFT-LIMIT PROTECTION (limits unconfirmed)"))
        handle = self.worker.submit(request)
        # Watch for cancellation while the worker owns the bus.
        while not handle.done():
            if _goal_is_canceling(goal_handle):
                handle.cancel()
            handle.wait(0.1)

        task_result = handle.result
        result.motion_completed = bool(task_result.completed)
        result.persisted = bool(task_result.persisted)
        result.position_valid = bool(task_result.position_valid)
        result.error_code = int(task_result.result_code)
        if degrees:
            result.final_angle_deg = proto.deci_deg_to_degrees(task_result.final_angle_deci_deg) if task_result.position_valid else 0.0
        else:
            result.final_angle_rad = float(task_result.final_angle_rad)
        result.message = task_result.message or DRIVER_RESULT_NAMES.get(
            int(task_result.result_code), "")
        if task_result.detail:
            # Carries caveats the caller must not miss, e.g. that no soft-limit check
            # was possible because the mechanical limits are unconfirmed.
            result.message = ("%s | %s" % (result.message, task_result.detail)
                              if result.message else task_result.detail)
        if degrees:
            result.message = result.message.replace("limits_rad", "limits_deg")
        if task_result.cancelled:
            result.message = ("cancelled; " + result.message) if result.message else "cancelled"
            if task_result.stop_confirmed:
                goal_handle.canceled()
            else:
                # The mechanism may still be moving, so this is not a clean cancel.
                goal_handle.abort()
            return result
        if task_result.unknown:
            goal_handle.abort()
            return result
        if task_result.completed and task_result.result_code in (DRIVER_OK, 0):
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _on_selftest(self, goal_handle):
        # type: (object) -> RunSelfTest.Result
        goal = goal_handle.request
        result = RunSelfTest.Result()
        scope_all = bool(goal.scope == RunSelfTest.Goal.ALL)
        if not scope_all:
            joint = self._resolve_joint(goal.joint_name)
            if joint is None:
                result.error_code = int(RESULT_GOAL_INVALID)
                result.message = self._unknown_joint_text(goal.joint_name)
                goal_handle.abort()
                return result
            channel = int(joint.channel)
        else:
            channel = 0
        if goal.direction not in (0, 1):
            result.error_code = int(RESULT_GOAL_INVALID)
            result.message = "direction must be 0 (positive) or 1 (negative)"
            goal_handle.abort()
            return result
        if not MIN_TIMEOUT_SEC <= float(goal.timeout_sec) <= MAX_TIMEOUT_SEC:
            result.error_code = int(RESULT_GOAL_INVALID)
            result.message = "timeout_sec must be %.1f..%.1f" % (MIN_TIMEOUT_SEC, MAX_TIMEOUT_SEC)
            goal_handle.abort()
            return result

        def publish_feedback(payload):
            if not _goal_is_active(goal_handle):
                return
            feedback = RunSelfTest.Feedback()
            feedback.command_sequence = int(payload["sequence"])
            # uint8 has no room for -1, so 255 is the "not applicable" sentinel.
            active = int(payload.get("active_channel", -1))
            feedback.active_channel = active if 0 <= active < 255 else 255
            feedback.phase = int(payload["phase"])
            feedback.state = int(payload["state"])
            feedback.fault = int(payload["fault"])
            feedback.powered_ms = int(payload["powered_ms"])
            feedback.return_ms = int(payload["return_ms"])
            feedback.communication_ok = bool(payload["communication_ok"])
            try:
                goal_handle.publish_feedback(feedback)
            except Exception as error:      # pragma: no cover - defensive
                self.get_logger().warn("feedback publish failed: %s" % error)

        opcode = proto.Opcode.SELFTEST_ALL if scope_all else proto.Opcode.SELFTEST
        request = TaskRequest(kind="selftest", channel=channel, opcode=int(opcode),
                              argument=int(goal.direction), timeout_sec=float(goal.timeout_sec),
                              ros_target_id=_goal_id(goal_handle), scope_all=scope_all,
                              feedback=publish_feedback)
        self.get_logger().warn(
            "SELFTEST %s: this MOVES the mechanism briefly. It was requested explicitly and is "
            "never started automatically." % ("all channels" if scope_all else goal.joint_name))
        handle = self.worker.submit(request)
        while not handle.done():
            if _goal_is_canceling(goal_handle):
                handle.cancel()
            handle.wait(0.1)
        task_result = handle.result
        result.completed = bool(task_result.completed)
        result.persisted = bool(task_result.persisted)
        result.error_code = int(task_result.result_code)
        result.message = task_result.message
        record = task_result.selftest
        if record is not None:
            result.selftest_state = int(record.state)
            result.selftest_reason = int(record.reason)
            result.phase = int(record.phase)
            result.outbound_pulses = int(record.outbound_pulses)
            result.return_pulses = int(record.return_pulses)
            result.residual_pulses = int(record.residual_pulses)
        for entry in task_result.channel_results:
            message = SelftestChannel()
            message.channel = int(entry["channel"])
            message.executed = bool(entry["executed"])
            message.sequence = int(entry["sequence"])
            message.state = int(entry["state"])
            message.reason = int(entry["reason"])
            message.phase = int(entry["phase"])
            message.outbound_pulses = int(entry["outbound_pulses"])
            message.return_pulses = int(entry["return_pulses"])
            message.residual_pulses = int(entry["residual_pulses"])
            message.powered_ms = int(entry["powered_ms"])
            message.return_ms = int(entry["return_ms"])
            message.position_valid = bool(entry["position_valid"])
            message.fault = int(entry["fault"])
            result.channels.append(message)
        result.completed = bool(task_result.completed)
        if scope_all:
            # A batch that ran to the end is "completed" even when some channels failed;
            # all_passed is the only field that says everything is good. A batch that was
            # cut short (channels left UNTESTED) is NOT completed.
            result.batch_completed = bool(task_result.batch_completed)
            result.all_passed = bool(task_result.all_passed)
            result.passed = bool(task_result.all_passed)
        else:
            result.batch_completed = False
            result.all_passed = bool(task_result.passed)
            result.passed = bool(task_result.passed)
        if result.completed and not task_result.unknown:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ---------------------------------------------------------------- services

    def _finish(self, handle, response, timeout):
        # type: (object, object, float) -> object
        handle.wait(timeout)
        task_result = handle.result
        response.accepted = bool(task_result.accepted)
        response.sequence = int(task_result.sequence)
        response.result_code = int(task_result.result_code)
        response.message = task_result.message or task_result.result_name
        return response

    def _on_set_direction(self, request, response):
        from .board_session import DRIVER_BAD_REQUEST, DRIVER_IDENTITY, DRIVER_UNKNOWN
        joint = self._resolve_joint(request.joint_name)
        if joint is None:
            response.result_code = DRIVER_BAD_REQUEST
            response.message = self._unknown_joint_text(request.joint_name)
            return response
        if self.session.read_only:
            response.result_code = DRIVER_READONLY
            response.message = "read-only refuses DIR output"
            return response
        if self.identity_error:
            response.result_code = DRIVER_IDENTITY
            response.message = "identity refused: " + self.identity_error
            return response
        handle = self.worker.submit(TaskRequest(kind="direction", channel=int(joint.channel),
            opcode=int(proto.Opcode.DIRECTION), argument=int(request.high), timeout_sec=5.0,
            ros_target_id="service:/robot/set_direction"))
        if not handle.wait(7.0):
            response.unknown = True
            response.result_code = DRIVER_UNKNOWN
            response.message = "DIR outcome unknown; do not automatically toggle again"
            return response
        self._finish(handle, response, 0.0)
        for field in ("direction", "direction_confirmed", "unknown"):
            setattr(response, field, bool(getattr(handle.result, field)))
        return response

    def _on_retry_save(self, request, response):
        from .board_session import DRIVER_BAD_REQUEST, DRIVER_IDENTITY
        if not request.confirm:
            response.result_code = DRIVER_BAD_REQUEST
            response.message = "explicit confirm=true required; no SAVE sent"
            return response
        if self.session.read_only:
            response.result_code = DRIVER_READONLY
            response.message = "read-only refuses SAVE; use maintenance or save_recovery_only"
            return response
        if self.identity_error:
            response.result_code = DRIVER_IDENTITY
            response.message = "identity refused: " + self.identity_error
            return response
        handle = self.worker.submit(TaskRequest(kind="retry_save", channel=0,
            opcode=int(proto.Opcode.SAVE), timeout_sec=5.0,
            ros_target_id="service:/robot/retry_save"))
        if not handle.wait(7.0):
            from .board_session import DRIVER_UNKNOWN
            response.unknown = True
            response.result_code = DRIVER_UNKNOWN
            response.message = "SAVE result not available yet; outcome unknown; do not retry automatically"
            return response
        self._finish(handle, response, 0.0)
        for field in ("persisted", "flags_valid", "dirty", "storage_error", "pending_save", "unknown"):
            setattr(response, field, bool(getattr(handle.result, field)))
        return response

    def _on_stop(self, request, response):
        # type: (Stop.Request, Stop.Response) -> Stop.Response
        channel = None
        if request.joint_name:
            joint = self._resolve_joint(request.joint_name)
            if joint is None:
                response.accepted = False
                response.message = self._unknown_joint_text(request.joint_name)
                return response
            channel = int(joint.channel)
        if self.session.read_only:
            response.accepted = False
            response.stop_confirmed = False
            response.unknown = False
            response.message = ("read-only mode refuses STOP: the driver cannot write to the "
                                "board. Use the hardware e-stop, or restart with maintenance:=true.")
            self.get_logger().error(response.message)
            return response
        handle = self.worker.request_stop(channel, ros_target_id="service:/robot/stop")
        self._finish(handle, response, 5.0)
        response.stop_confirmed = bool(handle.result.stop_confirmed)
        response.unknown = bool(handle.result.unknown)
        if not response.stop_confirmed and not response.unknown:
            response.message = ("STOP was accepted but the board did NOT confirm the channel "
                                "settled; do not treat the mechanism as stopped")
        return response

    def _maintenance_command(self, request, response, kind, opcode, channel, argument=0,
                             timeout=5.0):
        if self.session.read_only:
            response.accepted = False
            response.message = ("read-only mode refuses %s; restart with maintenance:=true or "
                                "allow_motion:=true" % kind)
            return response
        if self.identity_error:
            response.accepted = False
            response.message = "refused: %s" % self.identity_error
            return response
        handle = self.worker.submit(TaskRequest(kind=kind, channel=channel, opcode=int(opcode),
                                                argument=int(argument), timeout_sec=timeout,
                                                ros_target_id="service:/robot/%s" % kind))
        self._finish(handle, response, timeout + 2.0)
        # Only SetZero reports persistence; the other services have no such field.
        if hasattr(response, "persisted"):
            response.persisted = bool(handle.result.persisted)
        return handle

    def _fresh_status(self, channel, timeout=3.0):
        # type: (int, float) -> Optional[object]
        """A status reading the worker took just now, or None.

        Never falls back to the published snapshot: after a write, reporting an old
        valid/fault bit as the current truth is worse than reporting nothing.
        """
        snapshot = self.worker.refresh_now(timeout=timeout)
        if snapshot is None:
            return None
        return snapshot.channel_status(channel)

    def _on_set_manual_adjust(self, request, response):
        joint = self._resolve_joint(request.joint_name)
        if joint is None:
            response.message = self._unknown_joint_text(request.joint_name)
            return response
        handle = self._maintenance_command(request, response, "manual_adjust",
                                           proto.Opcode.MANUAL_ADJUST, int(joint.channel),
                                           int(request.enable), timeout=5.0)
        if hasattr(handle, "result"):
            response.active = handle.result.manual_mode == 1
            response.exiting = handle.result.manual_mode == 2
            response.position_valid = handle.result.position_valid
        return response

    def _on_clear_fault(self, request, response):
        # type: (ClearFault.Request, ClearFault.Response) -> ClearFault.Response
        joint = self._resolve_joint(request.joint_name)
        if joint is None:
            response.accepted = False
            response.message = self._unknown_joint_text(request.joint_name)
            return response
        self._maintenance_command(request, response, "clear_fault", proto.Opcode.CLEAR_FAULT,
                                  int(joint.channel))
        status = self._fresh_status(joint.channel)
        if status is None:
            response.message += ("; the state after CLEAR_FAULT could NOT be read back, so the "
                                 "fault status is unknown -- do not assume it was cleared")
        else:
            response.fault_after = int(status.fault)
            response.position_valid = bool(status.valid)
        response.message += ("; note that CLEAR_FAULT does not restore position_valid, a lost "
                             "zero still needs /robot/set_zero")
        return response

    def _on_set_zero(self, request, response):
        # type: (SetZero.Request, SetZero.Response) -> SetZero.Response
        joint = self._resolve_joint(request.joint_name)
        if joint is None:
            response.accepted = False
            response.message = self._unknown_joint_text(request.joint_name)
            return response
        self.get_logger().warn(
            "SET_ZERO %s: the CURRENT mechanical pose becomes the zero reference and is written "
            "to flash. This was requested by an operator." % joint.name)
        self._maintenance_command(request, response, "zero", proto.Opcode.ZERO, int(joint.channel))
        status = self._fresh_status(joint.channel)
        if status is None:
            response.position_valid = False
            response.message += ("; position_valid could NOT be read back after ZERO, so the new "
                                 "zero reference is unconfirmed")
        else:
            response.position_valid = bool(status.valid)
            if not status.valid:
                response.message += "; the board did NOT report position_valid after ZERO"
        return response

    def _on_set_brake(self, request, response):
        # type: (SetBrake.Request, SetBrake.Response) -> SetBrake.Response
        joint = self._resolve_joint(request.joint_name)
        if joint is None:
            response.accepted = False
            response.message = self._unknown_joint_text(request.joint_name)
            return response
        self._maintenance_command(request, response, "brake", proto.Opcode.BRAKE,
                                  int(joint.channel), 1 if request.engage else 0)
        status = self._fresh_status(joint.channel)
        if status is None:
            response.message += "; the brake bit could NOT be read back, so its state is unknown"
        else:
            response.brake = bool(status.brake)
        return response

    def _on_apply_config(self, request, response):
        # type: (ApplyConfig.Request, ApplyConfig.Response) -> ApplyConfig.Response
        if self.session.read_only:
            response.accepted = False
            response.message = "read-only mode refuses configuration changes"
            return response
        if self.identity_error:
            response.accepted = False
            response.message = "refused: %s" % self.identity_error
            return response
        if self.save_recovery_only:
            response.accepted = False
            response.result_code = DRIVER_READONLY
            response.message = "SAVE recovery only refuses ApplyConfig"
            return response
        # The APPLIED configuration, read freshly, is the baseline. An empty request
        # must preserve exactly what the board holds; falling back to the YAML file
        # would silently rewrite a working arm's role, gear or rpm.
        snapshot = self.worker.refresh_now()
        if snapshot is None or len(snapshot.configs) < proto.CHANNEL_COUNT:
            response.accepted = False
            response.changed = False
            response.message = ("refused: the applied configuration could not be read back "
                                "freshly, so nothing was staged -- the driver will not guess what "
                                "the board currently holds")
            return response
        current = list(snapshot.configs[:proto.CHANNEL_COUNT])

        requested = {}    # type: Dict[int, object]
        for item in request.channels:
            index = int(item.channel)
            if not 0 <= index < proto.CHANNEL_COUNT:
                response.accepted = False
                response.changed = False
                response.message = ("channel %d is outside 0..3; this field is the PROTOCOL "
                                    "channel, not the 1..4 numbering used in YAML" % index)
                return response
            requested[index] = proto.ChannelConfigValues(
                enabled=bool(item.enabled), role=int(item.role), gear=int(item.gear_ratio),
                inverted=int(item.inverted), rpm=int(item.rpm))

        staged = []    # type: List[proto.ChannelConfigValues]
        for channel in range(proto.CHANNEL_COUNT):
            values = requested.get(channel)
            if values is None:
                values = proto.ChannelConfigValues(
                    enabled=bool(current[channel].enabled), role=int(current[channel].role),
                    gear=int(current[channel].gear), inverted=int(current[channel].inverted),
                    rpm=int(current[channel].rpm))
            staged.append(values)

        # Board-level fields stay exactly as the board has them: CONFIG applies pwm_hz
        # immediately but protocol/address/node only after a power cycle, so rewriting
        # them here could strand the board on another address.
        active = snapshot.info
        active_board = [int(active.protocol), int(active.address), int(active.node),
                        int(active.pwm_hz)] if active is not None else None
        staged_board = list(active_board) if active_board else \
            [0, int(self.board.address), int(self.board.node), int(self.board.pwm_hz)]

        # Validate with the firmware's own rules, including the two that are easy to get
        # wrong: rpm/gear ranges apply to disabled channels too, and role uniqueness
        # includes disabled channels that still hold a non-zero role.
        reasons = proto.validate_board_image(staged, staged_board)
        if reasons:
            response.accepted = False
            response.changed = False
            response.message = ("configuration rejected before anything was written (rules mirror "
                                "robot_config_valid, BSP/robot.c:11-23): " + "; ".join(reasons))
            self.get_logger().error(response.message)
            return response

        changed = [channel for channel in range(proto.CHANNEL_COUNT)
                   if staged[channel].to_registers() != current[channel].to_registers()]
        board_changed = bool(active_board) and staged_board != active_board
        response.changed = bool(changed or board_changed)
        for channel in range(proto.CHANNEL_COUNT):
            response.staged.append(_channel_config_message(channel, staged[channel]))
            response.applied.append(_channel_config_message(channel, current[channel]))

        if not response.changed:
            response.accepted = True
            response.message = ("nothing differs from the applied configuration, so CONFIG was NOT "
                                "sent and no flash was written")
            self.get_logger().info(response.message)
            return response

        self.get_logger().warn(
            "APPLY_CONFIG commit=%s changing channels %s%s: CONFIG validates the whole staged area "
            "and writes flash, and requires the ENTIRE board to be quiet. Changing gear, role or "
            "inverted clears that channel's zero reference and self-test record."
            % (request.commit, changed, " and board-level fields" if board_changed else ""))
        handle = self.worker.submit(TaskRequest(
            kind="config", opcode=int(proto.Opcode.CONFIG), staged_configs=staged,
            staged_board=staged_board, commit=bool(request.commit), timeout_sec=10.0,
            ros_target_id="service:/robot/apply_config"))
        self._finish(handle, response, 12.0)
        response.changed = bool(changed or board_changed)
        response.persisted = bool(handle.result.persisted)
        if handle.result.staged_configs:
            del response.staged[:]
            for channel, values in enumerate(handle.result.staged_configs):
                response.staged.append(_channel_config_message(channel, values))
        if handle.result.applied_configs:
            del response.applied[:]
            for channel, values in enumerate(handle.result.applied_configs):
                response.applied.append(_channel_config_message(channel, values))
        elif handle.result.accepted:
            response.message += ("; the applied area could NOT be read back, so it is unverified")
        return response

    def _on_acknowledge_unknown(self, request, response):
        # type: (Trigger.Request, Trigger.Response) -> Trigger.Response
        if not self.session.unknown_command_state:
            response.success = True
            response.message = "no UNKNOWN state to acknowledge"
            return response
        detail = self.session.acknowledge_unknown()
        response.success = True
        response.message = ("acknowledged: %s. Nothing was resumed automatically; verify the "
                            "mechanism by hand before sending a new motion." % detail)
        self.get_logger().warn(response.message)
        return response

    def _on_board_identity(self, request, response):
        # type: (Trigger.Request, Trigger.Response) -> Trigger.Response
        snapshot = self.worker.snapshot
        response.success = bool(snapshot.uid) and not self.identity_error
        response.message = ("uid=%s command_protocol=%d expected=%s error=%s"
                            % (snapshot.uid or "unknown", snapshot.command_protocol,
                               self.board.uid, self.identity_error or "none"))
        return response

    # ------------------------------------------------------------------ logging

    def _log(self, level, message):
        # type: (str, str) -> None
        logger = self.get_logger()
        if level == "error":
            logger.error(message)
        elif level == "warn":
            logger.warn(message)
        else:
            logger.info(message)

    # ----------------------------------------------------------------- shutdown

    def destroy_node(self):
        # type: () -> None
        try:
            if hasattr(self, "worker"):
                self.worker.stop(best_effort_stop_all=bool(self.allow_motion))
            if hasattr(self, "transport"):
                self.transport.close()
        finally:
            super().destroy_node()


def _goal_id(goal_handle):
    # type: (object) -> str
    """Stable text identity for one ROS goal, used in the board request log."""
    try:
        return str(bytes(goal_handle.goal_id.uuid).hex())
    except Exception:      # pragma: no cover - defensive
        return "unknown-goal"


def _channel_config_message(channel, values):
    # type: (int, proto.ChannelConfigValues) -> ChannelConfig
    message = ChannelConfig()
    message.channel = int(channel)
    message.enabled = bool(values.enabled)
    message.role = int(values.role)
    message.gear_ratio = int(values.gear)
    message.inverted = int(values.inverted)
    message.rpm = int(values.rpm)
    return message


def _summarise(config):
    # type: (object) -> str
    try:
        from .config import summarise
        return summarise(config)
    except Exception:      # pragma: no cover - summarise is cosmetic
        return "%d board(s), %d joint(s)" % (len(config.boards), len(config.joints))


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = None
    try:
        node = RobotMotorNode()
        executor.add_node(node)
        executor.spin()
    except (ConfigError, TransportNotAvailable) as error:
        if node is not None:
            node.get_logger().error("startup refused: %s" % error)
        else:
            print("startup refused: %s" % error)
        raise SystemExit(2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
