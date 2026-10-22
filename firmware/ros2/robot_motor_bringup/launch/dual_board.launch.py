"""Two-board bring-up: one node process per board, each in its own namespace.

WHY ONE PROCESS PER BOARD (and not one node driving both)
---------------------------------------------------------
* Isolation is structural, not coded: each board gets its own serial-ownership thread,
  its own sequence allocator and dedupe cache, and its own UNKNOWN halt. One board
  hanging, losing its port or entering UNKNOWN cannot stop the other from being polled
  or stopped.
* Each board can carry its own policy, so an unconfirmed board can be kept read-only
  while the other one moves. Node-level parameters could not express that.
* A crash in one board's process does not take the other arm down.

WHAT THIS FILE DOES *NOT* PROVIDE
---------------------------------
* No cross-board coordination: no synchronised trajectories, no simultaneous arrival,
  no interpolated motion. Two independent workers on two independent UARTs.
* No atomic cross-board stop. `robot_stop_all()` only affects the board that receives
  the frame (BSP/robot.c:144-148), so "stop everything" means calling
  /<ns_a>/robot/stop AND /<ns_b>/robot/stop and checking BOTH confirmations. Never
  report "all stopped" from one board's reply.
* No aggregate success. If one board's motion fails and the other succeeds, that is a
  partial failure and must be reported as such.

Naming: the node declares its interfaces with RELATIVE names, so the namespace given
here decides the final names -- /board_a/robot/stop, /board_b/robot/motor_states, etc.
`/diagnostics` stays shared, which is normal for that topic; each board publishes its
own DiagnosticStatus entry keyed by UID.

Both boards default to allow_motion:=false (read-only). Enabling motion is a separate,
per-board decision.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

try:
    from launch_ros.parameter_descriptors import ParameterValue
except ImportError:                                   # pragma: no cover - distro variance
    try:
        from launch_ros.parameter_descriptions import ParameterValue
    except ImportError:                               # pragma: no cover - distro variance
        ParameterValue = None


def typed(substitution, value_type):
    """Force a parameter type when the distro supports it, otherwise rely on YAML inference."""
    if ParameterValue is None:                        # pragma: no cover - distro variance
        return substitution
    return ParameterValue(substitution, value_type=value_type)


def generate_launch_description():
    bringup_share = FindPackageShare('robot_motor_bringup')
    default_config = PathJoinSubstitution([bringup_share, 'config', 'robot.yaml'])

    config_file = LaunchConfiguration('config_file')
    transport = LaunchConfiguration('transport')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    polling_hz = LaunchConfiguration('polling_hz')
    require_uid = LaunchConfiguration('require_uid')
    log_level = LaunchConfiguration('log_level')

    args = {}
    for side in ('a', 'b'):
        for key in ('name', 'namespace', 'allow_motion', 'allow_unreferenced_relative',
                    'maintenance', 'commissioning', 'max_commissioning_step_deg', 'start'):
            args[(side, key)] = LaunchConfiguration('board_%s_%s' % (side, key))

    declarations = [
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='A configuration declaring BOTH boards. The shipped robot.yaml only '
                        'declares main_board, so board B will fail with a clear "board_name is '
                        'not in the configuration" error until a two-board file is provided. '
                        'Both entries must use DISTINCT by-path ports: two boards never share '
                        'one serial line, and two TTL adapters are never wired onto one bus.'),
        DeclareLaunchArgument(
            'transport', default_value='',
            description='fake | modbus_rtu | can, applied to both boards. Empty means use each '
                        'board\'s own value from the configuration file.'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='71',
            description='Dedicated domain. Domain 0 on the robot host already runs a live '
                        'control stack; joining it would feed other people\'s controllers.'),
        DeclareLaunchArgument('polling_hz', default_value='5.0'),
        DeclareLaunchArgument('require_uid', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        DeclareLaunchArgument('board_a_name', default_value='main_board'),
        DeclareLaunchArgument('board_a_namespace', default_value='board_a'),
        DeclareLaunchArgument('board_a_allowed_motion_joints', default_value='*'),
        DeclareLaunchArgument('board_b_allowed_motion_joints', default_value='*'),
        DeclareLaunchArgument('board_a_start', default_value='true'),
        DeclareLaunchArgument('board_a_allow_motion', default_value='false'),
        DeclareLaunchArgument('board_a_allow_unreferenced_relative', default_value='false'),
        DeclareLaunchArgument('board_a_maintenance', default_value='false'),
        DeclareLaunchArgument('board_a_commissioning', default_value='false'),
        DeclareLaunchArgument('board_a_max_commissioning_step_deg', default_value='10.0'),

        DeclareLaunchArgument('board_b_name', default_value='right_board'),
        DeclareLaunchArgument('board_b_namespace', default_value='board_b'),
        # Off by default: the second board is opt-in, so this file cannot start driving a
        # board nobody asked about.
        DeclareLaunchArgument('board_b_start', default_value='false'),
        DeclareLaunchArgument('board_b_allow_motion', default_value='false'),
        DeclareLaunchArgument('board_b_allow_unreferenced_relative', default_value='false'),
        DeclareLaunchArgument('board_b_maintenance', default_value='false'),
        DeclareLaunchArgument('board_b_commissioning', default_value='false'),
        DeclareLaunchArgument('board_b_max_commissioning_step_deg', default_value='10.0'),
    ]

    nodes = []
    for side in ('a', 'b'):
        nodes.append(Node(
            package='robot_motor_driver',
            executable='robot_motor_node',
            name='robot_motor_driver',
            namespace=args[(side, 'namespace')],
            output='screen',
            parameters=[{
                'config_file': typed(config_file, str),
                'transport': typed(transport, str),
                'board_name': typed(args[(side, 'name')], str),
                'allow_motion': typed(args[(side, 'allow_motion')], bool),
                'allow_unreferenced_relative': typed(args[(side, 'allow_unreferenced_relative')],
                                                     bool),
                'maintenance': typed(args[(side, 'maintenance')], bool),
                'commissioning': typed(args[(side, 'commissioning')], bool),
                'max_commissioning_step_deg': typed(
                    args[(side, 'max_commissioning_step_deg')], float),
                'polling_hz': typed(polling_hz, float),
                'require_uid': typed(require_uid, bool),
                'publish_joint_states': False,
                'allowed_motion_joints': typed(LaunchConfiguration('board_' + side + '_allowed_motion_joints'), str),

            }],
            arguments=['--ros-args', '--log-level', log_level],
            condition=IfCondition(args[(side, 'start')]),
        ))

    return LaunchDescription(declarations + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
    ] + nodes)
