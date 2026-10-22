"""Bring-up for the robot motor driver.

Launch arguments required by the execution plan (section 8C) are implemented
exactly as written there:

    ros2 launch robot_motor_bringup bringup.launch.py transport:=fake allow_motion:=false
    ros2 launch robot_motor_bringup bringup.launch.py transport:=modbus_rtu \
        allow_motion:=false config_file:=$HOME/robot_motor_ros2_ws/config/robot.yaml

Two safety defaults differ from a naive bring-up and are deliberate:

* ROS_DOMAIN_ID is set to a dedicated value. The target host already runs a live
  control stack on domain 0 (controller_manager, joint_state_broadcaster, several
  robot_state_publisher instances). Joining that domain with a new publisher would
  feed other people's controllers.
* publish_joint_states is false, so nothing is written to /joint_states unless an
  operator asks for it explicitly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ParameterValue moved between launch_ros modules across ROS 2 distributions, and a
# source-built Humble may ship only one of the two names. Fall back to plain
# substitutions: launch_ros evaluates dictionary parameter substitutions "according
# to yaml rules", so "false"/"5.0" still become a bool/double.
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

    transport = LaunchConfiguration('transport')
    allow_motion = LaunchConfiguration('allow_motion')
    allow_unreferenced_relative = LaunchConfiguration('allow_unreferenced_relative')
    maintenance = LaunchConfiguration('maintenance')
    save_recovery_only = LaunchConfiguration('save_recovery_only')
    commissioning = LaunchConfiguration('commissioning')
    max_commissioning_step_deg = LaunchConfiguration('max_commissioning_step_deg')
    board_name = LaunchConfiguration('board_name')
    board_namespace = LaunchConfiguration('board_namespace')
    config_file = LaunchConfiguration('config_file')
    polling_hz = LaunchConfiguration('polling_hz')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    require_uid = LaunchConfiguration('require_uid')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    start_driver = LaunchConfiguration('start_driver')
    log_level = LaunchConfiguration('log_level')

    declarations = [
        DeclareLaunchArgument("save_recovery_only", default_value="false", description="Restricted recovery: only STOP and SAVE writes, no motion/zero/config/manual."),
        DeclareLaunchArgument("allowed_motion_joints", default_value="*", description="Comma-separated movable joints; empty denies all; * allows all mapped joints."),
        DeclareLaunchArgument(
            'transport', default_value='',
            description='fake | modbus_rtu | can. Empty means use the value from the '
                        'configuration file. "can" refuses to start until python-can '
                        'exists and hardware evidence is recorded.'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='false keeps the driver read-only: every write is intercepted, '
                        'including STOP. Motion goals are refused.'),
        DeclareLaunchArgument(
            'allow_unreferenced_relative', default_value='false',
            description='Allow a RELATIVE move while position_valid is false. The '
                        'firmware supports unreferenced relative motion; enabling this '
                        'is an explicit decision, and it never triggers an automatic '
                        'zero. Absolute moves stay refused without a valid zero.'),
        DeclareLaunchArgument(
            'maintenance', default_value='false',
            description='Explicit maintenance entry: allows stop, clear_fault, set_zero, '
                        'set_brake, save and apply_config while still refusing motion.'),
        DeclareLaunchArgument(
            'commissioning', default_value='false',
            description='Debug entry for joints whose limits_rad is null. While false, motion '
                        'on such a joint is refused because the driver cannot verify where it '
                        'would end. While true, motion is allowed with NO soft-limit '
                        'protection, each command is capped at max_commissioning_step_deg, and '
                        'the log, /robot/motor_states and diagnostics all say so.'),
        DeclareLaunchArgument(
            'max_commissioning_step_deg', default_value='10.0',
            description='Ceiling on a single command while commissioning, in degrees.'),
        DeclareLaunchArgument(
            'board_name', default_value='',
            description='Which board of the configuration THIS process drives. Empty means "the '
                        'only one". A configuration with several boards requires it: one node '
                        'drives exactly one board, so that each board keeps its own serial owner '
                        'thread, sequence allocator and UNKNOWN state, and can carry its own '
                        'allow_motion/commissioning policy.'),
        DeclareLaunchArgument(
            'board_namespace', default_value='',
            description='ROS namespace for this board\'s interfaces. Empty keeps the historical '
                        'names (/robot/stop, /robot/motor_states, ...). Set it (e.g. board_b) when '
                        'another board node runs in the SAME domain, so the two cannot answer each '
                        'other\'s service calls. Names become /<ns>/robot/stop and so on; '
                        '/diagnostics stays shared, which is normal for that topic.'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to robot.yaml.'),
        DeclareLaunchArgument('polling_hz', default_value='5.0'),
        DeclareLaunchArgument(
            'publish_joint_states', default_value='false',
            description='Publish /joint_states for joints with a fresh, valid position. '
                        'Off by default because a live ros2_control stack on the same '
                        'domain would consume these positions.'),
        DeclareLaunchArgument('require_uid', default_value='true'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='71',
            description='Dedicated domain so this driver does not join the running '
                        'robot graph on domain 0 of the target host.'),
        DeclareLaunchArgument(
            'start_driver', default_value='true',
            description='Set to false to only export the environment (useful for '
                        'composing this file into a larger bring-up).'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    # Launch substitutions are strings; without an explicit value_type a declared
    # bool/double parameter would be rejected with InvalidParameterTypeException.
    driver = Node(
        package='robot_motor_driver',
        executable='robot_motor_node',
        name='robot_motor_driver',
        # An empty namespace keeps the historical absolute names, because the node
        # declares its interfaces with RELATIVE names ("robot/stop"). Two board nodes in
        # one domain must therefore be given distinct namespaces.
        namespace=board_namespace,
        output='screen',
        parameters=[{
            'config_file': typed(config_file, str),
            'transport': typed(transport, str),
            'allow_motion': typed(allow_motion, bool),
                'allowed_motion_joints': typed(LaunchConfiguration('allowed_motion_joints'), str),
            'allow_unreferenced_relative': typed(allow_unreferenced_relative, bool),
            'maintenance': typed(maintenance, bool),
            'save_recovery_only': typed(save_recovery_only, bool),
            'commissioning': typed(commissioning, bool),
            'max_commissioning_step_deg': typed(max_commissioning_step_deg, float),
            'polling_hz': typed(polling_hz, float),
            'publish_joint_states': typed(publish_joint_states, bool),
            'require_uid': typed(require_uid, bool),
            'board_name': typed(board_name, str),
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(start_driver),
    )

    return LaunchDescription(declarations + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        driver,
    ])
