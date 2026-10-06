"""Convenience launch: HTS bridge + JAKA dual-arm rc_ctrl in one command.

Requires both ``hand_tracking_robot`` and ``jaka_driver`` to be available on the
ROS package path (source both workspace install/setup.bash files first).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_left_ip_arg = DeclareLaunchArgument(
        'robot_left_ip', default_value='192.168.71.37')
    robot_right_ip_arg = DeclareLaunchArgument(
        'robot_right_ip', default_value='192.168.71.36')
    use_rpy_ctrl_arg = DeclareLaunchArgument(
        'use_rpy_ctrl', default_value='false',
        description='false = translation-only hand control')
    enable_y_limit_arg = DeclareLaunchArgument(
        'enable_y_limit', default_value='true',
        description='Keep the two arms from crossing the shared center line')

    protocol_arg = DeclareLaunchArgument('protocol', default_value='udp')
    host_arg = DeclareLaunchArgument('host', default_value='0.0.0.0')
    port_arg = DeclareLaunchArgument('port', default_value='9000')
    publish_rate_arg = DeclareLaunchArgument('publish_rate', default_value='100.0')
    motion_scale_arg = DeclareLaunchArgument('motion_scale', default_value='1.0')
    pinch_threshold_arg = DeclareLaunchArgument('pinch_threshold', default_value='0.035')
    position_source_arg = DeclareLaunchArgument('position_source', default_value='wrist')
    tf_translation_arg = DeclareLaunchArgument('tf_translation', default_value='[0.0, 0.0, 0.0]')
    tf_rotation_arg = DeclareLaunchArgument('tf_rotation', default_value='[1.0, 0.0, 0.0, 0.0]')

    rc_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('jaka_driver'), 'launch', 'RC_ctrl.launch.py'])
        ]),
        launch_arguments={
            'robot_left_ip': LaunchConfiguration('robot_left_ip'),
            'robot_right_ip': LaunchConfiguration('robot_right_ip'),
            'use_rpy_ctrl': LaunchConfiguration('use_rpy_ctrl'),
            'enable_y_limit': LaunchConfiguration('enable_y_limit'),
        }.items(),
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('hand_tracking_robot'), 'launch', 'hand_tracking_bridge.launch.py'])
        ]),
        launch_arguments={
            'protocol': LaunchConfiguration('protocol'),
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'motion_scale': LaunchConfiguration('motion_scale'),
            'pinch_threshold': LaunchConfiguration('pinch_threshold'),
            'position_source': LaunchConfiguration('position_source'),
            'tf_translation': LaunchConfiguration('tf_translation'),
            'tf_rotation': LaunchConfiguration('tf_rotation'),
        }.items(),
    )

    return LaunchDescription([
        robot_left_ip_arg,
        robot_right_ip_arg,
        use_rpy_ctrl_arg,
        enable_y_limit_arg,
        protocol_arg,
        host_arg,
        port_arg,
        publish_rate_arg,
        motion_scale_arg,
        pinch_threshold_arg,
        position_source_arg,
        tf_translation_arg,
        tf_rotation_arg,
        rc_ctrl_launch,
        bridge_launch,
    ])
