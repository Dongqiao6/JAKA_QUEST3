"""Convenience launch: HTS bridge + JAKA dual-arm rc_ctrl in one command.

``hand_tracking_robot`` and ``jaka_driver`` are both part of this workspace, so
only ``source install/setup.bash`` is required before running this file.
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
    servo_tran_deadband_arg = DeclareLaunchArgument(
        'servo_tran_deadband', default_value='0.25')
    servo_rot_deadband_arg = DeclareLaunchArgument(
        'servo_rot_deadband', default_value='0.005')
    servo_max_tran_step_arg = DeclareLaunchArgument(
        'servo_max_tran_step', default_value='0.75')
    servo_max_rot_step_arg = DeclareLaunchArgument(
        'servo_max_rot_step', default_value='0.005')
    max_cart_vel_arg = DeclareLaunchArgument(
        'max_cart_vel', default_value='80.0')
    max_cart_acc_arg = DeclareLaunchArgument(
        'max_cart_acc', default_value='80.0')
    max_cart_jerk_arg = DeclareLaunchArgument(
        'max_cart_jerk', default_value='160.0')
    max_rot_vel_arg = DeclareLaunchArgument(
        'max_rot_vel', default_value='20.0')
    max_rot_acc_arg = DeclareLaunchArgument(
        'max_rot_acc', default_value='40.0')
    max_rot_jerk_arg = DeclareLaunchArgument(
        'max_rot_jerk', default_value='80.0')
    servo_filter_mode_arg = DeclareLaunchArgument(
        'servo_filter_mode', default_value='joint')
    max_joint_vel_arg = DeclareLaunchArgument(
        'max_joint_vel', default_value='40.0')
    max_joint_acc_arg = DeclareLaunchArgument(
        'max_joint_acc', default_value='80.0')
    max_joint_jerk_arg = DeclareLaunchArgument(
        'max_joint_jerk', default_value='160.0')

    protocol_arg = DeclareLaunchArgument('protocol', default_value='udp')
    host_arg = DeclareLaunchArgument('host', default_value='0.0.0.0')
    port_arg = DeclareLaunchArgument('port', default_value='9000')
    publish_rate_arg = DeclareLaunchArgument('publish_rate', default_value='100.0')
    smooth_alpha_arg = DeclareLaunchArgument('smooth_alpha', default_value='0.45')
    motion_scale_arg = DeclareLaunchArgument('motion_scale', default_value='0.40')
    pinch_threshold_arg = DeclareLaunchArgument('pinch_threshold', default_value='0.035')
    position_source_arg = DeclareLaunchArgument('position_source', default_value='wrist')
    mapping_mode_arg = DeclareLaunchArgument('mapping_mode', default_value='world_wrist')
    head_timeout_arg = DeclareLaunchArgument('head_timeout', default_value='1.0')
    enable_left_arg = DeclareLaunchArgument('enable_left', default_value='true')
    enable_right_arg = DeclareLaunchArgument('enable_right', default_value='true')
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
            'servo_tran_deadband': LaunchConfiguration('servo_tran_deadband'),
            'servo_rot_deadband': LaunchConfiguration('servo_rot_deadband'),
            'servo_max_tran_step': LaunchConfiguration('servo_max_tran_step'),
            'servo_max_rot_step': LaunchConfiguration('servo_max_rot_step'),
            'max_cart_vel': LaunchConfiguration('max_cart_vel'),
            'max_cart_acc': LaunchConfiguration('max_cart_acc'),
            'max_cart_jerk': LaunchConfiguration('max_cart_jerk'),
            'max_rot_vel': LaunchConfiguration('max_rot_vel'),
            'max_rot_acc': LaunchConfiguration('max_rot_acc'),
            'max_rot_jerk': LaunchConfiguration('max_rot_jerk'),
            'servo_filter_mode': LaunchConfiguration('servo_filter_mode'),
            'max_joint_vel': LaunchConfiguration('max_joint_vel'),
            'max_joint_acc': LaunchConfiguration('max_joint_acc'),
            'max_joint_jerk': LaunchConfiguration('max_joint_jerk'),
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
            'smooth_alpha': LaunchConfiguration('smooth_alpha'),
            'motion_scale': LaunchConfiguration('motion_scale'),
            'pinch_threshold': LaunchConfiguration('pinch_threshold'),
            'position_source': LaunchConfiguration('position_source'),
            'mapping_mode': LaunchConfiguration('mapping_mode'),
            'head_timeout': LaunchConfiguration('head_timeout'),
            'enable_left': LaunchConfiguration('enable_left'),
            'enable_right': LaunchConfiguration('enable_right'),
            'tf_translation': LaunchConfiguration('tf_translation'),
            'tf_rotation': LaunchConfiguration('tf_rotation'),
        }.items(),
    )

    return LaunchDescription([
        robot_left_ip_arg,
        robot_right_ip_arg,
        use_rpy_ctrl_arg,
        enable_y_limit_arg,
        servo_tran_deadband_arg,
        servo_rot_deadband_arg,
        servo_max_tran_step_arg,
        servo_max_rot_step_arg,
        max_cart_vel_arg,
        max_cart_acc_arg,
        max_cart_jerk_arg,
        max_rot_vel_arg,
        max_rot_acc_arg,
        max_rot_jerk_arg,
        servo_filter_mode_arg,
        max_joint_vel_arg,
        max_joint_acc_arg,
        max_joint_jerk_arg,
        protocol_arg,
        host_arg,
        port_arg,
        publish_rate_arg,
        smooth_alpha_arg,
        motion_scale_arg,
        pinch_threshold_arg,
        position_source_arg,
        mapping_mode_arg,
        head_timeout_arg,
        enable_left_arg,
        enable_right_arg,
        tf_translation_arg,
        tf_rotation_arg,
        rc_ctrl_launch,
        bridge_launch,
    ])
