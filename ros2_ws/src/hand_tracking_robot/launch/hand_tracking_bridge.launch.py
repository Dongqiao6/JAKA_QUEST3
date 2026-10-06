"""Launch only the HTS -> JAKA hand-tracking bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    protocol_arg = DeclareLaunchArgument(
        'protocol', default_value='udp',
        description="HTS transport: 'udp' (wired-recommended) or 'tcp'")
    host_arg = DeclareLaunchArgument(
        'host', default_value='0.0.0.0', description='UDP/TCP bind address')
    port_arg = DeclareLaunchArgument(
        'port', default_value='9000', description='UDP/TCP bind port')
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='100.0', description='Pose/Joy publish rate (Hz)')
    smooth_alpha_arg = DeclareLaunchArgument(
        'smooth_alpha', default_value='0.45',
        description='Position/quaternion EMA factor (0 disables filtering)')
    motion_scale_arg = DeclareLaunchArgument(
        'motion_scale', default_value='1.0',
        description='Scale applied to the hand displacement')
    pinch_threshold_arg = DeclareLaunchArgument(
        'pinch_threshold', default_value='0.035',
        description='Thumb-index distance (m) below which tracking engages')
    pinch_release_ratio_arg = DeclareLaunchArgument(
        'pinch_release_ratio', default_value='1.5',
        description='Hysteresis: release when distance exceeds threshold * ratio')
    tracking_timeout_arg = DeclareLaunchArgument(
        'tracking_timeout', default_value='1.0',
        description='Seconds without HTS data before the hand is considered lost')
    position_source_arg = DeclareLaunchArgument(
        'position_source', default_value='wrist',
        description="Control point: 'wrist', 'index_tip', or 'pinch_midpoint'")
    left_topic_arg = DeclareLaunchArgument(
        'left_target_topic', default_value='/rc_ctrl/left_target')
    right_topic_arg = DeclareLaunchArgument(
        'right_target_topic', default_value='/rc_ctrl/right_target')
    button_topic_arg = DeclareLaunchArgument(
        'button_topic', default_value='/rc_ctrl/button')
    enable_left_arg = DeclareLaunchArgument(
        'enable_left', default_value='true', description='Publish left-arm targets')
    enable_right_arg = DeclareLaunchArgument(
        'enable_right', default_value='true', description='Publish right-arm targets')
    unity_to_robot_arg = DeclareLaunchArgument(
        'unity_to_robot', default_value='[0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]',
        description='3x3 row-major Unity LH -> robot RH rotation')
    tf_translation_arg = DeclareLaunchArgument(
        'tf_translation', default_value='[0.0, 0.0, 0.0]',
        description='Glasses origin in robot base [x, y, z] meters')
    tf_rotation_arg = DeclareLaunchArgument(
        'tf_rotation', default_value='[1.0, 0.0, 0.0, 0.0]',
        description='Glasses orientation in robot base [w, x, y, z]')
    glasses_frame_arg = DeclareLaunchArgument(
        'glasses_frame', default_value='quest')
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame', default_value='world')

    node = Node(
        package='hand_tracking_robot',
        executable='hand_tracking_bridge',
        name='hand_tracking_bridge',
        output='screen',
        parameters=[{
            'protocol': LaunchConfiguration('protocol'),
            'host': LaunchConfiguration('host'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=float),
            'smooth_alpha': ParameterValue(LaunchConfiguration('smooth_alpha'), value_type=float),
            'motion_scale': ParameterValue(LaunchConfiguration('motion_scale'), value_type=float),
            'pinch_threshold': ParameterValue(LaunchConfiguration('pinch_threshold'), value_type=float),
            'pinch_release_ratio': ParameterValue(LaunchConfiguration('pinch_release_ratio'), value_type=float),
            'tracking_timeout': ParameterValue(LaunchConfiguration('tracking_timeout'), value_type=float),
            'position_source': LaunchConfiguration('position_source'),
            'left_target_topic': LaunchConfiguration('left_target_topic'),
            'right_target_topic': LaunchConfiguration('right_target_topic'),
            'button_topic': LaunchConfiguration('button_topic'),
            'enable_left': ParameterValue(LaunchConfiguration('enable_left'), value_type=bool),
            'enable_right': ParameterValue(LaunchConfiguration('enable_right'), value_type=bool),
            'unity_to_robot': LaunchConfiguration('unity_to_robot'),
            'tf_translation': LaunchConfiguration('tf_translation'),
            'tf_rotation': LaunchConfiguration('tf_rotation'),
            'glasses_frame': LaunchConfiguration('glasses_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        }],
    )

    return LaunchDescription([
        protocol_arg,
        host_arg,
        port_arg,
        publish_rate_arg,
        smooth_alpha_arg,
        motion_scale_arg,
        pinch_threshold_arg,
        pinch_release_ratio_arg,
        tracking_timeout_arg,
        position_source_arg,
        left_topic_arg,
        right_topic_arg,
        button_topic_arg,
        enable_left_arg,
        enable_right_arg,
        unity_to_robot_arg,
        tf_translation_arg,
        tf_rotation_arg,
        glasses_frame_arg,
        robot_base_frame_arg,
        node,
    ])
