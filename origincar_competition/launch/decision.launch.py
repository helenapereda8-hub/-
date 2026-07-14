import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 参数（数值微调）
    args = [
        DeclareLaunchArgument('cruise_speed', default_value='1.12'),
        DeclareLaunchArgument('kp_line', default_value='0.00318'),
        DeclareLaunchArgument('evade_speed', default_value='0.38'),
        DeclareLaunchArgument('cone_detect_thresh', default_value='135.0'),
        DeclareLaunchArgument('cone_critical_thresh', default_value='260.0'),
        DeclareLaunchArgument('evade_steer_gain', default_value='0.48'),
        DeclareLaunchArgument('centered_tolerance', default_value='28.0'),
        DeclareLaunchArgument('centered_bias', default_value='1.0'),
        DeclareLaunchArgument('forward_search_time', default_value='0.45'),
        DeclareLaunchArgument('recovery_time_limit', default_value='0.0'),
        DeclareLaunchArgument('recovery_speed_ratio', default_value='0.28'),
        DeclareLaunchArgument('swing_freq', default_value='0.3'),
    ]

    # 包含视觉流水线（origincar_bringup 保持不变）
    visual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('origincar_bringup'), 'launch', 'usb_websocket_display.launch.py')
        ])
    )

    # 二维码节点
    qr_node = Node(
        package="qr_decoder",
        executable="qr_decoder",
        name="qr_code_detection_node",
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # 决策节点（新包名）
    decision_node = Node(
        package="robot_decision",
        executable="robot_decision_node",
        name="decision_node",
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'cruise_speed': LaunchConfiguration('cruise_speed'),
            'kp_line': LaunchConfiguration('kp_line'),
            'evade_speed': LaunchConfiguration('evade_speed'),
            'cone_detect_thresh': LaunchConfiguration('cone_detect_thresh'),
            'cone_critical_thresh': LaunchConfiguration('cone_critical_thresh'),
            'evade_steer_gain': LaunchConfiguration('evade_steer_gain'),
            'centered_tolerance': LaunchConfiguration('centered_tolerance'),
            'centered_bias': LaunchConfiguration('centered_bias'),
            'forward_search_time': LaunchConfiguration('forward_search_time'),
            'recovery_time_limit': LaunchConfiguration('recovery_time_limit'),
            'recovery_speed_ratio': LaunchConfiguration('recovery_speed_ratio'),
            'swing_freq': LaunchConfiguration('swing_freq'),
        }]
    )

    return LaunchDescription(args + [
        visual_launch,
        qr_node,
        decision_node
    ])