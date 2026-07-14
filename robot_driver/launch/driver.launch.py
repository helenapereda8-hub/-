from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():
    akmcar = LaunchConfiguration('akmcar', default='false')
    return LaunchDescription([
        DeclareLaunchArgument('akmcar', default_value='false'),
        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='robot_driver',
            executable='robot_driver_node',
            parameters=[{'use_ackermann': True}],
            remappings=[('/cmd_vel', 'cmd_vel')],
        ),
        launch_ros.actions.Node(
            condition=UnlessCondition(akmcar),
            package='robot_driver',
            executable='robot_driver_node',
            parameters=[{'use_ackermann': False}],
        ),
    ])