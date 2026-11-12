"""VRHAND-compatible entry point: MoveIt/RViz using existing controllers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ruka3_moveit_config'), 'launch', 'demo.launch.py'])),
            launch_arguments={
                'start_control': 'false',
                'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
                'can_interface': LaunchConfiguration('can_interface'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items()),
    ])
