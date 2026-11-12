"""VRHAND-compatible entry point: controllers only, without MoveIt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration('use_mock_hardware')
    can = LaunchConfiguration('can_interface')
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ruka3_control'), 'launch', 'real.launch.py'])),
            launch_arguments={'can_interface': can}.items(),
            condition=UnlessCondition(mock)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ruka3_control'), 'launch', 'mock.launch.py'])),
            launch_arguments={'use_rviz': 'false'}.items(),
            condition=IfCondition(mock)),
    ])
