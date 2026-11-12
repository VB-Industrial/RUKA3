from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    description_file = PathJoinSubstitution(
        [FindPackageShare("ruka3_control"), "urdf", "ruka3_control.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ruka3_control"), "config", "controllers.yaml"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    description_file,
                    " use_mock_hardware:=false can_interface:=",
                    can_interface,
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN CAN-FD interface (1 Mbit/s nominal, 4 Mbit/s data)",
            ),
            Node(
                package="ruka3_control",
                executable="recovery_supervisor.py",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_file],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]
    )
