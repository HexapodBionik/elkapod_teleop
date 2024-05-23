from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    teleop_twist_joy_pkg_prefix = get_package_share_directory('teleop_twist_joy')
    teleop_twist_joy_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [teleop_twist_joy_pkg_prefix, '/launch/teleop-launch.py']),
        launch_arguments={"joy_config": "xbox"}.items(),
    )
    ld.add_action(teleop_twist_joy_handler_launch)

    ld.add_action(
        Node(
            package='elkapod_teleop_joy',
            executable='elkapod_joy_controller',
        )
    )

    return ld
