from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    ld = LaunchDescription()

    elkapod_motion_manager_dir = get_package_share_directory("elkapod_motion_manager")
    config_path = os.path.join(
        elkapod_motion_manager_dir, "config", "elkapod_motion_manager.yaml"
    )

    joy_dev = LaunchConfiguration("joy_dev")
    gamepad_model = LaunchConfiguration("gamepad_model")

    ld.add_action(DeclareLaunchArgument("joy_dev", default_value="0"))

    ld.add_action(DeclareLaunchArgument("gamepad_model", default_value="xbox-series-x"))

    ld.add_action(
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[
                {
                    "device_id": joy_dev,
                    "deadzone": 0.3,
                    "autorepeat_rate": 20.0,
                }
            ],
        ),
    )

    ld.add_action(
        Node(
            package="elkapod_gamepad",
            executable="joy_controller",
            parameters=[config_path, {"gamepad_model": gamepad_model}],
            output="screen",
            emulate_tty=True,
        )
    )

    return ld
