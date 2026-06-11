from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="/loclite_livox.yaml",
        ),
        Node(
            package="hikari_loclite",
            executable="run_loclite_online",
            name="hikari_loclite",
            output="screen",
            parameters=[{"config_path": config}],
        ),
    ])
