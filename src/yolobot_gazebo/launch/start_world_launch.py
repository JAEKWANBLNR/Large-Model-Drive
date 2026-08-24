"""Launch Gazebo with a selectable world file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Create the Gazebo launch for a selectable world file."""
    gazebo_share = get_package_share_directory("gazebo_ros")
    package_share = get_package_share_directory("yolobot_gazebo")
    default_world = os.path.join(package_share, "worlds", "testhouse.world")
    world = LaunchConfiguration("world")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Absolute path to the Gazebo world file.",
            ),
            gazebo,
        ]
    )
