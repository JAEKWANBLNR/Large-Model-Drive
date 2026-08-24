"""Launch Gazebo, Yolobot, and the YOLO perception stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Create the combined Gazebo world and robot spawn launch."""
    gazebo_share = get_package_share_directory("yolobot_gazebo")
    description_share = get_package_share_directory("yolobot_description")
    recognition_share = get_package_share_directory("yolobot_recognition")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    default_world = os.path.join(gazebo_share, "worlds", "testhouse.world")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "start_world_launch.py")
                ),
                launch_arguments={"world": world}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        description_share,
                        "launch",
                        "spawn_yolobot_launch.launch.py",
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        recognition_share,
                        "launch",
                        "launch_yolov8.launch.py",
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
