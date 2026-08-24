"""Launch the simulation, perception, LLM, and mobile robot tool server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the complete LLM, robot, audio, vision, and simulation launch."""
    gazebo_share = get_package_share_directory("yolobot_gazebo")
    description_share = get_package_share_directory("yolobot_description")
    recognition_share = get_package_share_directory("yolobot_recognition")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    enable_recognition = LaunchConfiguration("enable_recognition")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_audio_input = LaunchConfiguration("enable_audio_input")
    enable_audio_output = LaunchConfiguration("enable_audio_output")
    audio_input_executable = LaunchConfiguration("audio_input_executable")

    default_world = os.path.join(gazebo_share, "worlds", "testhouse.world")
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "start_world_launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_share,
                "launch",
                "spawn_yolobot_launch.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(recognition_share, "launch", "launch_yolov8.launch.py")
        ),
        condition=IfCondition(enable_recognition),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "enable_depth": enable_depth,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("enable_recognition", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("enable_audio_input", default_value="false"),
            DeclareLaunchArgument("enable_audio_output", default_value="false"),
            DeclareLaunchArgument(
                "audio_input_executable",
                default_value="llm_audio_input_local",
                description=("Use llm_audio_input_local or llm_audio_input for AWS."),
            ),
            start_world,
            spawn_robot,
            recognition,
            Node(
                package="llm_model",
                executable="chatgpt",
                name="llm_model",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="llm_robot",
                executable="turtle_robot",
                name="yolobot_controller",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="llm_input",
                executable=audio_input_executable,
                output="screen",
                condition=IfCondition(enable_audio_input),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="llm_output",
                executable="llm_audio_output",
                output="screen",
                condition=IfCondition(enable_audio_output),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
