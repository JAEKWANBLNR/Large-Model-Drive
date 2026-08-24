"""Publish the robot description and spawn Yolobot in Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the robot-description publisher and Gazebo spawn launch."""
    package_share = get_package_share_directory("yolobot_description")
    default_model = os.path.join(package_share, "robot", "yolobot.urdf.xacro")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    entity_name = LaunchConfiguration("entity_name")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("entity_name", default_value="yolobot"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    }
                ],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    entity_name,
                ],
            ),
        ]
    )
