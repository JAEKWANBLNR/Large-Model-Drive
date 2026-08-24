"""Launch YOLO detection and optional point-cloud distance extraction."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the configurable YOLO detector, overlay, and depth launch."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_depth = LaunchConfiguration("enable_depth")
    model_path = LaunchConfiguration("model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("model_path", default_value="yolov8n.pt"),
            Node(
                package="yolobot_recognition",
                executable="yolov8_ros2_pt.py",
                name="yolov8_detector",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "model_path": model_path}],
            ),
            Node(
                package="yolobot_recognition",
                executable="yolov8_ros2_subscriber.py",
                name="yolov8_overlay",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="yolobot_recognition",
                executable="yolov8_depth.py",
                name="depth_extractor",
                output="screen",
                condition=IfCondition(enable_depth),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
