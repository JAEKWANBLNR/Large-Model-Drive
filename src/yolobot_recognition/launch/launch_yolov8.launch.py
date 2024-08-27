from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='yolobot_recognition', executable='yolov8_ros2_pt.py', output='screen'),
        ExecuteProcess(
            cmd=['python3', '/home/park/llmyolo_ws/src/yolobot_recognition/scripts/yolov8_depth.py'],
            output='screen'
        ),
    ])