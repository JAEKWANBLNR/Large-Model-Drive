#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_yolobot_gazebo = get_package_share_directory('yolobot_gazebo')
    pkg_yolobot_description = get_package_share_directory('yolobot_description')
    pkg_yolobot_recognition = get_package_share_directory('yolobot_recognition')


    # gazebo와 house.world 실행 
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_gazebo, 'launch', 'start_world_launch.py'),
        ),
    )

    #yolobot 생성 
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_description, 'launch', 'spawn_yolobot_launch.launch.py'),
        )
    )     

    #yolo inference 가져오기 
    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_recognition, 'launch', 'launch_yolov8.launch.py'),
        )
    )




    # ROS-LLM nodes
    llm_audio_input_node = Node(
        package="llm_input",
        executable="llm_audio_input",
        name="llm_audio_input",
        output="screen",
    )

    llm_chatgpt_node = Node(
        package="llm_model",
        executable="chatgpt",
        name="chatgpt",
        output="screen",
    )

    llm_audio_output_node = Node(
        package="llm_output",
        executable="llm_audio_output",
        name="llm_audio_output",
        output="screen",
    )

    llm_yolobot_controller_node = Node(
        package="llm_robot",
        executable="turtle_robot",
        name="yolobot_controller",
        output="screen",
    )

    return LaunchDescription([
        start_world,
        spawn_robot_world,
        spawn_yolo,
        llm_audio_input_node,
        llm_chatgpt_node,
        llm_audio_output_node,
        llm_yolobot_controller_node,
    ])
