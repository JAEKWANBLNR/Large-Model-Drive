#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This example demonstrates simulating function calls for any robot,
# such as controlling velocity and other service commands.
# By modifying the content of this file,
# A calling interface can be created for the function calls of any robot.
# The Python script creates a ROS 2 Node
# that controls the movement of the TurtleSim
# by creating a publisher for cmd_vel messages and a client for the reset service.
# It also includes a ChatGPT function call server
# that can call various functions to control the TurtleSim
# and return the result of the function call as a string.
#
# Author: Herman Ye @Auromix

# ROS related
import rclpy
import rclpy.duration
from rclpy.node import Node
from llm_interfaces.srv import ChatGPT
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion
import tf2_ros

from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
import math
from nav2_msgs.action import NavigateToPose

# LLM related
import json
import time

from llm_config.user_config import UserConfig

# Global Initialization
config = UserConfig()


class YolobotContoller(Node):
    def __init__(self):
        super().__init__("yolobot_controller")

        # Publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Pose Publisher
        self.pose_publisher_ = self.create_publisher(Pose, '/Pose', 10)

        self.function_call_server = self.create_service(
            ChatGPT, "/ChatGPT_function_call_service", self.function_call_callback
        )
        # Node initialization log
        self.get_logger().info("YolobotController node has been initialized")

    def function_call_callback(self, request, response):
        req = json.loads(request.request_text)
        function_name = req["name"]
        function_args = json.loads(req["arguments"])
        func_obj = getattr(self, function_name)
        try:
            function_execution_result = func_obj(**function_args)
        except Exception as error:
            self.get_logger().info(f"Failed to call function: {error}")
            response.response_text = str(error)
        else:
            response.response_text = str(function_execution_result)
        return response

    def publish_cmd_vel(self, **kwargs):
        """
        Publishes cmd_vel message to control the movement of turtlesim
        """
        linear_x = kwargs.get("linear_x", 0.0)
        linear_y = kwargs.get("linear_y", 0.0)
        linear_z = kwargs.get("linear_z", 0.0)
        angular_x = kwargs.get("angular_x", 0.0)
        angular_y = kwargs.get("angular_y", 0.0)
        angular_z = kwargs.get("angular_z", 0.0)

        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = float(linear_y)
        twist_msg.linear.z = float(linear_z)
        twist_msg.angular.x = float(angular_x)
        twist_msg.angular.y = float(angular_y)
        twist_msg.angular.z = float(angular_z)

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f"Publishing cmd_vel message successfully: {twist_msg}")
        return twist_msg
    
    def rotate_angle(self,**kwargs):
        pass


    def move_distance(self, **kwargs):
        pass    
 
def main():
    rclpy.init()
    yolobot_controller = YolobotContoller()
    rclpy.spin(yolobot_controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
