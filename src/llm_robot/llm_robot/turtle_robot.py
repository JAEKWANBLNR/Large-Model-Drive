"""Bounded mobile-base tool server for model-issued motion commands."""

import json
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from llm_interfaces.srv import ChatGPT


def _arguments_from_request(request_text: str) -> tuple[str, dict[str, Any]]:
    request = json.loads(request_text)
    name = request["name"]
    arguments = request.get("arguments", {})
    if isinstance(arguments, str):
        arguments = json.loads(arguments)
    if not isinstance(arguments, dict):
        raise ValueError("Tool arguments must be a JSON object.")
    return name, arguments


class YolobotController(Node):
    """Expose only the mobile-base velocity tool over a ROS 2 service."""

    def __init__(self) -> None:
        """Configure motion bounds, publisher, and tool service."""
        super().__init__("yolobot_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("function_service", "/llm/function_call")
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("max_command_duration", 10.0)

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_command_duration = float(
            self.get_parameter("max_command_duration").value
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.function_call_server = self.create_service(
            ChatGPT,
            self.get_parameter("function_service").value,
            self.function_call_callback,
        )
        self.stop_timer = None
        self.allowed_functions = {"publish_cmd_vel": self.publish_cmd_vel}
        self.get_logger().info("Bounded mobile robot tool server is ready.")

    def function_call_callback(
        self, request: ChatGPT.Request, response: ChatGPT.Response
    ) -> ChatGPT.Response:
        """Validate and execute one allow-listed tool call."""
        try:
            name, arguments = _arguments_from_request(request.request_text)
            function = self.allowed_functions.get(name)
            if function is None:
                raise ValueError(f"Unsupported robot function: {name}")
            result = function(**arguments)
            response.response_text = json.dumps({"ok": True, **result})
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Rejected robot function call: {error}")
            response.response_text = json.dumps({"ok": False, "error": str(error)})
        return response

    def publish_cmd_vel(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration: float,
    ) -> dict[str, Any]:
        """Publish a clamped velocity and schedule a guaranteed stop."""
        duration = min(max(float(duration), 0.1), self.max_command_duration)
        command = Twist()
        command.linear.x = self._clamp(linear_x, self.max_linear_speed)
        command.linear.y = self._clamp(linear_y, self.max_linear_speed)
        command.angular.z = self._clamp(angular_z, self.max_angular_speed)
        self.cmd_vel_publisher.publish(command)
        self._schedule_stop(duration)
        result = {
            "linear_x": command.linear.x,
            "linear_y": command.linear.y,
            "angular_z": command.angular.z,
            "duration": duration,
        }
        self.get_logger().info(f"Published bounded velocity command: {result}")
        return result

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        numeric_value = float(value)
        return min(max(numeric_value, -abs(limit)), abs(limit))

    def _schedule_stop(self, duration: float) -> None:
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(duration, self._publish_stop)

    def _publish_stop(self) -> None:
        self.cmd_vel_publisher.publish(Twist())
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
        self.get_logger().info("Published automatic stop command.")

    def destroy_node(self) -> bool:
        """Publish a final stop command before node destruction."""
        self.cmd_vel_publisher.publish(Twist())
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the mobile-base tool-server ROS node."""
    rclpy.init(args=args)
    node = YolobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
