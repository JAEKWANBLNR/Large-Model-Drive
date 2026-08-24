"""Allow-listed velocity tool server for multiple robot namespaces."""

import json
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from llm_config.user_config import UserConfig
from rclpy.node import Node

from llm_interfaces.srv import ChatGPT

from .turtle_robot import _arguments_from_request


class MultiRobotController(Node):
    """Route bounded velocity commands to configured robot namespaces."""

    def __init__(self) -> None:
        """Create publishers only for configured robot namespaces."""
        super().__init__("multi_robot_controller")
        self.config = UserConfig()
        self.declare_parameter("function_service", "/llm/function_call")
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("max_command_duration", 10.0)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_command_duration = float(
            self.get_parameter("max_command_duration").value
        )

        self.publishers = {
            name: self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            for name in self.config.multi_robots_name
        }
        self.publishers["default"] = self.create_publisher(Twist, "/cmd_vel", 10)
        self.stop_timers: dict[str, Any] = {}
        self.allowed_functions = {"publish_cmd_vel": self.publish_cmd_vel}
        self.function_call_server = self.create_service(
            ChatGPT,
            self.get_parameter("function_service").value,
            self.function_call_callback,
        )
        names = ", ".join(sorted(self.publishers))
        self.get_logger().info(f"Configured robot namespaces: {names}")

    def function_call_callback(
        self, request: ChatGPT.Request, response: ChatGPT.Response
    ) -> ChatGPT.Response:
        """Validate and execute one multi-robot tool request."""
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
        robot_name: str,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration: float,
    ) -> dict[str, Any]:
        """Publish a bounded velocity to one configured namespace."""
        normalized_name = robot_name.strip().strip("/") or "default"
        if normalized_name not in self.publishers:
            raise ValueError(f"Robot namespace is not configured: {robot_name}")

        duration = min(max(float(duration), 0.1), self.max_command_duration)
        command = Twist()
        command.linear.x = self._clamp(linear_x, self.max_linear_speed)
        command.linear.y = self._clamp(linear_y, self.max_linear_speed)
        command.angular.z = self._clamp(angular_z, self.max_angular_speed)
        self.publishers[normalized_name].publish(command)
        self._schedule_stop(normalized_name, duration)
        return {
            "robot_name": normalized_name,
            "linear_x": command.linear.x,
            "linear_y": command.linear.y,
            "angular_z": command.angular.z,
            "duration": duration,
        }

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        numeric_value = float(value)
        return min(max(numeric_value, -abs(limit)), abs(limit))

    def _schedule_stop(self, robot_name: str, duration: float) -> None:
        previous_timer = self.stop_timers.pop(robot_name, None)
        if previous_timer is not None:
            previous_timer.cancel()

        def stop_robot() -> None:
            self.publishers[robot_name].publish(Twist())
            timer = self.stop_timers.pop(robot_name, None)
            if timer is not None:
                timer.cancel()

        self.stop_timers[robot_name] = self.create_timer(duration, stop_robot)

    def destroy_node(self) -> bool:
        """Stop all configured robots before destroying the node."""
        for publisher in self.publishers.values():
            publisher.publish(Twist())
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the multi-robot tool-server ROS node."""
    rclpy.init(args=args)
    node = MultiRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
