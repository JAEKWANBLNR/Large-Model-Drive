"""Allow-listed Cartesian target-pose tool server for an ARX5 arm."""

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from llm_interfaces.srv import ChatGPT

from .turtle_robot import _arguments_from_request


class ArmRobot(Node):
    """Publish validated arm pose targets without invoking a shell."""

    def __init__(self) -> None:
        """Initialize the target-pose publisher and tool service."""
        super().__init__("arm_robot")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("function_service", "/llm/function_call")
        self.target_pose_publisher = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("target_pose_topic").value,
            10,
        )
        self.allowed_functions = {"publish_target_pose": self.publish_target_pose}
        self.function_call_server = self.create_service(
            ChatGPT,
            self.get_parameter("function_service").value,
            self.function_call_callback,
        )
        self.get_logger().info("Arm robot tool server is ready.")

    def function_call_callback(
        self, request: ChatGPT.Request, response: ChatGPT.Response
    ) -> ChatGPT.Response:
        """Validate and execute an allow-listed arm tool request."""
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

    def publish_target_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> dict[str, Any]:
        """Publish a bounded Cartesian pose as a numeric ROS message."""
        pose = [float(value) for value in (x, y, z, roll, pitch, yaw)]
        if not all(-10.0 <= value <= 10.0 for value in pose):
            raise ValueError("Pose components must be between -10 and 10.")
        message = Float64MultiArray()
        message.data = pose
        self.target_pose_publisher.publish(message)
        self.get_logger().info(f"Published arm target pose: {pose}")
        return {"target_pose": pose}


def main(args: list[str] | None = None) -> None:
    """Run the arm tool-server ROS node."""
    rclpy.init(args=args)
    node = ArmRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
