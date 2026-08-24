"""ROS 2 node that connects text input to the OpenAI Responses API."""

import json
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Any

import rclpy
from llm_config.user_config import UserConfig
from openai import OpenAI
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from llm_interfaces.srv import ChatGPT


def _as_json_item(item: Any) -> dict[str, Any]:
    """Convert an OpenAI SDK response item into replayable JSON data."""
    if isinstance(item, dict):
        return item
    if hasattr(item, "model_dump"):
        return item.model_dump(mode="json", exclude_none=True)
    raise TypeError(f"Unsupported OpenAI response item: {type(item)!r}")


class ChatGPTNode(Node):
    """Process queued prompts and route model tool calls to a ROS service."""

    def __init__(self) -> None:
        """Initialize ROS interfaces, OpenAI client, and local history."""
        super().__init__("llm_model")
        self.config = UserConfig()

        self.declare_parameter("input_topic", "/llm/input_text")
        self.declare_parameter("feedback_topic", "/llm/feedback")
        self.declare_parameter("function_service", "/llm/function_call")

        input_topic = self.get_parameter("input_topic").value
        feedback_topic = self.get_parameter("feedback_topic").value
        function_service = self.get_parameter("function_service").value

        self.initialization_publisher = self.create_publisher(
            String, "/llm/initialization_state", 10
        )
        self.state_publisher = self.create_publisher(String, "/llm/state", 10)
        self.response_type_publisher = self.create_publisher(
            String, "/llm/response_type", 10
        )
        self.feedback_publisher = self.create_publisher(String, feedback_topic, 10)
        self.input_subscriber = self.create_subscription(
            String, input_topic, self.llm_callback, 10
        )
        self.function_call_client = self.create_client(ChatGPT, function_service)

        self.client = None
        if self.config.openai_api_key:
            self.client = OpenAI(
                api_key=self.config.openai_api_key,
                timeout=self.config.openai_request_timeout,
                max_retries=2,
            )
        else:
            self.get_logger().error(
                "OPENAI_API_KEY is not set; text requests will be rejected."
            )

        self.input_items: list[dict[str, Any]] = []
        self.pending_prompts: deque[str] = deque()
        self.allowed_tool_names = {
            tool["name"] for tool in self.config.robot_functions_list
        }
        self.pending_tool_call: dict[str, str] | None = None
        self.tool_iterations = 0
        self.busy = False

        self.api_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="openai-responses"
        )
        self.api_future: Future | None = None
        self.poll_timer = self.create_timer(0.1, self._poll_api_future)

        timestamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        history_directory = self.config.chat_history_path
        history_directory.mkdir(parents=True, exist_ok=True)
        self.chat_history_file = history_directory / (f"chat_history_{timestamp}.json")
        self._write_chat_history()
        self._publish_string("llm_model", self.initialization_publisher)
        self.get_logger().info(f"OpenAI model configured as {self.config.openai_model}")

    def llm_callback(self, message: String) -> None:
        """Queue a non-empty user prompt without blocking the ROS executor."""
        prompt = message.data.strip()
        if not prompt:
            self.get_logger().warning("Ignored an empty LLM input message.")
            return
        self.pending_prompts.append(prompt)
        self._start_next_prompt()

    def _start_next_prompt(self) -> None:
        if self.busy or not self.pending_prompts:
            return
        if self.client is None:
            self.pending_prompts.popleft()
            self._publish_error(
                "OpenAI is not configured. Set OPENAI_API_KEY and restart."
            )
            return

        prompt = self.pending_prompts.popleft()
        self.busy = True
        self.tool_iterations = 0
        self.input_items.append({"role": "user", "content": prompt})
        self._publish_string("model_processing", self.state_publisher)
        self._submit_response_request()

    def _submit_response_request(self) -> None:
        request: dict[str, Any] = {
            "model": self.config.openai_model,
            "input": list(self.input_items),
            "instructions": self.config.system_prompt,
            "tools": self.config.robot_functions_list,
            "parallel_tool_calls": False,
            "store": False,
            "max_output_tokens": self.config.openai_max_output_tokens,
        }
        if self.config.openai_model.startswith("gpt-5"):
            request["reasoning"] = {"effort": self.config.openai_reasoning_effort}
            request["include"] = ["reasoning.encrypted_content"]

        self.api_future = self.api_executor.submit(
            self.client.responses.create, **request
        )

    def _poll_api_future(self) -> None:
        future = self.api_future
        if future is None or not future.done():
            return
        self.api_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"OpenAI request failed: {error}")
            self._publish_error("The language model request failed.")
            self._finish_turn()
            return
        self._handle_model_response(response)

    def _handle_model_response(self, response: Any) -> None:
        output_items = [_as_json_item(item) for item in response.output]
        self.input_items.extend(output_items)

        function_calls = [
            item
            for item in response.output
            if getattr(item, "type", None) == "function_call"
        ]
        if function_calls:
            self._dispatch_tool_call(function_calls[0])
            return

        text = (response.output_text or "").strip()
        if not text:
            self._publish_error("The language model returned no text output.")
        else:
            self._publish_string("feedback_for_user", self.response_type_publisher)
            self._publish_string(text, self.feedback_publisher)
        self._finish_turn()

    def _dispatch_tool_call(self, tool_call: Any) -> None:
        if self.tool_iterations >= self.config.max_tool_iterations:
            self._publish_error("The tool-call iteration limit was reached.")
            self._finish_turn()
            return
        if tool_call.name not in self.allowed_tool_names:
            self._publish_error(
                f"The model requested an unavailable tool: {tool_call.name}"
            )
            self._finish_turn()
            return
        if not self.function_call_client.wait_for_service(timeout_sec=0.1):
            self._publish_error("The robot function service is unavailable.")
            self._finish_turn()
            return

        self.tool_iterations += 1
        self.pending_tool_call = {
            "name": tool_call.name,
            "call_id": tool_call.call_id,
        }
        request = ChatGPT.Request()
        request.request_text = json.dumps(
            {
                "name": tool_call.name,
                "arguments": tool_call.arguments,
                "call_id": tool_call.call_id,
            }
        )
        self._publish_string("function_call", self.response_type_publisher)
        self._publish_string("function_execution", self.state_publisher)
        future = self.function_call_client.call_async(request)
        future.add_done_callback(self._tool_response_callback)

    def _tool_response_callback(self, future: Any) -> None:
        pending = self.pending_tool_call
        self.pending_tool_call = None
        if pending is None:
            self._publish_error("Received an unexpected robot tool response.")
            self._finish_turn()
            return
        try:
            service_response = future.result()
            output = service_response.response_text
        except Exception as error:
            output = json.dumps({"ok": False, "error": str(error)})

        self.input_items.append(
            {
                "type": "function_call_output",
                "call_id": pending["call_id"],
                "output": output,
            }
        )
        self._submit_response_request()

    def _finish_turn(self) -> None:
        self._trim_history()
        self._write_chat_history()
        self._publish_string("listening", self.state_publisher)
        self.busy = False
        self._start_next_prompt()

    def _trim_history(self) -> None:
        limit = max(1, self.config.chat_history_max_items)
        if len(self.input_items) <= limit:
            return
        start = len(self.input_items) - limit
        while start < len(self.input_items):
            if self.input_items[start].get("role") == "user":
                break
            start += 1
        if start >= len(self.input_items):
            start = len(self.input_items) - limit
        self.input_items = self.input_items[start:]

    def _write_chat_history(self) -> None:
        try:
            Path(self.chat_history_file).write_text(
                json.dumps(self.input_items, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
        except OSError as error:
            self.get_logger().error(f"Could not save chat history: {error}")

    def _publish_error(self, text: str) -> None:
        self._publish_string("error", self.response_type_publisher)
        self._publish_string(text, self.feedback_publisher)

    def _publish_string(self, text: str, publisher: Any) -> None:
        message = String()
        message.data = text
        publisher.publish(message)

    def destroy_node(self) -> bool:
        """Stop the API worker before destroying the ROS node."""
        self.api_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the model bridge with a two-thread ROS executor."""
    rclpy.init(args=args)
    node = ChatGPTNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
