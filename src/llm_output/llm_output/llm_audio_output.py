"""Synthesize LLM feedback with Amazon Polly and play it locally."""

import subprocess
import tempfile
import threading
import uuid
from pathlib import Path

import boto3
import rclpy
from llm_config.user_config import UserConfig
from rclpy.node import Node
from std_msgs.msg import String


class AudioOutput(Node):
    """Play queued feedback while keeping the ROS callback responsive."""

    def __init__(self) -> None:
        """Configure Polly access and ROS feedback interfaces."""
        super().__init__("llm_audio_output")
        self.config = UserConfig()
        self.declare_parameter("feedback_topic", "/llm/feedback")
        self.aws_session = boto3.Session(region_name=self.config.aws_region_name)
        self.state_publisher = self.create_publisher(String, "/llm/state", 10)
        self.initialization_publisher = self.create_publisher(
            String, "/llm/initialization_state", 10
        )
        self.feedback_subscriber = self.create_subscription(
            String,
            self.get_parameter("feedback_topic").value,
            self.feedback_for_user_callback,
            10,
        )
        self.busy = False
        self._publish_string("llm_audio_output", self.initialization_publisher)

    def feedback_for_user_callback(self, message: String) -> None:
        """Start speech synthesis for one non-empty feedback message."""
        text = message.data.strip()
        if not text:
            return
        if self.busy:
            self.get_logger().warning("Audio output is busy; feedback was skipped.")
            return
        self.busy = True
        threading.Thread(
            target=self._synthesize_and_play,
            args=(text,),
            name="aws-audio-output",
            daemon=True,
        ).start()

    def _synthesize_and_play(self, text: str) -> None:
        output_path = Path(tempfile.gettempdir()) / (
            f"llm_speech_{uuid.uuid4().hex}.mp3"
        )
        try:
            polly = self.aws_session.client("polly")
            response = polly.synthesize_speech(
                Text=text,
                OutputFormat="mp3",
                VoiceId=self.config.aws_voice_id,
            )
            output_path.write_bytes(response["AudioStream"].read())
            subprocess.run(
                [self.config.audio_player, "--no-video", str(output_path)],
                check=True,
                timeout=120,
            )
        except Exception as error:
            self.get_logger().error(f"Audio output failed: {error}")
        finally:
            try:
                output_path.unlink(missing_ok=True)
            except OSError:
                pass
            self.busy = False
            self._publish_string("listening", self.state_publisher)

    @staticmethod
    def _publish_string(text: str, publisher: object) -> None:
        message = String()
        message.data = text
        publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the AWS Polly audio-output ROS node."""
    rclpy.init(args=args)
    node = AudioOutput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
