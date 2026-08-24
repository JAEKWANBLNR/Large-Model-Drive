"""Record microphone audio and transcribe it with local Whisper."""

import tempfile
import threading
from pathlib import Path

import rclpy
import sounddevice as sd
import whisper
from llm_config.user_config import UserConfig
from rclpy.node import Node
from scipy.io.wavfile import write
from std_msgs.msg import String


class LocalAudioInput(Node):
    """Publish local Whisper transcripts without blocking ROS callbacks."""

    def __init__(self) -> None:
        """Configure audio parameters and ROS publishers/subscribers."""
        super().__init__("llm_audio_input_local")
        self.config = UserConfig()
        self.declare_parameter("input_topic", "/llm/input_text")
        self.state_publisher = self.create_publisher(String, "/llm/state", 10)
        self.initialization_publisher = self.create_publisher(
            String, "/llm/initialization_state", 10
        )
        self.transcript_publisher = self.create_publisher(
            String, self.get_parameter("input_topic").value, 10
        )
        self.state_subscriber = self.create_subscription(
            String, "/llm/state", self.state_listener_callback, 10
        )
        self.busy = False
        self.whisper_model = None
        self._publish_string("llm_audio_input_local", self.initialization_publisher)
        self.start_timer = self.create_timer(2.0, self._start_listening)

    def _start_listening(self) -> None:
        self._publish_string("listening", self.state_publisher)
        self.start_timer.cancel()

    def state_listener_callback(self, message: String) -> None:
        """Start one local transcription when the state becomes listening."""
        if message.data != "listening" or self.busy:
            return
        self.busy = True
        threading.Thread(
            target=self._record_and_transcribe,
            name="local-audio-input",
            daemon=True,
        ).start()

    def _record_and_transcribe(self) -> None:
        audio_path = Path(tempfile.gettempdir()) / "llm_user_audio_input.wav"
        try:
            sample_count = int(self.config.duration * self.config.sample_rate)
            self.get_logger().info("Recording microphone input.")
            audio_data = sd.rec(
                sample_count,
                samplerate=self.config.sample_rate,
                channels=1,
                dtype="float32",
            )
            sd.wait()
            audio_data *= self.config.volume_gain_multiplier
            write(audio_path, self.config.sample_rate, audio_data)
            self._publish_string("input_processing", self.state_publisher)

            if self.whisper_model is None:
                self.get_logger().info(
                    f"Loading Whisper model: {self.config.whisper_model_size}"
                )
                self.whisper_model = whisper.load_model(self.config.whisper_model_size)
            result = self.whisper_model.transcribe(
                str(audio_path), language=self.config.whisper_language
            )
            transcript = str(result.get("text", "")).strip()
            if transcript:
                self._publish_string(transcript, self.transcript_publisher)
            else:
                self.get_logger().warning("Whisper returned an empty transcript.")
                self._publish_string("listening", self.state_publisher)
        except Exception as error:
            self.get_logger().error(f"Local transcription failed: {error}")
            self._publish_string("listening", self.state_publisher)
        finally:
            try:
                audio_path.unlink(missing_ok=True)
            except OSError:
                pass
            self.busy = False

    @staticmethod
    def _publish_string(text: str, publisher: object) -> None:
        message = String()
        message.data = text
        publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the local Whisper audio-input ROS node."""
    rclpy.init(args=args)
    node = LocalAudioInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
