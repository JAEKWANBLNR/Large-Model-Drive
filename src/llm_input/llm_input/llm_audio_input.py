"""Record microphone audio and transcribe it with Amazon Transcribe."""

import datetime as dt
import json
import tempfile
import threading
import time
import uuid
from pathlib import Path

import boto3
import rclpy
import requests
import sounddevice as sd
from llm_config.user_config import UserConfig
from rclpy.node import Node
from scipy.io.wavfile import write
from std_msgs.msg import String


class AwsAudioInput(Node):
    """Publish AWS Transcribe results using the normal AWS credential chain."""

    def __init__(self) -> None:
        """Initialize AWS clients lazily and configure ROS interfaces."""
        super().__init__("llm_audio_input_aws")
        self.config = UserConfig()
        self.declare_parameter("input_topic", "/llm/input_text")
        self.aws_session = boto3.Session(region_name=self.config.aws_region_name)
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
        self._publish_string("llm_audio_input_aws", self.initialization_publisher)
        self.start_timer = self.create_timer(2.0, self._start_listening)

    def _start_listening(self) -> None:
        self._publish_string("listening", self.state_publisher)
        self.start_timer.cancel()

    def state_listener_callback(self, message: String) -> None:
        """Start one recording when the shared state enters listening."""
        if message.data != "listening" or self.busy:
            return
        self.busy = True
        threading.Thread(
            target=self._record_and_transcribe,
            name="aws-audio-input",
            daemon=True,
        ).start()

    def _record_and_transcribe(self) -> None:
        bucket = self.config.bucket_name
        if not bucket:
            self.get_logger().error("AWS_S3_BUCKET is required for AWS input.")
            self._publish_string("listening", self.state_publisher)
            self.busy = False
            return

        identifier = uuid.uuid4().hex
        audio_path = Path(tempfile.gettempdir()) / f"llm_input_{identifier}.wav"
        object_key = f"large-model-drive/audio/{identifier}.wav"
        job_name = f"large-model-drive-{identifier}"
        s3 = self.aws_session.client("s3")
        uploaded = False
        try:
            sample_count = int(self.config.duration * self.config.sample_rate)
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

            s3.upload_file(str(audio_path), bucket, object_key)
            uploaded = True
            transcribe = self.aws_session.client("transcribe")
            transcribe.start_transcription_job(
                TranscriptionJobName=job_name,
                LanguageCode=self.config.aws_transcription_language,
                MediaFormat="wav",
                Media={"MediaFileUri": f"s3://{bucket}/{object_key}"},
            )
            status = self._wait_for_job(transcribe, job_name)
            job = status["TranscriptionJob"]
            if job["TranscriptionJobStatus"] != "COMPLETED":
                raise RuntimeError(job.get("FailureReason", "Transcription failed"))

            transcript_url = job["Transcript"]["TranscriptFileUri"]
            http_response = requests.get(transcript_url, timeout=15)
            http_response.raise_for_status()
            transcript_data = json.loads(http_response.text)
            transcript = transcript_data["results"]["transcripts"][0][
                "transcript"
            ].strip()
            if transcript:
                self._publish_string(transcript, self.transcript_publisher)
            else:
                self._publish_string("listening", self.state_publisher)
        except Exception as error:
            timestamp = dt.datetime.now(dt.timezone.utc).isoformat()
            self.get_logger().error(f"AWS transcription failed at {timestamp}: {error}")
            self._publish_string("listening", self.state_publisher)
        finally:
            if uploaded:
                try:
                    s3.delete_object(Bucket=bucket, Key=object_key)
                except Exception as error:
                    self.get_logger().warning(f"S3 cleanup failed: {error}")
            try:
                audio_path.unlink(missing_ok=True)
            except OSError:
                pass
            self.busy = False

    @staticmethod
    def _wait_for_job(transcribe: object, job_name: str) -> dict:
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            status = transcribe.get_transcription_job(TranscriptionJobName=job_name)
            state = status["TranscriptionJob"]["TranscriptionJobStatus"]
            if state in {"COMPLETED", "FAILED"}:
                return status
            time.sleep(1.0)
        raise TimeoutError("Amazon Transcribe did not finish within 180 seconds.")

    @staticmethod
    def _publish_string(text: str, publisher: object) -> None:
        message = String()
        message.data = text
        publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the AWS audio-input ROS node."""
    rclpy.init(args=args)
    node = AwsAudioInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
