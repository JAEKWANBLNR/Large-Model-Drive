"""Environment-backed configuration for the Large Model Drive nodes."""

import os
from dataclasses import dataclass, field
from pathlib import Path

from .robot_behavior import RobotBehavior


def _env_float(name: str, default: float) -> float:
    """Read a floating-point environment variable with a safe default."""
    value = os.getenv(name)
    return default if value is None else float(value)


def _env_int(name: str, default: int) -> int:
    """Read an integer environment variable with a safe default."""
    value = os.getenv(name)
    return default if value is None else int(value)


def _history_directory() -> Path:
    configured = os.getenv("LLM_CHAT_HISTORY_DIR")
    if configured:
        return Path(configured).expanduser()
    return Path.home() / ".ros" / "large_model_drive"


@dataclass
class UserConfig:
    """Runtime settings shared by the ROS 2 packages.

    Secrets are read from the standard environment or provider credential chain.
    Nothing in this class writes credentials to disk.
    """

    openai_api_key: str | None = field(
        default_factory=lambda: os.getenv("OPENAI_API_KEY")
    )
    openai_model: str = field(
        default_factory=lambda: os.getenv("OPENAI_MODEL", "gpt-5.6-luna")
    )
    openai_reasoning_effort: str = field(
        default_factory=lambda: os.getenv("OPENAI_REASONING_EFFORT", "none")
    )
    openai_max_output_tokens: int = field(
        default_factory=lambda: _env_int("OPENAI_MAX_OUTPUT_TOKENS", 512)
    )
    openai_request_timeout: float = field(
        default_factory=lambda: _env_float("OPENAI_REQUEST_TIMEOUT", 30.0)
    )
    system_prompt: str = field(
        default_factory=lambda: os.getenv(
            "LLM_SYSTEM_PROMPT",
            (
                "Control only the configured robot. Use a tool for physical "
                "actions, and never claim an action succeeded until its tool "
                "result confirms success. Keep motion within the tool limits "
                "and ask for clarification when a command is ambiguous."
            ),
        )
    )
    chat_history_path: Path = field(default_factory=_history_directory)
    chat_history_max_items: int = field(
        default_factory=lambda: _env_int("LLM_CHAT_HISTORY_MAX_ITEMS", 100)
    )
    max_tool_iterations: int = field(
        default_factory=lambda: _env_int("LLM_MAX_TOOL_ITERATIONS", 3)
    )

    robot_profile: str = field(
        default_factory=lambda: os.getenv("ROBOT_PROFILE", "mobile")
    )
    multi_robots_name: tuple[str, ...] = field(
        default_factory=lambda: tuple(
            name.strip()
            for name in os.getenv("ROBOT_NAMES", "yolobot,turtle2,minipupper").split(
                ","
            )
            if name.strip()
        )
    )

    aws_region_name: str = field(
        default_factory=lambda: os.getenv("AWS_REGION", "ap-southeast-1")
    )
    bucket_name: str | None = field(default_factory=lambda: os.getenv("AWS_S3_BUCKET"))
    aws_transcription_language: str = field(
        default_factory=lambda: os.getenv("AWS_TRANSCRIBE_LANGUAGE", "en-US")
    )
    aws_voice_id: str = field(
        default_factory=lambda: os.getenv("AWS_POLLY_VOICE_ID", "Ivy")
    )

    whisper_model_size: str = field(
        default_factory=lambda: os.getenv("WHISPER_MODEL_SIZE", "base")
    )
    whisper_language: str = field(
        default_factory=lambda: os.getenv("WHISPER_LANGUAGE", "en")
    )
    duration: float = field(
        default_factory=lambda: _env_float("AUDIO_RECORD_SECONDS", 5.0)
    )
    sample_rate: int = field(
        default_factory=lambda: _env_int("AUDIO_SAMPLE_RATE", 16000)
    )
    volume_gain_multiplier: float = field(
        default_factory=lambda: _env_float("AUDIO_GAIN", 1.0)
    )
    audio_player: str = field(default_factory=lambda: os.getenv("AUDIO_PLAYER", "mpv"))

    robot_behavior: RobotBehavior = field(init=False)
    robot_functions_list: list[dict] = field(init=False)

    def __post_init__(self) -> None:
        """Resolve the profile-specific robot tool schema."""
        self.robot_behavior = RobotBehavior(self.robot_profile)
        self.robot_functions_list = self.robot_behavior.robot_functions_list
