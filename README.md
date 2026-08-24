# Large Model Drive

Large Model Drive is a ROS 2 mobile-robot simulation that combines natural-language control, bounded OpenAI function tools, YOLOv8 object detection, and PointCloud2 distance estimation.

The current release targets Ubuntu 22.04 with ROS 2 Humble. The Python code is also syntax-tested on Python 3.10 and 3.12.

> **Safety:** run new prompts in simulation first. Model output never bypasses the robot tool server, but hardware limits, frames, controller watchdogs, and emergency-stop behavior still need to be validated for each physical robot.

## What is included

- OpenAI Responses API integration with manual conversation replay and function-call outputs
- Profile-specific tool schemas for a mobile base, multiple robots, or an ARX5 arm
- Allow-listed tool execution with velocity, duration, and pose bounds
- YOLOv8 detection messages with class confidence and correct image coordinates
- Efficient organized-point-cloud sampling with median Euclidean distance
- Optional local Whisper input, AWS Transcribe input, and AWS Polly output
- Configurable ROS topics, model paths, launch options, and environment settings
- Dependency-free structural tests and GitHub Actions checks

The previous README mentioned BLIP, but this repository does not contain a BLIP/VLM node. The documentation now describes only functionality present in the source tree.

## Architecture

```text
/llm/input_text
      |
      v
  llm_model ---- OpenAI Responses API
      |                 |
      |          function_call
      v                 v
/llm/feedback   /llm/function_call service
                         |
                         v
                 bounded robot command

camera image -> YOLOv8 -> /yolov8/inference
                                |
organized point cloud ---------+-> /depth_extractor
```

## Packages

| Package | Purpose |
| --- | --- |
| `llm_bringup` | Full simulation and application launch |
| `llm_config` | Environment-backed model, robot, audio, and AWS settings |
| `llm_interfaces` | LLM tool-call service interface |
| `llm_model` | OpenAI Responses API node |
| `llm_robot` | Mobile, multi-robot, and arm tool servers |
| `llm_input` | Optional local Whisper or AWS Transcribe input |
| `llm_output` | Optional AWS Polly speech output |
| `yolobot_description` | URDF/Xacro, maps, and spawn launch |
| `yolobot_gazebo` | Gazebo worlds and simulation launch |
| `yolobot_recognition` | YOLOv8 and point-cloud distance nodes |
| `yolov8_msgs` | Detection message definitions |

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble Desktop
- Python 3.10+
- An OpenAI API key
- Gazebo Classic packages supplied with the ROS 2 Humble desktop install

AWS credentials are needed only for AWS Transcribe or Polly. A CUDA-capable PyTorch setup is optional; YOLO and Whisper can run on CPU.

## Installation

```bash
git clone https://github.com/JAEKWANBLNR/Large-Model-Drive.git
cd Large-Model-Drive

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

bash src/llm_install/dependencies_install.sh
# Include the larger local Whisper dependency only when needed:
# bash src/llm_install/dependencies_install.sh --with-whisper

colcon build --symlink-install
source install/setup.bash
```

The dependency script installs only required Ubuntu packages and Python dependencies. It no longer performs an unattended operating-system upgrade.

## Configuration

The application reads settings from environment variables. `.env.example` documents the available values; it is not loaded automatically.

Configure the OpenAI key without printing it or adding it to `.bashrc`:

```bash
bash src/llm_install/config_openai_api_key.sh
source "${XDG_CONFIG_HOME:-$HOME/.config}/large-model-drive/env"
```

The default model is `gpt-5.6-luna` with reasoning effort `none`, preserving the original project's latency-sensitive, lower-cost role while supporting modern tool calling. Override it when required:

```bash
export OPENAI_MODEL=gpt-5.6-terra
export OPENAI_REASONING_EFFORT=none
```

The implementation follows the official [OpenAI Responses API](https://developers.openai.com/api/reference/cli/resources/responses/methods/create) and [function calling](https://developers.openai.com/api/docs/guides/function-calling) contracts.

Robot profiles control which tools are visible to the model:

```bash
export ROBOT_PROFILE=mobile  # mobile, multi, or arm
export ROBOT_NAMES=yolobot,turtle2,minipupper
```

Chat histories contain prompts, model output, and tool results. They are written to `~/.ros/large_model_drive` by default. Change or disable persistence at the deployment layer if that data is sensitive.

## Run

Launch the simulation, Yolobot, YOLO, depth estimation, model node, and mobile tool server:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source "${XDG_CONFIG_HOME:-$HOME/.config}/large-model-drive/env"

ros2 launch llm_bringup chatgpt_with_turtle_robot.launch.py
```

YOLO downloads `yolov8n.pt` on first use when it is not cached. For repeatable or offline deployments, provide an absolute local path:

```bash
ros2 launch yolobot_recognition launch_yolov8.launch.py \
  model_path:=/absolute/path/to/yolov8n.pt
```

Send a text command:

```bash
ros2 topic pub --once /llm/input_text std_msgs/msg/String \
  "{data: 'Move forward slowly for two seconds, then stop.'}"
```

Observe feedback and state:

```bash
ros2 topic echo /llm/feedback
ros2 topic echo /llm/state
```

### Optional audio

Audio is disabled in the main launch by default.

Local Whisper input:

```bash
ros2 launch llm_bringup chatgpt_with_turtle_robot.launch.py \
  enable_audio_input:=true \
  audio_input_executable:=llm_audio_input_local
```

AWS input and output use the standard boto3 credential chain. Configure least-privilege credentials with an IAM role or `aws configure`, then store only the non-secret region and bucket settings:

```bash
bash src/llm_install/config_aws.sh
source "${XDG_CONFIG_HOME:-$HOME/.config}/large-model-drive/env"

ros2 launch llm_bringup chatgpt_with_turtle_robot.launch.py \
  enable_audio_input:=true \
  audio_input_executable:=llm_audio_input \
  enable_audio_output:=true
```

## Runtime contracts

| Name | Type | Direction |
| --- | --- | --- |
| `/llm/input_text` | `std_msgs/msg/String` | User/audio input to model |
| `/llm/feedback` | `std_msgs/msg/String` | Model text output |
| `/llm/state` | `std_msgs/msg/String` | Application state transitions |
| `/llm/response_type` | `std_msgs/msg/String` | Text, function call, or error |
| `/llm/function_call` | `llm_interfaces/srv/ChatGPT` | Model tool call to robot server |
| `/yolov8/inference` | `yolov8_msgs/msg/Yolov8Inference` | Typed detections |
| `/yolov8/annotated_image` | `sensor_msgs/msg/Image` | Detector visualization |
| `/yolov8/overlay_image` | `sensor_msgs/msg/Image` | Detection overlay on the camera stream |
| `/depth_extractor` | `std_msgs/msg/Float32` | Median target distance in metres |

Most topic names are ROS parameters and can be remapped without editing source.

## Safety limits

The mobile tool server clamps commands to these defaults:

- linear velocity: ±0.5 m/s
- yaw rate: ±1.5 rad/s
- duration: 0.1–10 seconds
- automatic zero-velocity command after the requested duration
- zero-velocity command when the controller node shuts down

Override the ROS parameters only after checking the target platform:

```bash
ros2 run llm_robot turtle_robot --ros-args \
  -p max_linear_speed:=0.25 \
  -p max_angular_speed:=0.8 \
  -p max_command_duration:=5.0
```

## Validation

The repository's local checks do not require ROS to be installed:

```bash
python3 -m compileall -q src
python3 -m unittest discover -s tests -v
bash -n src/llm_install/*.sh
```

On an Ubuntu/ROS machine, also run:

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## Breaking changes from the original prototype

- `openai.ChatCompletion` and legacy `functions` were replaced by the Responses API and strict function tools.
- The default model moved from `gpt-3.5-turbo` to the configurable `gpt-5.6-luna` latency tier.
- LLM topics and the tool service now use lowercase ROS namespaces shown above.
- `InferenceResult` now includes `confidence`, and bounding-box fields have conventional `left/top/right/bottom` semantics.
- Audio is opt-in, AWS secrets are no longer written to `.bashrc`, and the hard-coded bucket was removed.
- Stored `__pycache__` files and machine-specific launch paths were removed.

## License

Apache License 2.0. See [LICENSE](LICENSE).
