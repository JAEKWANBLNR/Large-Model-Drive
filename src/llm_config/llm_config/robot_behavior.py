"""OpenAI function-tool schemas exposed by each robot profile."""

from copy import deepcopy

MOBILE_MOTION_TOOL = {
    "type": "function",
    "name": "publish_cmd_vel",
    "description": (
        "Move the configured mobile robot for a short, bounded duration. "
        "Use zero for axes that the robot does not support."
    ),
    "parameters": {
        "type": "object",
        "properties": {
            "linear_x": {
                "type": "number",
                "minimum": -0.5,
                "maximum": 0.5,
                "description": "Forward velocity in metres per second.",
            },
            "linear_y": {
                "type": "number",
                "minimum": -0.5,
                "maximum": 0.5,
                "description": "Lateral velocity in metres per second.",
            },
            "angular_z": {
                "type": "number",
                "minimum": -1.5,
                "maximum": 1.5,
                "description": "Yaw rate in radians per second.",
            },
            "duration": {
                "type": "number",
                "minimum": 0.1,
                "maximum": 10.0,
                "description": "Seconds before an automatic stop command.",
            },
        },
        "required": ["linear_x", "linear_y", "angular_z", "duration"],
        "additionalProperties": False,
    },
    "strict": True,
}


MULTI_ROBOT_MOTION_TOOL = deepcopy(MOBILE_MOTION_TOOL)
MULTI_ROBOT_MOTION_TOOL["parameters"]["properties"]["robot_name"] = {
    "type": "string",
    "description": "ROS namespace of the target robot.",
}
MULTI_ROBOT_MOTION_TOOL["parameters"]["required"].append("robot_name")


ARM_POSE_TOOL = {
    "type": "function",
    "name": "publish_target_pose",
    "description": "Publish a Cartesian target pose for the configured arm.",
    "parameters": {
        "type": "object",
        "properties": {
            name: {
                "type": "number",
                "minimum": -10.0,
                "maximum": 10.0,
                "description": f"Target pose {name} component.",
            }
            for name in ("x", "y", "z", "roll", "pitch", "yaw")
        },
        "required": ["x", "y", "z", "roll", "pitch", "yaw"],
        "additionalProperties": False,
    },
    "strict": True,
}


PROFILE_TOOLS = {
    "mobile": [MOBILE_MOTION_TOOL],
    "multi": [MULTI_ROBOT_MOTION_TOOL],
    "arm": [ARM_POSE_TOOL],
}


class RobotBehavior:
    """Select the smallest tool surface required by a robot profile."""

    def __init__(self, profile: str = "mobile") -> None:
        """Initialize the tool list for a named robot profile."""
        normalized_profile = profile.strip().lower()
        if normalized_profile not in PROFILE_TOOLS:
            choices = ", ".join(sorted(PROFILE_TOOLS))
            raise ValueError(
                f"Unknown ROBOT_PROFILE '{profile}'. Expected one of: {choices}"
            )
        self.profile = normalized_profile
        self.robot_functions_list = deepcopy(PROFILE_TOOLS[normalized_profile])
