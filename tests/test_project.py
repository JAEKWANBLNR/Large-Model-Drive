"""Dependency-free structural tests for the repository."""

import ast
import os
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
CONFIG_SOURCE = ROOT / "src" / "llm_config"
sys.path.insert(0, str(CONFIG_SOURCE))

from llm_config.robot_behavior import RobotBehavior  # noqa: E402
from llm_config.user_config import UserConfig  # noqa: E402


class ProjectStructureTests(unittest.TestCase):
    """Check syntax and project metadata without requiring ROS to be installed."""

    def test_all_python_files_parse(self) -> None:
        for path in ROOT.glob("src/**/*.py"):
            with self.subTest(path=path.relative_to(ROOT)):
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    def test_package_manifests_are_complete(self) -> None:
        manifests = list(ROOT.glob("src/*/package.xml"))
        self.assertGreater(len(manifests), 0)
        for path in manifests:
            with self.subTest(path=path.relative_to(ROOT)):
                package = ET.parse(path).getroot()
                self.assertNotEqual(package.findtext("version"), "0.0.0")
                self.assertNotIn("TODO", package.findtext("description", ""))
                self.assertNotIn("TODO", package.findtext("license", ""))

    def test_no_zero_depth_qos_calls(self) -> None:
        method_names = {"create_publisher", "create_subscription"}
        for path in ROOT.glob("src/**/*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call) or not node.args:
                    continue
                function = node.func
                if not isinstance(function, ast.Attribute):
                    continue
                if function.attr not in method_names:
                    continue
                qos_argument = node.args[-1]
                with self.subTest(path=path.relative_to(ROOT), line=node.lineno):
                    self.assertFalse(
                        isinstance(qos_argument, ast.Constant)
                        and qos_argument.value == 0
                    )

    def test_source_has_no_user_specific_home_path(self) -> None:
        for pattern in ("src/**/*.py", "src/**/*.sh"):
            for path in ROOT.glob(pattern):
                text = path.read_text(encoding="utf-8")
                with self.subTest(path=path.relative_to(ROOT)):
                    self.assertNotIn("/home/park/", text)
                    self.assertNotIn("~/yolobot/", text)


class ConfigurationTests(unittest.TestCase):
    """Verify environment overrides and profile-specific tool schemas."""

    def test_robot_profiles_expose_only_expected_tools(self) -> None:
        expected = {
            "mobile": ["publish_cmd_vel"],
            "multi": ["publish_cmd_vel"],
            "arm": ["publish_target_pose"],
        }
        for profile, names in expected.items():
            with self.subTest(profile=profile):
                tools = RobotBehavior(profile).robot_functions_list
                self.assertEqual([tool["name"] for tool in tools], names)
                for tool in tools:
                    self.assertTrue(tool["strict"])
                    self.assertFalse(tool["parameters"]["additionalProperties"])

    def test_unknown_robot_profile_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            RobotBehavior("unknown")

    def test_environment_overrides_are_applied(self) -> None:
        environment = {
            "OPENAI_MODEL": "test-model",
            "OPENAI_MAX_OUTPUT_TOKENS": "321",
            "ROBOT_PROFILE": "arm",
            "ROBOT_NAMES": "alpha,beta",
        }
        with mock.patch.dict(os.environ, environment, clear=False):
            config = UserConfig()
        self.assertEqual(config.openai_model, "test-model")
        self.assertEqual(config.openai_max_output_tokens, 321)
        self.assertEqual(config.multi_robots_name, ("alpha", "beta"))
        self.assertEqual(config.robot_functions_list[0]["name"], "publish_target_pose")


if __name__ == "__main__":
    unittest.main()
