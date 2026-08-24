"""Package shared Large Model Drive configuration."""

from setuptools import setup

package_name = "llm_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hermanye",
    maintainer_email="hermanye233@icloud.com",
    description=("Environment-backed configuration and tool schemas for the robot."),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
