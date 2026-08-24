"""Package the OpenAI model orchestration node."""

from setuptools import setup

package_name = "llm_model"

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
    description=("ROS 2 bridge for the OpenAI Responses API and robot tools."),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chatgpt = llm_model.chatgpt:main",
        ],
    },
)
