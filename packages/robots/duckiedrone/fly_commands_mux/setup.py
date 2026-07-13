import os
from glob import glob

from setuptools import setup

package_name = "fly_commands_mux"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="ROS2 fly commands multiplexer for Duckiedrone.",
    license="Duckietown License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fly_commands_mux_node = fly_commands_mux.fly_commands_mux_node:main",
        ],
    },
)