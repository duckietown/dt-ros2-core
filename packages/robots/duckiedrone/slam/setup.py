import os
from glob import glob

from setuptools import find_packages, setup

package_name = "slam"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="ROS2 FastSLAM for Duckiedrone.",
    license="Duckietown License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "slam_node = slam.slam_node:main",
        ],
    },
)