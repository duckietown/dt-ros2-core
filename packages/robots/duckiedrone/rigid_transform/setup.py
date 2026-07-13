import os
from glob import glob

from setuptools import setup

package_name = "rigid_transform"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="ROS2 rigid-transform visual odometry for Duckiedrone.",
    license="Duckietown License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rigid_transform_node = rigid_transform.rigid_transform_node:main",
        ],
    },
)