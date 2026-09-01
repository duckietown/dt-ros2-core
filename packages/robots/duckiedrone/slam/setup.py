"""Setuptools config for Duckiedrone SLAM."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "slam"
share_dir = Path("share") / package_name
launch_files = [
    path.as_posix()
    for path in Path("launch").glob("*.launch.py")
]

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=["slam", "slam.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (str(share_dir), ["package.xml"]),
        (str(share_dir / "launch"), launch_files),
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
