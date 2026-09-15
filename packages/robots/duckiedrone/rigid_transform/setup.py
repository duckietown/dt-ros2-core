"""Setuptools config for Duckiedrone rigid-transform."""

from pathlib import Path

from setuptools import setup

package_name = "rigid_transform"
share_dir = Path("share") / package_name
launch_files = [
    path.as_posix()
    for path in Path("launch").glob("*.launch.py")
]

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
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
    description="ROS2 rigid-transform visual odometry for Duckiedrone.",
    license="Duckietown License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rigid_transform_node = rigid_transform.rigid_transform_node:main",
        ],
    },
)
