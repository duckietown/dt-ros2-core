"""Setuptools config for Duckiedrone localization."""

# ruff: noqa: INP001

from pathlib import Path

from setuptools import find_packages, setup

package_name = "localization"
launch_files = [path.as_posix() for path in Path("launch").glob("*.launch.py")]
share_dir = Path("share") / package_name

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        ("share/" + package_name, ["package.xml"]),
        (str(share_dir / "launch"), launch_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="ROS2 Monte-Carlo localization for Duckiedrone.",
    license="Duckietown License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization_node = localization.localization_node:main",
        ],
    },
)
