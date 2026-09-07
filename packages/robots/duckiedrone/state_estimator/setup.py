"""Setuptools config for Duckiedrone state estimator."""

from pathlib import Path

from setuptools import setup

package_name = "state_estimator"
share_dir = Path("share") / package_name
launch_files = [path.as_posix() for path in Path("launch").glob("*.launch.py")]
config_files = [path.as_posix() for path in Path("config").glob("*.yaml")]

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
        (str(share_dir / "config"), config_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="ROS2 state estimation nodes for Duckiedrone.",
    license="Duckietown License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_estimator_node = state_estimator.state_estimator_node:main",
            "simple_state_estimator_node = "
            "state_estimator.simple_state_estimator_node:main",
        ],
    },
)
