"""Setuptools config for Duckiedrone fly commands mux."""

from pathlib import Path

from setuptools import setup

package_name = "fly_commands_mux"
launch_files = [path.as_posix() for path in Path("launch").glob("*.launch.py")]
config_files = [path.as_posix() for path in Path("config").glob("*.yaml")]
share_dir = Path("share") / package_name

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        ("share/" + package_name, ["package.xml"]),
        (str(share_dir / "launch"), launch_files),
        (str(share_dir / "config"), config_files),
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
            (
                "fly_commands_mux_node = "
                "fly_commands_mux.fly_commands_mux_node:main"
            ),
        ],
    },
)
