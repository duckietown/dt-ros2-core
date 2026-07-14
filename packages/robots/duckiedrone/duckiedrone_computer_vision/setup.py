"""Setuptools config for Duckiedrone computer vision."""

from pathlib import Path

from setuptools import setup

package_name = "duckiedrone_computer_vision"
launch_files = [path.as_posix() for path in Path("launch").glob("*.launch.py")]
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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="ROS2 launch composition for Duckiedrone computer vision.",
    license="Duckietown License",
    tests_require=["pytest"],
)
