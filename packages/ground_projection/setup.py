from setuptools import setup

package_name = "ground_projection"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/ground_projection.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Davide Iafrate",
    maintainer_email="davide.iafrate@duckietown.com",
    description="Projects image line segments onto the ground plane using camera extrinsics (ROS 2).",
    license="GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ground_projection_node = ground_projection.ground_projection_node:main",
        ],
    },
)
