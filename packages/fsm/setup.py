from glob import glob
import os
from setuptools import find_packages, setup
from pathlib import Path

package_name = 'fsm'

here = Path(__file__).parent.resolve()

launch_dir = here / 'launch'
config_dir = here / 'config'

data_files = {
    'share/ament_index/resource_index/packages': [str(Path('resource') / package_name)],
    'share/' + package_name: ['package.xml'],
}

if launch_dir.exists():
    launch_files = [str(p.relative_to(here)) for p in launch_dir.glob('*.py')]
    if launch_files:
        data_files['share/' + package_name + '/launch'] = launch_files

if config_dir.exists():
    for path in config_dir.rglob('*'):
       if path.is_file():
            relative = path.relative_to(here)
            parent = relative.parent.as_posix()
            destination = 'share/' + package_name
            if parent and parent != '.':
                destination += '/' + parent
            data_files.setdefault(destination, []).append(str(path.relative_to(here)))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'fsm_node'), glob('config/fsm_node/*.yaml')),
        (os.path.join('share', package_name, 'config', 'logic_gate_node'), glob('config/logic_gate_node/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide Iafrate',
    maintainer_email='davide.iafrate@duckietown.org',
    description="Projects image line segments onto the ground plane using camera extrinsics (ROS 2).",
    license="GPLv3",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = fsm.fsm_node:main'
        ],
    },
)
