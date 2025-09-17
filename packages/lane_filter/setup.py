
from setuptools import find_packages, setup
from pathlib import Path

package_name = 'lane_filter'

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
    version='1.0.0',
    packages=find_packages('include'),
    package_dir={'': 'include'},
    data_files=list(data_files.items()),
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Duckietown',
    maintainer_email='maintainer@example.com',
    description='Lane filter for estimating lane pose from line segments.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_filter_node = lane_filter.ros2_lane_filter_node:main',
            'lane_filter_tester_node = lane_filter.ros2_lane_filter_tester:main',
        ],
    },
)
