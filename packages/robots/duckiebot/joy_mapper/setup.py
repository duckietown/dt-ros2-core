from setuptools import setup
import os

package_name = 'joy_mapper'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/joy_mapper_node.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/joy_mapper_node.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mack',
    maintainer_email='mack@duckietown.org',
    description='ROS 2 joy_mapper package for Duckietown',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_mapper = joy_mapper.joy_mapper_node:main',
        ],
    },
)
