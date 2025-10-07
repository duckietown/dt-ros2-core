from setuptools import setup

package_name = 'lane_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lane_control']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lane_controller_node.launch.py']),
        ('share/' + package_name + '/config', ['config/lane_controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Thayer',
    maintainer_email='daniel.thayer@duckietown.com',
    description='ROS 2 lane control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_controller=lane_control.lane_controller_node:main',
            'lane_controller_node=lane_control.lane_controller_node:main',
        ],
    },
)
