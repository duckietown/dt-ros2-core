from setuptools import setup

package_name = 'dagu_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duckietown',
    maintainer_email='maintainer@example.com',
    description='ROS 2 port of Dagu car interface nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = dagu_car.kinematics_node:main',
            'velocity_to_pose_node = dagu_car.velocity_to_pose_node:main',
            'car_cmd_switch_node = dagu_car.car_cmd_switch_node:main',
        ],
    },
)


