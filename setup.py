from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tablebot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),

        # Install maps if present
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uvs4',
    maintainer_email='uvs4@todo.todo',
    description='ROS 2 bringup package for the Tablebot skid-steer waypoint challenge robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teensy_serial_bridge = tablebot_bringup.teensy_serial_bridge:main',
            'circle_table_waypoints = tablebot_bringup.circle_table_waypoints:main',
        ],
    },
)
