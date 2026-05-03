from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    tablebot_pkg = get_package_share_directory('tablebot_bringup')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    base_launch = os.path.join(tablebot_pkg, 'launch', 'base.launch.py')
    lidar_launch = os.path.join(sllidar_pkg, 'launch', 'sllidar_a2m8_launch.py')
    slam_launch = os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
    slam_config = os.path.join(tablebot_pkg, 'config', 'slam_toolbox_mapping.yaml')
    rviz_config = os.path.join(tablebot_pkg, 'config', 'mapping.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '115200',
                'frame_id': 'laser_frame',
                'inverted': 'false',
                'angle_compensate': 'true',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'slam_params_file': slam_config,
                'use_sim_time': 'false',
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])
