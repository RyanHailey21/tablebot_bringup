from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    tablebot_pkg = get_package_share_directory('tablebot_bringup')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    base_launch = os.path.join(tablebot_pkg, 'launch', 'base.launch.py')
    lidar_launch = os.path.join(sllidar_pkg, 'launch', 'sllidar_a2m8_launch.py')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
    nav2_params = os.path.join(tablebot_pkg, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(tablebot_pkg, 'maps', 'table_area.yaml')
    rviz_config = os.path.join(tablebot_pkg, 'config', 'mapping.rviz')

    lidar_port = LaunchConfiguration('lidar_port')
    teensy_port = LaunchConfiguration('teensy_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for the SLLidar'
        ),

        DeclareLaunchArgument(
            'teensy_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the Teensy motor controller'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'teensy_port': teensy_port,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            launch_arguments={
                'serial_port': lidar_port,
                'serial_baudrate': '115200',
                'frame_id': 'laser_frame',
                'inverted': 'false',
                'angle_compensate': 'true',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'use_sim_time': 'false',
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
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
