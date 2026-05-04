from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('tablebot_bringup')

    static_tf_launch = os.path.join(pkg, 'launch', 'static_tf.launch.py')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    teensy_port = LaunchConfiguration('teensy_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'teensy_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the Teensy motor controller'
        ),

        Node(
            package='tablebot_bringup',
            executable='teensy_serial_bridge',
            name='teensy_serial_bridge',
            output='screen',
            parameters=[{
                'port': teensy_port,
                'baudrate': 115200,
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/wheel/odom',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'max_linear_x': 0.35,
                'max_angular_z': 1.50,
                'command_timeout_sec': 0.50,
                'send_rate_hz': 20.0,
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch)
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
    ])
