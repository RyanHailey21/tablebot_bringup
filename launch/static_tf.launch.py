from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                # x y z roll pitch yaw parent child
                # Robot length: 27 in. Center is 13.5 in from rear.
                # Lidar is 17.5 in from rear, so x = 4.0 in = 0.1016 m.
                # Lidar is centered laterally, so y = 0.0 m.
                # Lidar scan plane is 13 in above floor, so z = 0.3302 m.
                '0.1016', '0.0000', '0.3302',
                '0.0000', '0.0000', '0.0000',
                'base_link', 'laser_frame'
            ]
        )
    ])
