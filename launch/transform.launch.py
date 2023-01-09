from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='transform',
            executable='transform',
            name='transform_broadcaster',
            parameters=[
                {'imuName': 'imu_palm'}
            ]
        ),
    ])

