from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dtf_calculator',
            executable='dtf_calculator',
            parameters=[
                {'imuName': 'imu_palm'}
            ]
        ),
    ])

