from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dtf_calculator',
            executable='dtf_calculator',
            parameters=[
                {'finger_length': 2.15},
                {'use_current_time': False},
            ]
        ),
        Node(
            package="tf2_ros", 
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["0", "0", "3", "0", "0", "0", "world", "imu0"]
        ),
    ])

