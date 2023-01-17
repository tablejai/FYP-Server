from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='receiver',
            executable='receiver',
            output="screen"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('imu_filter_madgwick'),
                    'launch/imu_cluster.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('dynamic_transform_calculator'),
                    'launch/calculator.launch.py'))
        ),
    ])