from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

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
                    get_package_share_directory('dtf_calculator'),
                    'launch/dtf_calculator.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('detector'),
                    'launch/gesture_detection.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('commander'),
                    'launch/control.launch.py'))
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="viz",
            output="log",
            arguments=["--display-config", "/home/ubuntu/FYP-ROS/src/glove.rviz"],
        ),
        # Node(
        #     package="rqt_graph",
        #     executable="rqt_graph",
        #     name="node_graph",
        #     output="log",
        # ),
    ])