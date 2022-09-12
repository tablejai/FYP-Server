from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='msgs',
            executable='send_triple_node_raw',
            name='pseudo_triple_node_raw',
            output="screen"
        ),
        Node(
            package='augmentor',
            executable='augmentor',
            name='augmentor'
        ),
    ])