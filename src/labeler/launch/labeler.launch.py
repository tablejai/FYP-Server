from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')
    return LaunchDescription([
        Node(
            package='labeler',
            executable='labeler',
            output="screen",
            parameters=[
                {'bag_path': bag_path}
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', f'/home/ubuntu/FYP-ROS/rosbag/bag/real_data/{bag_path}'],
        # ),
    ])