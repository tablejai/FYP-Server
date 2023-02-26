from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    bag_name = "rosbag2_2023_02_10-08_23_35"

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            output="log",
            arguments=["--display-config", "src/glove.rviz"],
        ),
        
        Node(
            package='labeler',
            executable='labeler',
            output="screen",
            parameters=[
                {'bag_name': bag_name},
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', f"rosbag/bag/{bag_name}"],
                    output='log'
                ),
            ]
        ),
    ])