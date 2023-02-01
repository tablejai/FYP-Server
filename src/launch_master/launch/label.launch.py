from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    bag_name = "rosbag2_2023_01_31-04_21_03"
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            output="log",
            arguments=["--display-config", "/home/ubuntu/FYP-ROS/src/glove.rviz"],
        ),
        
        Node(
            package='labeler',
            executable='labeler',
            output="screen",
            parameters=[
                {'bag_name': bag_name}
            ],
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', f"/home/ubuntu/FYP-ROS/rosbag/bag/real_data/{bag_name}"],
                    output='log'
                ),
            ]
        ),
    ])