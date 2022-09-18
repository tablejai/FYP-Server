# Labeler

## Dependencies
* rclpy
* rosbag2
* msgs
* gesture_definition

## How to compile
```sh
colcon build --packages-up-to labeler
source install/setup.bash
```

## How to run
```sh
# run solely
ros2 run labeler labeler
```

## For development
```sh 
colcon build && source install/setup.bash && ros2 run labeler labeler
```