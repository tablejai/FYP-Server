# Covariance Matrix Calculator

## Dependencies
* rclpy
* rosbag2 (pip install rosbags)
* msgs

## How to compile
```sh
colcon build --packages-up-to covariance_matrix_calculator
source install/setup.bash
```

## How to run
```sh
# run solely
ros2 run covariance_matrix_calculator calculator
```

## For development
```sh 
colcon build && source install/setup.bash && ros2 run covariance_matrix_calculator calculator
```
