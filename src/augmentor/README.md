# Augmentor

## Dependencies
* rclpy
* msgs
* ros2launch
* tf_transformations   (sudo apt install ros-foxy-tf-transformations)
* transforms3d         (sudo pip3 install transforms3d)

## Brief
This module takes raw acceleration data and integrates it to get velocity and translation, both linearly and rotationally

## How to compile
```sh
colcon build --packages-up-to augmentor
source install/setup.bash
```

## How to run
```sh
# run solely
ros2 run augmentor augmentor

# run in launch file
cd src/augmentor/launch
ros2 launch augment_launch.py 
```