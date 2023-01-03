# FYP ROS SYSTEM

## How to setup Docker 
### Run Ros2 docker container
```sh
xhost + 127.0.0.1
docker run --name fyp_ros2_x11 -it --privileged --net=host --ipc=host \
--device=/dev/dri:/dev/dri \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=host.docker.internal:0 \
-v $HOME/.Xauthority:/home/$(id -un)/.Xauthority -e XAUTHORITY=/home/$(id -un)/.Xauthority \
-e DOCKER_USER_NAME=$(id -un) \
-e DOCKER_USER_ID=$(id -u) \
-e DOCKER_USER_GROUP_NAME=$(id -gn) \
-e DOCKER_USER_GROUP_ID=$(id -g) \
-e ROS_IP=127.0.0.1 \
ros:foxy
```

### Install requirements
```sh
sudo bash setup.sh
```

## How to setup ROS
```sh
# adding ROS system variables, such as ros2 command line
source /opt/ros/foxy/setup.bash     

## adding custom ROS variables so that ros2 command line understand our packages, such as msgs, augmentor
source /path_to_this_workspace/install/setup.bash 
```

## How to create a package

### C/C++
```sh
ros2 pkg create --build-type ament_cmake [package]
```

### Python
```sh
ros2 pkg create --build-type ament_python [package]
````

More about creating and configuring the package
1. Write your python code under [package]/[package]/[package.py]
2. Add an entry point to setup.py so that ros2 can locate its position
    ```py
    entry_points={
        'console_scripts': [
            '[node_name] = [package].[script]:main',
        ],
    }
    ```

Links:

* [Create a Python package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* [Entry Point For Python](https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html#python-packages)

## How to build
* build a package
    ```sh 
    colcon build --packages-select [package]
    ```

* build a package with its dependencies(recommended)
    ```sh 
    colcon build --packages-up-to [package]
    ```

* build all packages under the workspace 
    ```sh 
    colcon build
    ```


## How to run
### C/C++
```sh 
ros2 run [package] [executable]
```

### Python
```sh 
ros2 run [package] [executable]
# or
python3 src/[package]/src/[executable]
```

## System Overview

* receiver
    ```
    Subscribers:

    Publishers:
        /raw: std_msgs/msg/String
    ```

* augmentor
    ```
    Subscribers:
        /ImuRawArray: msgs/msg/ImuRawArray
    Publishers:
        /ImuAugmentedArray: msgs/msg/ImuAugmentedArray
    ```

* labeler
    ```
    Subscribers:

    Publishers:
        /Imu_viz: msgs/msg/ImuAugmentedArray
    ```

* visualizer(TODO)
    ```
    Subscribers:
        /ImuAugmentedArray: msgs/msg/ImuAugmentedArray
    Publishers:
        /Imu_viz: msgs/msg/ImuAugmentedArray
    ```

* model(TODO)
    ```
    Subscribers:
        /ImuAugmentedArray: msgs/msg/ImuAugmentedArray
    Publishers:
        /gesture: msgs/msg/GestureType
    ```

* decision(TODO)
    ```
    Subscribers:
        /gesture: msgs/msg/GestureType
    Publishers:
        /command: msgs/msg/Command
    ```