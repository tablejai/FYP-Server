# FYP ROS SYSTEM

## How to launch ROS
```sh
source /opt/ros/foxy/setup.bash     # adding ROS system variables, such as ros2 command line

source /path_to_this_workspace/install/setup.bash  ## adding custom ROS variables and include files, such as msgs
```

## How to create package

### C/C++
```sh
ros2 pkg create --build-type ament_cmake [package]
```

### Python
```sh
ros2 pkg create --build-type ament_python [package]
````
ref: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

## How to build

### C/C++
* build a package
    ```sh 
    colcon build --packages-select [package]
    ```

* build a package with its dependencies
    ```sh 
    colcon build --packages-up-to [package]
    ```

* build all packages under the workspace 
    ```sh 
    colcon build
    ```

### Python
No need to colcon build if using Python

## How to run
### C/C++
```sh 
ros2 run [package] [executable]
```

### Python
```sh 
python3 src/[package]/src/[executable]
```
