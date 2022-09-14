# FYP ROS SYSTEM

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
1. Create your python code under [package]/[package]/[package.py]
2. Add entry point to setup.py so that ros2 can locate its position
```py
'[package] = [package].[package]:main',
```

Links:
[Create a Python package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
[Entry Point For Python](https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html#python-packages)

## How to build
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
