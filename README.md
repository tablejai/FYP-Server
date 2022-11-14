# FYP ROS SYSTEM

<!-- vscode-markdown-toc -->
* 1. [Create Environment](#CreateEnvironment)
* 2. [Setup Environment](#SetupEnvironment)
	* 2.1. [Install dependencies/requirements](#Installdependenciesrequirements)
	* 2.2. [Setup ROS](#SetupROS)
* 3. [ROS commands/usage](#ROScommandsusage)
	* 3.1. [How to create a package](#Howtocreateapackage)
		* 3.1.1. [C/C++](#CC)
		* 3.1.2. [Python](#Python)
	* 3.2. [How to build](#Howtobuild)
	* 3.3. [How to run](#Howtorun)
		* 3.3.1. [C/C++](#CC-1)
		* 3.3.2. [Python](#Python-1)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->
<!-- This TOC is created by Github Extension "Markdown TOC" -->

##  1. <a name='CreateEnvironment'></a>Create Environment

Two ways to run ROS workspace, either run on docker or run on ec2:
1. Connect to EC2 
    ```bash
    ssh -i FYP-Glove.pem ubuntu@ec2-16-163-138-9.ap-east-1.compute.amazonaws.com
    ```

2. Run Ros2 docker container
    * For MacOS 
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
    * For Ubuntu 
	    ```sh
	    xhost + 127.0.0.1
	    docker run --name fyp_ros2_x11 -it --privileged --net=host --ipc=host \
	    --device=/dev/dri:/dev/dri \
	    -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY" \
	    -v $HOME/.Xauthority:/home/$(id -un)/.Xauthority -e XAUTHORITY=/home/$(id -un)/.Xauthority \
	    -e DOCKER_USER_NAME=$(id -un) \
	    -e DOCKER_USER_ID=$(id -u) \
	    -e DOCKER_USER_GROUP_NAME=$(id -gn) \
	    -e DOCKER_USER_GROUP_ID=$(id -g) \
	    -e ROS_IP=127.0.0.1 \
	    ros:foxy
	    ```

##  2. <a name='SetupEnvironment'></a>Setup Environment

###  2.1. <a name='Installdependenciesrequirements'></a>Install dependencies/requirements
Refer to `setup.sh`

###  2.2. <a name='SetupROS'></a>Setup ROS
```sh
# adding ROS system variables, such as ros2 command line
source /opt/ros/foxy/setup.bash     

## adding custom ROS variables so that ros2 command line understand our packages, such as msgs, augmentor
source /path_to_this_workspace/install/setup.bash  
```

##  3. <a name='ROScommandsusage'></a>ROS commands/usage

###  3.1. <a name='Howtocreateapackage'></a>How to create a package

####  3.1.1. <a name='CC'></a>C/C++
```sh
ros2 pkg create --build-type ament_cmake [package]
```

####  3.1.2. <a name='Python'></a>Python
```sh
ros2 pkg create --build-type ament_python [package]
````

More about creating and configuring the package
1. Create your python code under [package]/[package]/[package.py]
2. Add entry point to setup.py so that ros2 can locate its position
```py
'[node_name] = [package].[script]:main',
```

Links:

* [Create a Python package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* [Entry Point For Python](https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html#python-packages)

###  3.2. <a name='Howtobuild'></a>How to build
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


###  3.3. <a name='Howtorun'></a>How to run
####  3.3.1. <a name='CC-1'></a>C/C++
```sh 
ros2 run [package] [executable]
```

####  3.3.2. <a name='Python-1'></a>Python
```sh 
ros2 run [package] [executable]
# or
python3 src/[package]/src/[executable]
```
