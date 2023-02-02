set -x
sudo apt update

# Install Tools
sudo apt install\
    curl\
    vim\
    keychain\
    python3-pip\
    python3-argcomplete\
    -y

# Install Universe for finding Foxy
sudo apt install software-properties-common -y
sudo add-apt-repository universe

# Add ROS-Foxy repository public key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install dependencies for foxy and foxy itself
sudo apt install\
    ros-foxy-desktop\
    ros-foxy-rqt\
    ros-foxy-rqt-common-plugins\
    ros-foxy-tf2-tools\
    ros-foxy-tf-transformations\
    ros-foxy-ros2bag\
    python3-colcon-common-extensions\
    ros-foxy-ros2bag\
    ros-foxy-rosbag2-storage-default-plugins\
    -y

# Install ros 3rd party packages
python3 -m pip install transforms3d rosbags

# Install flask
python3 -m pip install Flask==2.1.2 Flask-Classful==0.14.2 Werkzeug==2.1.2 

# Install pandas
python3 -m pip install tensorflow pandas numpy pyautogui

# reinit rqt
file="~/.config/ros.org/rqt_gui.ini"
if [ -f "$file" ] ; then
    rm "$file"
fi