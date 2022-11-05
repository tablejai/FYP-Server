sudo apt update

# Install Universe for finding Foxy
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS-Foxy repository public key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install dependencies for foxy and foxy itself
sudo apt install keychain python3-pip ros-foxy-tf-transformations ros-foxy-rqt ros-foxy-rqt-common-plugins ros-foxy-desktop python3-argcomplete -y

# Install colcon for python
sudo apt install python3-colcon-common-extensions

# Install flask
python3 -m pip install Flask==2.1.2 Flask-Classful==0.14.2 Werkzeug==2.1.2 transforms3d

# reinit rqt
rm ~/.config/ros.org/rqt_gui.ini
