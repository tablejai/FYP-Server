sudo apt update

sudo apt install keychain python3-pip ros-foxy-tf-transformations ros-foxy-rqt ros-foxy-rqt-common-plugins -y
sudo pip install transforms3d

# reinit rqt
rm ~/.config/ros.org/rqt_gui.ini