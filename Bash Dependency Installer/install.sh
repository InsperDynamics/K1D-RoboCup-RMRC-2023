#!/bin/bash

# Before using the script make sure cmake and pip3 are up-to-date
# Before using the script make sure internet connection is ok

cd ~

sudo apt update
sudo apt upgrade

sudo apt install curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
apt search ros-noetic
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python3-catkin-tools python3-osrf-pycommon
sudo rosdep init
rosdep update
cd ~

### donwload SDL 2.26.3 Release from github
### https://github.com/libsdl-org/SDL/releases/tag/release-2.26.3
### extract the zip file and enter the folder
mkdir build
cd build
../configure
make
sudo make install
cd ~

sudo apt install cmake pkg-config
sudo apt-get install python swig

sudo apt-get install libzbar-dev

sudo apt-get install libcanberra-gtk-module

sudo apt install sox

sudo snap install arduino

sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial

sudo apt update
sudo apt install software-properties-common apt-transport-https wget
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt install code

wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Uncomment if pc is 64 bits
#sudo apt-get install libncurses5-dev:i386

cd ~
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip
# Create build directory and switch into it
mkdir -p build && cd build
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
# Build
cmake --build .
# Install
sudo make install

sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
sudo apt-get install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench*

sudo reboot

# After running this script, you can setup ros workspace, upload the OpenCR code and recompile the machine vision interface.

# copy and paste the following link to the Additional Boards Manager URLs textbox in order to add OpenCR board to Arduino IDE:
#https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
