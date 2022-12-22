#!/bin/bash

cd ~
mkdir --parents catkin_ws/src
cd catkin_ws
catkin init
sudo apt-get install ros-noetic-catkin
echo "source ~/opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-hls-lfcd-lds-driver
sudo chmod a+rw /dev/ttyUSB0
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
cd ..
catkin_make