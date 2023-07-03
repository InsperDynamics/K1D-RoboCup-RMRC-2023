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
sudo apt-get install ros-noetic-hls-lfcd-lds-driver
sudo apt-get install ros-noetic-hector-slam
sudo chmod a+rw /dev/ttyUSB0
cd ~/catkin_ws/src
cd ..
catkin_make