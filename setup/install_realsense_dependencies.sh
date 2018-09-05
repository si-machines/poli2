#!/bin/bash

# This script performs the instructions to install dependencies 
# from https://github.com/intel-ros/realsense.
# It assumes that ROS Kinetic is installed.

sudo apt-get install -yq ros-kinetic-rgbd-launch

sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo rm -f /etc/apt/sources.list.d/realsense-public.list.

sudo apt-get update
sudo apt-get install -yq librealsense2*

echo "Realsense Installation complete. You can run realsense-viewer"
echo "to verify the installation. Also, if the installation was successful,"
echo "the following line should contain the string 'realsense':"
modinfo uvcvideo | grep "version:"
