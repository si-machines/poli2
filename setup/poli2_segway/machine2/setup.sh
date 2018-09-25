#!/bin/bash

# Set up the #2 machine on a Segway Poli2


####################################
# these commands are common to all robots and computers and should be standardized
# in another file
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -yq \
  apt-utils \
  dialog \
  libusb-1.0-0 \
  python-catkin-tools \
  python-wstool
  wget

# create a custom desktop background based on the computer hostname
convert -size 1920x1080 xc:"#222222" -font "Ubuntu" -pointsize 128 -fill white -annotate +100+1050 "`hostname`" ${HOME}/`hostname`_bg_image.png
gsettings set org.gnome.desktop.background picture-uri "file://${HOME}/`hostname`_bg_image.png"

source /opt/ros/kinetic/setup.bash
rosdep update

cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace

####################################

# download and install everything in one fell swoop
wget https://raw.githubusercontent.com/si-machines/poli2/master/setup/poli2_segway/machine1/poli2_segway_machine2.rosinstall
wstool init ./ poli2_segway_machine2.rosinstall

# gitignore some packages that won't build without vector. Grouped by repository
touch kinova-ros/kinova_moveit/CATKIN_IGNORE && \
  touch kinova-ros/kinova_moveit_demo/kinova_arm_moveit_demo/CATKIN_IGNORE

# install all needed dependencies
rosdep install --from-paths . --ignore-src --rosdistro=kinetic -y

# post-installation steps
sudo apt-get purge ros-kinetic-dynamixel-workbench-toolbox
sudo wget https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/hector_localization/catkin/hector_pose_estimation/hector_pose_estimation_nodelets.xml -P /opt/ros/kinetic/share/hector_pose_estimation/

# now build
cd ..
catkin build