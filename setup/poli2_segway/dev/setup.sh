#!/bin/bash
# Install all needed dependencies and code for a development machine working with the
# Segway-base Poli2 platform (i.e. Moe)

sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -yq \
  apt-utils \
  dialog \
  git \
  wget

# install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -yq \
  python-catkin-tools
  ros-kinetic-desktop-full
sudo rosdep init
rosdep update
source /opt/ros/kinetic/setup.bash

# set up the workspace
cd ~
mkdir -p poli2_segway_ws/src
cd poli2_segway_ws/src
catkin_init_workspace

# segway-base specific
git clone https://github.com/StanleyInnovation/segway_v3.git
git clone https://github.com/iralabdisco/ira_laser_tools.git -b kinetic

# dependencies for HLP-R
git clone https://github.com/si-machines/dynamixel-workbench.git
git clone https://github.com/si-machines/dynamixel-workbench-msgs.git
git clone https://github.com/si-machines/wpi_jaco.git -b vector-develop && \
  touch wpi_jaco/jaco_description/CATKIN_IGNORE && \
  touch wpi_jaco/jaco_interaction/CATKIN_IGNORE && \
  touch wpi_jaco/jaco_teleop/CATKIN_IGNORE && \
  touch wpi_jaco/wpi_jaco/CATKIN_IGNORE
git clone https://github.com/GT-RAIL/rail_manipulation_msgs.git
git clone https://github.com/RIVeR-Lab/epos_hardware.git -b kinetic-devel
git clone https://github.com/ros-drivers/smart_battery_msgs.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
(git config --global advice.detachedHead false && \
 cd DynamixelSDK && git checkout tags/3.5.4 && \
 git config --global advice.detachedHead true)
git clone https://github.com/si-machines/robotiq_85_gripper.git
git clone https://github.com/si-machines/kinova-ros.git -b moe-devel && \
  touch kinova-ros/kinova_moveit/CATKIN_IGNORE && \
  touch kinova-ros/kinova_moveit_demo/kinova_arm_moveit_demo/CATKIN_IGNORE

# HLP-R
git clone https://github.com/HLP-R/hlpr_common.git -b kinetic-devel
git clone https://github.com/HLP-R/hlpr_robots.git -b kinetic-devel && \
  touch hlpr_robots/hlpr_poli/CATKIN_IGNORE
git clone https://github.com/HLP-R/hlpr_kinesthetic_teaching.git -b kinetic-devel
# don't use robot configurations that use the vector base
git clone https://github.com/HLP-R/hlpr_manipulation.git -b kinetic-devel && \
  touch hlpr_manipulation/hlpr_manipulation/CATKIN_IGNORE && \
  touch hlpr_manipulation/hlpr_wpi_jaco_moveit_config/CATKIN_IGNORE && \
  touch hlpr_manipulation/hlpr_j2s7s300_moveit_config/CATKIN_IGNORE && \
  touch hlpr_manipulation/hlpr_wpi_jaco_moveit_config_two_arms/CATKIN_IGNORE
git clone https://github.com/HLP-R/hlpr_speech.git -b kinetic-devel
git clone https://github.com/HLP-R/hlpr_lookat.git -b kinetic-devel
# The simulator is Vector-only (i.e. Poli1)...for now
# git clone https://github.com/HLP-R/hlpr_simulator.git -b kinetic-devel

# Poli 2 platform code
git clone https://github.com/si-machines/poli2

# install all needed dependencies
rosdep install --from-paths . --ignore-src --rosdistro=kinetic -y

# post-installation steps
sudo apt-get purge ros-kinetic-dynamixel-workbench-toolbox
sudo wget https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/hector_localization/catkin/hector_pose_estimation/hector_pose_estimation_nodelets.xml -P /opt/ros/kinetic/share/hector_pose_estimation/
sudo rm /etc/udev/rules.d/10-local.rules
sudo ln -s ~/catkin_ws/src/poli2/setup/poli2_segway/machine1/udev.rules /etc/udev/rules.d/10-local.rules

# add shortcuts to bashrc
echo "
# Include handy Poli2-specific command line aliases, such as opening the gripper.
# This line was added by the Poli2 machine1 setup.sh script.
source `pwd`/poli2/setup/shortcuts.bash" >> ~/.bashrc

# now we can build
cd ..
catkin build
source devel/setup.bash
