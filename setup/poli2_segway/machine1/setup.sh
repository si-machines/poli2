#!/bin/bash

# Set up the #1 machine on a Segway Poli2

sudo wget https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/hector_localization/catkin/hector_pose_estimation/hector_pose_estimation_nodelets.xml -P /opt/ros/kinetic/share/hector_pose_estimation/
sudo rm /etc/udev/rules.d/10-local.rules
sudo ln -s ~/catkin_ws/src/poli2/setup/poli2_segway/machine1/udev.rules /etc/udev/rules.d/10-local.rules