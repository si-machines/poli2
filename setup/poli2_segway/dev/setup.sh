# docker commands to get helper running

sudo apt-get update
sudo apt-get install -yq \
  apt-utils \
  dialog \
  python-catkin-tools \
  wget

source /opt/ros/kinetic/setup.bash
rosdep update

mkdir -p poli2_segway_ws/src
cd poli2_segway_ws/src
catkin_init_workspace

# segway-base specific
git clone https://github.com/StanleyInnovation/segway_v3.git
git clone https://github.com/iralabdisco/ira_laser_tools.git -b kinetic
git clone https://github.com/si-machines/dynamixel-workbench.git
git clone https://github.com/si-machines/dynamixel-workbench-msgs.git
git clone https://github.com/si-machines/wpi_jaco.git -b vector-develop && \
  touch wpi_jaco/jaco_description/CATKIN_IGNORE && \
  touch wpi_jaco/jaco_interaction/CATKIN_IGNORE && \
  touch wpi_jaco/jaco_teleop/CATKIN_IGNORE && \
  touch wpi_jaco/wpi_jaco/CATKIN_IGNORE
# is this segway-specific?
git clone https://github.com/si-machines/poli2

# dependencies for HLP-R
git clone https://github.com/GT-RAIL/rail_manipulation_msgs.git
git clone https://github.com/RIVeR-Lab/epos_hardware.git -b kinetic-devel
git clone https://github.com/ros-drivers/smart_battery_msgs.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
(git config --global advice.detachedHead false && \
 cd DynamixelSDK && git checkout tags/3.5.4 && \
 git config --global advice.detachedHead true)
git clone https://github.com/si-machines/robotiq_85_gripper.git
git clone https://github.com/si-machines/kinova-ros.git -b moe-devel && \
  touch kinova-ros/kinova_moveit/CATKIN_IGNORE
git clone https://github.com/si-machines/wpi_jaco.git -b vector-develop && \
  touch wpi_jaco/jaco_description/CATKIN_IGNORE && \
  touch wpi_jaco/jaco_interaction/CATKIN_IGNORE && \
  touch wpi_jaco/jaco_teleop/CATKIN_IGNORE && \
  touch wpi_jaco/wpi_jaco/CATKIN_IGNORE

# HLPR
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
# The simulator is Vector-only (i.e. Poli1)
# git clone https://github.com/HLP-R/hlpr_simulator.git -b kinetic-devel

rosdep install --from-paths . --ignore-src --rosdistro=kinetic -y

cd ..
catkin build