#!/bin/bash

set -e

echo "Installing utilities..."
echo
apt-get update -q && \
	apt-get install \
       	git \
	ros-melodic-rqt-joint-trajectory-controller \
	-y -q --no-install-recommends 

echo
echo "Downloading UR deps source..."
echo
set +e
cd ~
mkdir -p ~/ur_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git ~/ur_ws/src/Universal_Robots_ROS_Driver
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git ~/ur_ws/src/fmauch_universal_robot

echo
echo "Installing deps to build UR driver..."
echo
set -e
source /opt/ros/melodic/setup.bash && \
rosdep update && rosdep install --from-paths ~/ur_ws/src --ignore-src -y

echo
echo "Building UR drivers..."
echo
cd ~/ur_ws
source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release

echo
echo "Finished install"
echo

echo "Fixing 'stuff'"
rosdep --fix-permissions
echo "source ~/ur_ws/devel/setup.bash" >> ~/.bashrc
source ~/ur_ws/devel/setup.bash


