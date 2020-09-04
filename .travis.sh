#!/bin/bash

set -e

apt-get update -qq && apt-get install -qq -y -q wget sudo lsb-release gnupg # for docker

# Setup ccache
apt-get install -qq -y -q ccache
export PATH=/usr/lib/ccache:$PATH

#before_install:
# Install ROS
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu xenial main\" > /etc/apt/sources.list.d/ros-latest.list"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update -qq
sudo apt-get install dpkg
sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-desktop-full
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init
rosdep update

#install:
# Create a catkin workspace with the package under test.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s $CI_SOURCE_PATH . # Link the repo we are testing to the new workspace

#before_script:
# Install all dependencies, using wstool and rosdep.
cd ~/catkin_ws/src
wstool init # wstool looks for a ROSINSTALL_FILE defined in before_install.
if [[ -f $ROSINSTALL_FILE ]] ; then wstool merge $ROSINSTALL_FILE ; fi
wstool up
cd ~/catkin_ws
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO # rosdep install package depdencies

#script:
# Compile and test.
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin_make $( [ -f $CATKIN_OPTIONS ] && cat $CATKIN_OPTIONS ) # catkin_make looks for flags on catkin.option file
# catkin build -p1 -j1
# catkin run_tests -p1 -j1
# catkin_test_results --all build
# catkin clean -b --yes
# catkin config --install
# catkin build -p1 -j1