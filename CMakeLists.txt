cmake_minimum_required(VERSION 2.8.3)
project(leica_scanstation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

#catkin_python_setup()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
