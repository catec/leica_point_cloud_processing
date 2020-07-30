cmake_minimum_required(VERSION 2.8.3)
project(leica_point_cloud_processing)

set(CMAKE_CXX_STANDARD 11)
set(VTK_LEGACY_SILENT ON)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  tf
  # pcl_conversions
  # pcl_ros
  sensor_msgs
  leica_scanstation_msgs
  leica_scanstation_utils
)

#catkin_python_setup()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
src/Utils.cpp
src/Filter.cpp
# src/Viewer.cpp
include/Viewer.h
src/InitialAlignment.cpp
src/GICPAlignment.cpp
src/BooleanDifference.cpp
src/FODDetector.cpp
src/CADToPointCloud.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(node 
src/node.cpp
)

target_link_libraries(node
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(load_and_publish_clouds 
src/load_and_publish_clouds.cpp
)

target_link_libraries(load_and_publish_clouds
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)