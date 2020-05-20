cmake_minimum_required(VERSION 2.8.3)
project(leica_scanstation)

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
)

#catkin_python_setup()

catkin_package()

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add_library(cad_to_pointcloud src/cad_to_pointcloud.cpp)
# target_link_libraries(cad_to_pointcloud
#   ${catkin_LIBRARIES}
# )

add_executable(3dscan_to_cad_align 
src/3dscan_to_cad_align.cpp
src/cad_to_pointcloud.cpp
)

target_link_libraries(3dscan_to_cad_align
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
# target_link_libraries(laserscan_to_pointcloud
#   ${catkin_LIBRARIES}
# )