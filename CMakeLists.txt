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
src/utils.cpp
src/filter.cpp
src/viewer.cpp
src/initial_alignment.cpp
src/gicp_alignment.cpp
src/boolean_difference.cpp
src/fod_detector.cpp
src/cad_to_pointcloud.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(node 
src/node.cpp
src/cad_to_pointcloud.cpp
)

target_link_libraries(node
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(publish_cloud 
src/publish_cloud.cpp
)

target_link_libraries(publish_cloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)


# add_executable(input_cloud 
# src/input_cloud.cpp
# src/cad_to_pointcloud.cpp
# )

# target_link_libraries(input_cloud
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${PROJECT_NAME}
# )