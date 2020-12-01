cmake_minimum_required(VERSION 2.8.3)
project(leica_point_cloud_processing)

set(CMAKE_CXX_STANDARD 11)
set(VTK_LEGACY_SILENT ON)
add_compile_options(-std=c++14)

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

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Library that define alignment and fod detection process
add_library(${PROJECT_NAME} 
# include/Viewer.h
src/Utils.cpp
src/Filter.cpp
src/InitialAlignment.cpp
src/GICPAlignment.cpp
src/FODDetector.cpp
src/CADToPointCloud.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Library with statemachine to set process flow
add_library(StateMachine src/LeicaStateMachine.cpp)
target_link_libraries(StateMachine ${PROJECT_NAME})

# Main node where start statemachine
add_executable(node src/node.cpp)
target_link_libraries(node StateMachine)

# Node to load and publish clouds
add_executable(load_clouds src/load_and_publish_clouds.cpp)
target_link_libraries(load_clouds ${PROJECT_NAME})

# Testing
catkin_add_gtest(test_utils test/test_utils.cpp)
target_link_libraries(test_utils ${PROJECT_NAME})

# catkin_add_gtest(test_cad_to_pointcloud test/test_cad_to_pointcloud.cpp)
# target_link_libraries(test_cad_to_pointcloud ${PROJECT_NAME})

# catkin_add_gtest(test_initial_alignment test/test_initial_alignment.cpp)
# target_link_libraries(test_initial_alignment ${PROJECT_NAME})

catkin_add_gtest(test_gicp_alignment test/test_gicp_alignment.cpp)
target_link_libraries(test_gicp_alignment ${PROJECT_NAME})

catkin_add_gtest(test_fod_detector test/test_fod_detector.cpp)
target_link_libraries(test_fod_detector ${PROJECT_NAME})

catkin_add_gtest(test_filter test/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME})

# catkin_add_gtest(test_state_machine test/test_state_machine.cpp)
# target_link_libraries(test_state_machine StateMachine)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_publish_clouds test/test_publish_clouds.test test/test_publish_clouds.cpp)
  target_link_libraries(test_publish_clouds ${PROJECT_NAME})
endif()
