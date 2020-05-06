cmake_minimum_required(VERSION 2.8.3)
project(leica_scanstation)

find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  laser_geometry
  pcl_conversions
  pcl_conversions
  pcl_ros
  sensor_msgs
)

#catkin_python_setup()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)


# add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
# target_link_libraries(laserscan_to_pointcloud
#   ${catkin_LIBRARIES}
# )