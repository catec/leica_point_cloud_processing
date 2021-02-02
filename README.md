# LEICA POINT CLOUD PROCESSING #

[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build Status](https://travis-ci.com/fada-catec/leica_point_cloud_processing.svg?branch=master)](https://travis-ci.com/fada-catec/leica_point_cloud_processing)
[![codecov](https://codecov.io/gh/fada-catec/leica_point_cloud_processing/branch/master/graph/badge.svg?token=CKF4OU74PZ)](https://codecov.io/gh/fada-catec/leica_point_cloud_processing)

This package has been developed to help identifying [FODs](https://www.fodcontrol.com/what-is-fod/) in aeronautical structures. 

Using point cloud analysis techniques, this software aims to compare the current state of the structure with previous state to identify possible foreign objects. It opens up the posibility of applying a pre-scan in which it is guaranteed to be free of FODs or a CAD of the structure that is ideal and free of fods. 

It has been designed to be used in combination with the [leica_scanstation](https://github.com/fada-catec/leica_scanstation) package, which allows to control the scanstation to make a scan of an aeronautical part. When the scan is finished, you get the point cloud that proceeds to be analyzed. In order to reproduce this behaviour if the device is not available, we created a [simulator](https://github.com/fada-catec/leica_gazebo_simulator).

It is involved in the ROSIN project [Large_3D_inspection](http://wiki.ros.org/large_3d_inspection)

## Set up ##
1. Create a workspace and clone *leica_scanstation* (for listed dependencies)

        mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
        git clone https://github.com/fada-catec/leica_scanstation.git

2. Clone this repo

        cd ~/catkin_ws/src
        git clone https://github.com/fada-catec/leica_point_cloud_processing.git

3. Compile (be careful: package *leica_scanstation_ros* is meant to be compiled on Windows) 

        cd ~/catkin_ws/
        catkin_make -DCATKIN_BLACKLIST_PACKAGES="leica_scanstation_ros"

## Nodes ##

**load_clouds** node will load both scanned and CAD clouds into ROS topics: `/target/cloud` and `/source/cloud`. This is done after calling rosservice `/publish_clouds`.

Make sure pointcloud files are on the correct folder, specified in ROS param server as `/pointcloud_folder` (default: package leica_scanstation_utils/pointclouds).

Supported formats: `.obj` and `.ply` for CAD files and `.pcd` for scanned files.

**node** is the main node that perform alignment and FOD detection. It opens subscribers to clouds topics and start process. It's procedure is based on a Finite State Machine with the states that are shown in the [package wiki](http://wiki.ros.org/leica_point_cloud_processing#Workflow).

## Pre Alignment process ##

The process of detecting FODs is based on comparing two pointclouds. Thus, the first step is to get both clouds registered [PCL Registration API](https://pointclouds.org/documentation/tutorials/registration_api.html). This is achieved by a pre alignment process and an iterative algorithm called GICP. The pre alignment step reduces the GICP process time but may vary it's effectiveness depending on the input clouds. In this package, four different methods of alignment are presented. The prefered method must be set on launch before starting inspection procedure. 

- **HARRIS**. This method is focused on finding corners and set them as keypoints.
- **BOUNDARY**. This method detects all parts edges and set them as keypoints.
- **MULTISCALE**. This method is able to find keypoints that keeps relevance on different scales. It is recommended when compairing cloud with a previous scan.
- **NORMALS**. This is an in-house method based on relation between point clouds normals. It finds correspondences from orientation of dominant normals. 
- **NONE**. It is possible to avoid pre alignment process if both clouds are close enough and start inmediatly GICP.

## Usage ##

1. Tell load_clouds to publish clouds. Clouds should be available in path specified in param `/pointcloud_folder`. You can use [examples pointclouds](https://github.com/fada-catec/leica_scanstation/tree/master/leica_scanstation_utils/pointclouds).

        rosrun leica_point_cloud_processing load_clouds

        rosservice call /publish_clouds "source_cloud_file: 'scan_fods.pcd' target_cloud_file: 'cad.ply'" 

2. Launch the state machine node. Inspection process starts if both clouds are available in ROS topics: `/target/cloud` and `/source/cloud`. As the target cloud indicated in example above comes from a CAD file, set param `using_CAD` to true.

        roslaunch leica_point_cloud_processing leica_point_cloud_processing.launch using_CAD:=true

3. When FOD detection process ends, results clouds are available in ROS topics. You can visualize detected FODs as pointclouds in RViz.

## Dependencies ##

1. [ROS for Ubuntu](http://wiki.ros.org/Installation/Ubuntu)

2. Boost, Flann, Eigen3, OpenNI, OpenNI2

        sudo apt-get install libboost1.58* libflann1.8 libeigen3-dev libopenni-dev libopenni2-dev

3. VTK > 6

        wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz
        tar -xf VTK-7.1.0.tar.gz
        cd VTK-7.1.0 && mkdir build && cd build
        cmake ..
        make                                                                   
        sudo make install

4. PCL >1.8.1

        wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
        tar -xf pcl-1.8.1.tar.gz
        cd pcl-pcl-1.8.1 && mkdir build && cd build
        cmake .. -DBUILD_tools=ON
        make
        sudo make install

5. PCL-ROS

        sudo apt-get install ros-$ROS_DISTRO-pcl-*
        sudo apt-get install pcl-tools

6. PYTHON-PCL

        pip3 install python-pcl

7. [leica_scanstation_msgs](https://github.com/fada-catec/leica_scanstation/tree/master/leica_scanstation_msgs)

8. [leica_scanstation_utils](https://github.com/fada-catec/leica_scanstation/tree/master/leica_scanstation_utils)

## Code API ##

[Topics, Services](http://wiki.ros.org/leica_point_cloud_processing#Code_API)

Read documentation `leica_point_cloud_processing/doc/html/index.html`

##  Acknowledgement ##
***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 

## Help ##
Ines M. Lara - imlara@catec.aero
