# LEICA SCANSTATION C5 #

This repo contains source code to develop a library for controlling Leica Scanstation C5.
This is involved in a ROSIN project.

Meshes and pointclouds are not updated online because they are confidential.

![process](alignment_process.gif)

## Set up ##

* Clone
* Compile

## Run order ##

    rosrun leica_point_cloud_processing_utils main

    rosrun leica_point_cloud_processing input_cloud

    

## Usage ##

    rosrun leica_point_cloud_processing filter_scan_noise conjunto_estranio_scan 5

    rosrun leica_point_cloud_processing filter_scan_floor conjunto_estranio_scan_no_noise 0.8

    roslaunch leica_point_cloud_processing align.launch

    rosrun leica_point_cloud_processing cloud_to_mesh conjunto_estranio_scan_aligned 

    rosrun leica_point_cloud_processing differentiate_point_clouds 0.1

## Hacks ##

    rosrun pcl_ros bag_to_pcd /media/catec/Datos/Bags/rosin_leica/assembly.bag /camera/depth/points /home/catec/catkin_ws/src/leica_point_cloud_processing/pointclouds

    pcl_viewer assembly.pcd

* Downsample pointcloud     -> reduce the number of points of the pc
* Apply passthrough filter  -> cut off values that are either inside or outside a given range (not useful for now)

## Dependencies ##

1. Boost, Flann, Eigen3, OpenNI, OpenNI2

        sudo apt-get install libboost1.58* libflann1.8 libeigen3-dev libopenni-dev libopenni2-dev

2. VTK

        wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz
        tar -xf VTK-7.1.0.tar.gz
        cd VTK-7.1.0 && mkdir build && cd build
        cmake ..
        make                                                                   
        sudo make install

3. PCL >1.8.1

        wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
        tar -xf pcl-1.8.1.tar.gz
        cd pcl-pcl-1.8.1 && mkdir build && cd build
        cmake .. -DBUILD_tools=ON
        make
        sudo make install

4. PCL-ROS

        sudo apt-get install ros-kinetic-pcl-*
        sudo apt-get install pcl-tools

5. PYTHON-PCL

        pip3 install python-pcl

## RESULTS

const Eigen::Vector4f downsampling_leaf_size(0.1f, 0.1f, 0.1f, 0.0f);
[ 0.5099,  0.5881, -0.6278,  -10.92]
[-0.8591,  0.3851, -0.3371,   -3.87]
[0.04355,  0.7112,  0.7016,   -4.63]
[      0,       0,       0,       1]

const Eigen::Vector4f downsampling_leaf_size(0.05f, 0.05f, 0.05f, 0.0f);
[ 0.8794, -0.3988,  0.2599,  -10.46]
[-0.3362, -0.1339,  0.9322, -0.3845]
[ -0.337, -0.9072, -0.2518,  -0.315]
[      0,       0,       0,       1]


## Help ##
Ines M. Lara - imlara@catec.aero
Other community or team contact