# LEICA SCANSTATION C5 #

This repo contains source code to develop a library for controlling Leica Scanstation C5.
This is involved in a ROSIN project.

Meshes and pointclouds are not updated online because they are confidential.

## Set up ##

* Clone
* Compile

## Usage ##

    roslaunch leica_scanstation system_spawn.launch

    roslaunch leica_scanstation clustering.launch

    rosrun leica_scanstation downsample_pc.py
    
    rosrun leica_scanstation segment_pc.py

    rosrun leica_scanstation segment_and_publish_pcs.py

## Hacks ##

    rosrun pcl_ros bag_to_pcd /media/catec/Datos/Bags/rosin_leica/assembly.bag /camera/depth/points /home/catec/catkin_ws/src/leica_scanstation/pointclouds

    pcl_viewer assembly.pcd

* Downsample pointcloud     -> reduce the number of points of the pc
* Apply passthrough filter  -> cut off values that are either inside or outside a given range (not useful for now)

## Dependencies ##

* PCL:

    1. Boost

        sudo apt-get install libboost1.58*

    2. Flann

        sudo apt-get install libflann1.8

    3. Eigen3

        sudo apt-get install libeigen3-dev

    4. VTK

        wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz
        tar -xf VTK-7.1.0.tar.gz
        cd VTK-7.1.0 && mkdir build && cd build
        cmake ..
        make                                                                   
        sudo make install

    5. PCL y PCL-ROS

        sudo apt-get install ros-kinetic-pcl-*

        wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
        tar -xf pcl-1.8.0.tar.gz
        cd pcl-pcl-1.8.0 && mkdir build && cd build
        cmake ..
        make
        sudo make install

* PYTHON-PCL

    pip3 install python-pcl

## Help ##
Ines M. Lara - imlara@catec.aero
Other community or team contact