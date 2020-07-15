# LEICA SCANSTATION C5 #

This repo contains source code to develop a library for controlling Leica Scanstation C5.
This is involved in a ROSIN project.

Meshes and pointclouds are not updated online because they are confidential.

![process](alignment_process.gif)

## Set up ##

* Clone
* Compile

## Run ##

    rosrun leica_scanstation_utils main

    rosrun leica_point_cloud_processing input_cloud

    rosrun leica_point_cloud_processing node
    
OR

    rosrun leica_scanstation main

    wineconsole leica_scanstation_sdk_control/release/leica_scanstation_sdk_control_node.exe

    rosrun leica_point_cloud_processing publish_cloud

    rosrun leica_point_cloud_processing node 


## Usage ##

Both clouds should be available in ROS topics: `/cad/cloud` y `/scan/cloud`

Then, the node starts calculating cloud alignment. After GICP, user could do more iterations

    rosservice call /iterate_gicp

If the results are wrong you can undo the last operation

    rosservice call /undo_iteration

When both clouds are finally aligned, ask for the algorithm to look for FODs

    rosservice call /get_fods

## Dependencies ##

1. Boost, Flann, Eigen3, OpenNI, OpenNI2

        sudo apt-get install libboost1.58* libflann1.8 libeigen3-dev libopenni-dev libopenni2-dev

2. VTK > 6

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

6. [leica_scanstation_msgs](https://bitbucket.org/ayr_catec/leica_scanstation_msgs/src/master/)

7. [leica_scanstation_utils](https://bitbucket.org/ayr_catec/leica_scanstation_utils/src/master/)


## Help ##
Ines M. Lara - imlara@catec.aero
Other community or team contact