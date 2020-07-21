# LEICA SCANSTATION C5 #

This repo contains source code to develop a library for controlling Leica Scanstation C5.
This is involved in a ROSIN project.

Meshes and pointclouds are not updated online because they are confidential.

![process](alignment_process.gif)

## Set up ##

* Clone
* Compile

## Files ##

### node.cpp
This node will perfom alignment between CAD and scanned clouds. 
1. Create subscriber for cloud topics `/cad/cloud` and `/scan/cloud`. 
2. Perform alignment between both clouds.
3. Publish aligned cloud.
4. Let user modify result as specified in [Usage](##usage)

### load_and_publish_clouds.cpp
This node will load and publish clouds on topic read by [node](##nodecpp).
- Make sure cloud files are on the correct folder (default: package leica_scanstation_utils/pointclouds)
- Supported formats: `.obj` for CAD file, `.pcd` for scan file.
- Publisher start when service is called:
    - Automatically done by leica_scanstation_sdk_control_node when scan is finished
    - Manually call by user

            rosservice call /publish_clouds "file_name: '{file}'"

## Run ##

    rosrun leica_scanstation_utils main

    rosrun leica_point_cloud_processing node

    rosrun leica_point_cloud_processing load_and_publish_clouds

If using Leica Scanstation C5

    wineconsole leica_scanstation_sdk_control/release/leica_scanstation_sdk_control_node.exe

## Usage ##

Both clouds should be available in ROS topics: `/cad/cloud` and `/scan/cloud`

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