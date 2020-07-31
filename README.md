# LEICA POINT CLOUD PROCESSING #

This package has been developed to help identifying [FODs](https://www.fodcontrol.com/what-is-fod/) in aeronautical structures. 

Using point cloud analysis techniques, this software aims to compare the current state of the structure with its CAD to identify possible foreign objects. 

It has been designed to be used in combination with the [leica_scanstation](https://github.com/fada-catec/leica_scanstation) package, which allows to control the scanstation to make a scan of an aeronautical part. When the scan is finished, you get the point cloud that proceeds to be analyzed. 

It is involved in the ROSIN project [Large_3D_inspection](http://wiki.ros.org/large_3d_inspection)

NOTE: the following release is an alpha experimental release corresponding to Milestone 1 of the ROSIN project.

## Set up ##
1. Create a workspace and clone *leica_scanstation* (for listed dependencies)

        mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
        git clone https://github.com/fada-catec/leica_scanstation.git

2. Remove package *leica_scanstation_ros* to avoid compilation errors.

        rm -r leica_scanstation/leica_scanstation_ros

3. Clone this repo and Compile

        cd ~/catkin_ws/src
        git clone https://github.com/fada-catec/leica_point_cloud_processing.git
        cd ~/catkin_ws/
        catkin_make

## Nodes ##

**load_and_publish_clouds** node will load both scanned and CAD clouds into ROS topics: `/cad/cloud` and `/scan/cloud`. This is done after calling rosservice `/publish_clouds`.

Make sure pointcloud files are on the correct folder, specified in ROS param server as `/pointcloud_folder` (default: package leica_scanstation_utils/pointclouds).

Supported formats: `.obj` for CAD files and `.pcd` for scanned files. NOTE: both files *must* have the same name.

**node** is the main node that perform alignment and FOD detection. It opens subscribers to clouds topics and start process.

## Usage ##

If you plan to use it with Leica Scanstation C5 visit [ros.wiki](http://wiki.ros.org/leica_scanstation) to get more info. 

    rosrun leica_scanstation_utils main

    rosrun leica_point_cloud_processing load_and_publish_clouds

    rosrun leica_point_cloud_processing node

    rosservice call /publish_clouds "file_name: '{file}'"

Both clouds should be available in ROS topics: `/cad/cloud` and `/scan/cloud`

Then, the node starts calculating cloud alignment. After GICP, user could do more iterations

    rosservice call /iterate_gicp

If the results are wrong you can undo the last operation

    rosservice call /undo_iteration

When both clouds are finally aligned, ask for the algorithm to look for FODs

    rosservice call /get_fods

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

        sudo apt-get install ros-kinetic-pcl-*
        sudo apt-get install pcl-tools

6. PYTHON-PCL

        pip3 install python-pcl

7. [leica_scanstation_msgs](https://github.com/fada-catec/leica_scanstation/tree/master/leica_scanstation_msgs)

8. [leica_scanstation_utils](https://github.com/fada-catec/leica_scanstation/tree/master/leica_scanstation_utils)

## Code API ##

[Topics, Services](http://wiki.ros.org/leica_point_cloud_processing#Code_API)

Read documentation `leica_point_cloud_processing/doc/html/index.html`

## Help ##
Ines M. Lara - imlara@catec.aero