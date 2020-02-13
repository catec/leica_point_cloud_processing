# LEICA SCANSTATION C5 #

This repo contains source code to develop a library for controlling Leica Scanstation C5.
This is involved in a ROSIN project.

Meshes and pointclouds are not updated online because they are confidential.

## Set up ##

* Clone
* Compile

## Usage ##

    roslaunch leica_scanstation system_spawn.launch

    rosrun leica_scanstation downsample_pc.py
    
    rosrun leica_scanstation segment_pc.py
    
    rosrun leica_scanstation segment_and_publish_pcs.py

## Hacks ##

    rosrun pcl_ros bag_to_pcd /media/catec/Datos/Bags/rosin_leica/assembly.bag /camera/depth/points /home/catec/catkin_ws/src/leica_scanstation/pointclouds

    pcl_viewer assembly.pcd

* Downsample pointcloud     -> reduce the number of points of the pc
* Apply passthrough filter  -> cut off values that are either inside or outside a given range (not useful for now)

## Help ##
Ines M. Lara - imlara@catec.aero
Other community or team contact