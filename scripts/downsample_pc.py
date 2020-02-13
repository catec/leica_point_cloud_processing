#!/usr/bin/env python

import rospy
import rospkg
import os
import pcl

if __name__ == '__main__':
    rospy.init_node('downsample_pointcloud')

    rospack = rospkg.RosPack()
    pc_path = rospack.get_path("leica_scanstation") + "/pointclouds/"
    print(pc_path)

    # Load point cloud
    pc = pcl.load_XYZRGB(pc_path + "assembly.pcd")

    # Set resolution for downsampling
    LEAF_SIZE = 0.01

    # Apply Voxel Grid Filter
    voxel_grid = pc.make_voxel_grid_filter()
    voxel_grid.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    print("Filtered")

    # Get and save downsampled pointcloud
    downsampled_pc = voxel_grid.filter()
    pcl.save(downsampled_pc, pc_path + "assembly_downsampled.pcd")
    print("Saved")

    # Run viewer
    os.system("pcl_viewer " + pc_path + "assembly_downsampled.pcd")