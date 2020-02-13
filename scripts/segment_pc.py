#!/usr/bin/env python2

import rospy
import rospkg
import os
import pcl
from pcl import pcl_visualization

if __name__ == '__main__':
    rospy.init_node('downsample_pointcloud')

    rospack = rospkg.RosPack()
    pc_path = rospack.get_path("leica_scanstation") + "/pointclouds/"
    print(pc_path)

    # Load point cloud
    pc = pcl.load_XYZRGB(pc_path + "assembly_downsampled.pcd")

    # Use cylinder segmentation to separate skin from stringers
    segmenter = pc.make_segmenter_normals(ksearch=50)

    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_CYLINDER)
    # segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(10000)
    segmenter.set_distance_threshold(0.05)
    segmenter.set_radius_limits(1.5, 4)

    # obtain inlier indices and model coefficients
    inlier_indices, coefficients = segmenter.segment()

    # extract separated objects from pointcloud
    skin_pc = pc.extract(inlier_indices, False)
    stringers_pc = pc.extract(inlier_indices, True)
    print("Separated")

    # Save segmented pointclouds
    pcl.save(skin_pc,      pc_path + "skin_pc.pcd")
    pcl.save(stringers_pc, pc_path + "stringers_pc.pcd")
    print("Saved")

    # Run viewer
    os.system("pcl_viewer " + pc_path + "skin_pc.pcd &")
    os.system("pcl_viewer " + pc_path + "coloured_stringers_pc.pcd")