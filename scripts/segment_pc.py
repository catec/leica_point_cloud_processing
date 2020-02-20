#!/usr/bin/env python2

import rospy
import rospkg
import os
import pcl
from pcl import pcl_visualization


def extract_part(point_cloud,radius):
    segmenter = point_cloud.make_segmenter_normals(ksearch=50)

    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_CYLINDER)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(10000)
    segmenter.set_distance_threshold(0.05)
    segmenter.set_radius_limits(radius[0], radius[1])

    # obtain inlier indices and model coefficients
    inlier_indices, coefficients = segmenter.segment()

    extracted_part = pc.extract(inlier_indices, False)
    substracted_part = pc.extract(inlier_indices, True)
    return extracted_part,substracted_part


def get_extraction_radius(part):
    if part=="skin":
        return [1.5,4]
    elif part=="stringer":
        return [0,1.5]


if __name__ == '__main__':
    rospy.init_node('segment_pointcloud')

    rospack = rospkg.RosPack()
    pc_path = rospack.get_path("leica_scanstation") + "/pointclouds/"
    print(pc_path)

    # Load point cloud
    pc = pcl.load_XYZRGB(pc_path + "assembly_downsampled.pcd")
    
    # Extract stringers
    r = get_extraction_radius("stringer")
    stringers_pc,pc = extract_part(pc,r)
    if stringers_pc.is_emp
    stringers_pc2,pc = extract_part(pc,r)

    # Extract skin
    # r = get_extraction_radius("skin")
    # skin_pc = extract_part(pc,r)
    print("Separated")

    # Save segmented pointclouds
    pcl.save(pc,      pc_path + "skin_pc.pcd")
    pcl.save(stringers_pc, pc_path + "stringers_pc.pcd")
    pcl.save(stringers_pc2,      pc_path + "stringers_pc2.pcd")
    print("Saved")

    # Run viewer
    os.system("pcl_viewer " + pc_path + "skin_pc.pcd &")
    os.system("pcl_viewer " + pc_path + "stringers_pc2.pcd &")
    os.system("pcl_viewer " + pc_path + "stringers_pc.pcd")