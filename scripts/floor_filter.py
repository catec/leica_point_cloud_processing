#!/usr/bin/env python2

import rospy
import rospkg
import os
import sys
import argparse
import pcl
from pcl import pcl_visualization

parser = argparse.ArgumentParser(description='Extract floor from pointcloud. Save both pointclouds: with and without floor.\nWill be saved on /pointclouds folder')
parser.add_argument('-f', dest='file_name', action='store',help='pointcloud file without extension. File should be on pointcloud folder')
parser.add_argument('-z', dest='z_threshold', action='store',help='send filter only for points with coordinate z below this threshold')
parser.add_argument('--view', dest='view', action='store_true',help='run viewer after floor extraction')
parser.set_defaults(view=False)
args = parser.parse_args()

def diff_z_values(cloud,threshold):
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name("z")
    passthrough.set_filter_limits(-1.0, threshold)

    cloud_filtered = passthrough.filter()
    return cloud_filtered

def extract_floor(point_cloud,threshold):
    point_cloud_filtered = diff_z_values(point_cloud,threshold)
    
    segmenter = point_cloud_filtered.make_segmenter_normals(ksearch=500)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_PLANE)SACMODEL_PERPENDICULAR_PLANE
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(10000)
    segmenter.set_distance_threshold(2.5)

    inlier_indices, coefficients = segmenter.segment()

    floor = point_cloud.extract(inlier_indices, False)
    no_floor = point_cloud.extract(inlier_indices, True)
    return floor,no_floor

def main():
    rospy.init_node('segment_pointcloud')

    rospack = rospkg.RosPack()
    pc_path = rospack.get_path("leica_scanstation") + "/pointclouds/"

    # Load point cloud
    if (len(sys.argv)<2):
        rospy.logerr("Please specify file_name.\n    rosrun leica_scanstation floor_filter.py -h")
        return 0
    else:
        file_name = args.file_name

    if (file_name.endswith(".pcd")):
        rospy.logerr("Please, introduce file name without extension")
        return 0

    print("Opening: " + pc_path + file_name + ".pcd")
    try:
        pc = pcl.load(pc_path + file_name + ".pcd")
    except:
        return 0

    # Get max z to interpret as floor
    z_th = float(args.z_threshold)

    # Extract floor
    print("Separating floor...")
    floor,no_floor = extract_floor(pc,z_th)

    # Save segmented pointclouds
    print("Saving files with and without floor")
    pcl.save(floor,   pc_path + file_name + "_floor.pcd")
    pcl.save(no_floor,pc_path + file_name + "_no_floor.pcd")
    print("Saved")

    if args.view:
        visual = pcl.pcl_visualization.CloudViewing()
        visual.ShowMonochromeCloud(no_floor, b'cloud')

        v = True
        while v:
            v = not(visual.WasStopped())
        

if __name__ == '__main__':
    main()