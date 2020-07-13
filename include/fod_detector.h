// fod_detector.h
#pragma once
#ifndef _FOD_DETECTOR_H
#define _FOD_DETECTOR_H

#include <stdlib.h> 
#include <string>
#include <pcl_ros/point_cloud.h> 
#include <pcl/segmentation/extract_clusters.h>

#endif

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class FODDetector 
{
    public:
        FODDetector(double resolution);
        ~FODDetector() {};

        void clusterPossibleFODs(PointCloudRGB::Ptr cloud,
                                 std::vector<pcl::PointIndices> &cluster_indices);
        int clusterIndicesToROSMsg(std::vector<pcl::PointIndices> cluster_indices,
                                   PointCloudRGB::Ptr cloud,
                                   std::vector<sensor_msgs::PointCloud2> &cluster_msg_array);

    private:
        double _voxel_resolution;
};