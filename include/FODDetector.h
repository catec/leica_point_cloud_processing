// fod_detector.h
#pragma once
#ifndef _FOD_DETECTOR_H
#define _FOD_DETECTOR_H

#include <Utils.h>

#include <stdlib.h> 
#include <string>
#include <pcl/segmentation/extract_clusters.h>

#endif

class FODDetector 
{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

public:

    /**
     * @brief Construct a new FODDetector object with given resolution.
     * 
     * @param resolution 
     */
    FODDetector(double resolution);

    /**
     * @brief Destroy the FODDetector object
     * 
     */
    ~FODDetector() {};

    /**
     * @brief Extract cluster indices from input cloud with resolution set. Each cluster is a possible FOD.
     * 
     * @param[in] cloud 
     * @param[out] cluster_indices 
     */
    void clusterPossibleFODs(PointCloudRGB::Ptr cloud,
                             std::vector<pcl::PointIndices> &cluster_indices);

    /**
     * @brief Save each cluster (possible FOD) in a PointCloud2 msg. Return an array of all generated msgs.
     *        \n Returns the number of possible FODs.
     * 
     * @param[in] cluster_indices 
     * @param[in] cloud 
     * @param[out] cluster_msg_array 
     * @return int number_of_fod
     */
    int clusterIndicesToROSMsg(std::vector<pcl::PointIndices> cluster_indices,
                               PointCloudRGB::Ptr cloud,
                               std::vector<sensor_msgs::PointCloud2> &cluster_msg_array);

private:

    /** @brief Resolution to set as cluster Tolerance. */
    double _voxel_resolution;
};