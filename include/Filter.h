// filter.h
#pragma once
#ifndef _FILTER_H
#define _FILTER_H

#include <Utils.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "pcl/common/angles.h"
#include "pcl/segmentation/sac_segmentation.h"

#endif


class Filter {


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


public:

    /**
     * @brief Construct a new Filter object.
     * 
     */
    Filter();

    /**
     * @brief Construct a new Filter object.
     * 
     * @param leaf_size 
     */
    Filter(double leaf_size);

    /**
     * @brief Construct a new Filter object.
     * 
     * @param leaf_size 
     * @param noise_threshold 
     */
    Filter(double leaf_size, double noise_threshold);

    /**
     * @brief Construct a new Filter object.
     * 
     * @param leaf_size 
     * @param noise_threshold 
     * @param floor_threshold 
     */
    Filter(double leaf_size, double noise_threshold, double floor_threshold);

    Filter(Eigen::Vector3f cloud_center, double leaf_size, double noise_threshold, double floor_threshold);
    
    /**
     * @brief Destroy the Filter object.
     * 
     */
    ~Filter() {};


    /** @brief pcl::VoxelGrid leaf size */
    double _leaf_size;

    /** @brief pcl::CropBox size */
    double _noise_filter_threshold;

    /** @brief pcl::SACSegmentation distance threshold */
    double _floor_filter_threshold;

    Eigen::Vector3f _cloud_center;

    bool _user_given_center;

    /**
     * @brief Set the Leaf Size object.
     * 
     * @param new_leaf_size 
     */
    void setLeafSize(double new_leaf_size);

    /**
     * @brief Downsample cloud. 
     *        \n If noise threshold is specified, filter noise in cloud.
     *        \n If floor threshold is specified, search the floor and remove it from cloud.
     * 
     * @param[in] cloud 
     * @param[out] cloud_filtered 
     */
    void run(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);


private:

    /**
     * @brief Apply _leaf_size to downsample input cloud.
     * 
     * @param[in] cloud 
     * @param[out] cloud_downsampled 
     */
    void downsampleCloud(PointCloudRGB::Ptr cloud, 
                         PointCloudRGB::Ptr cloud_downsampled);

    /**
     * @brief Apply _noise_filter_threshold to filter noise in cloud.
     *        \n Everything out of a box with given size (threshold) is consider noise.
     * 
     * @param[in] threshold 
     * @param[in] cloud 
     * @param[out] cloud_filtered 
     */
    void filter_noise(double threshold, 
                      PointCloudRGB::Ptr cloud, 
                      PointCloudRGB::Ptr cloud_filtered);

    void filter_noise(Eigen::Vector3f cloud_center, 
                      double threshold, 
                      PointCloudRGB::Ptr cloud, 
                      PointCloudRGB::Ptr cloud_filtered);

    /**
     * @brief Apply _floor_filter_threshold to search floor in cloud and filter it.
     *        \n Floor is consider a plane perpendicular to z axis.
     * 
     * @param[in] threshold 
     * @param[in] cloud 
     * @param[out] cloud_filtered 
     */
    void filter_floor(double threshold, 
                      PointCloudRGB::Ptr cloud, 
                      PointCloudRGB::Ptr cloud_filtered);
};