/**
 * @file Filter.h
 * @copyright Copyright (c) 2020, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#pragma once
#ifndef _FILTER_H
#define _FILTER_H

#include <Utils.h>

class Filter
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
    typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
    
public:
    /**
     * @brief Construct a new Filter object.
     *
     */
    Filter(){};

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
     * @param floor_threshold
     */
    Filter(double leaf_size, double noise_threshold, double floor_threshold);

    /**
     * @brief Destroy the Filter object.
     *
     */
    ~Filter(){};

    /**
     * @brief Set the Leaf Size object.
     *
     * @param new_leaf_size
     */
    void setLeafSize(double new_leaf_size);

    /**
     * @brief Set the Noise Threshold object
     * 
     * @param noise_th 
     */
    void setNoiseThreshold(double noise_th);

    /**
     * @brief Set the Floor Threshold object
     * 
     * @param floor_th 
     */
    void setFloorThreshold(double floor_th);
    
    /**
     * @brief Set the Cloud Center object
     * 
     * @param cloud_center 
     */
    void setCloudCenter(Eigen::Vector3f cloud_center);

    /**
     * @brief Perform defined filter 
     *        \n If noise threshold is specified, filter noise in cloud.
     *        \n If floor threshold is specified, search the floor and remove it from cloud.
     *
     * @param[in] cloud
     * @param[out] cloud_filtered
     */
    void run(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);

    /**
     * @brief Apply Leaf Size to downsample input cloud.
     *
     * @param[in] cloud
     * @param[out] cloud_downsampled
     */
    void downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled);

    /**
     * @brief Apply Noise Threshold to filter noise in cloud.
     *        \n Everything out of a box with given size (threshold) is consider noise.
     *
     * @param[in] cloud
     * @param[out] cloud_filtered
     */
    void filterNoise(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);

    /**
     * @brief Apply Floor Threshold to search floor in cloud and filter it.
     *        \n Floor is consider a plane perpendicular to z axis.
     *
     * @param[in] cloud
     * @param[out] cloud_filtered
     */
    void filterFloor(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);

    /**
     * @brief Filter outlier points to remove noise from cloud
     *
     * @param[in] cloud
     * @param[out] cloud_filtered
     */
    void outlierRemove(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);

    /**
     * @brief Remove points in input_cloud based on substract_cloud. Similar to boolean difference. Store differences in cloud_filtered
     * 
     * @param[in] input_cloud 
     * @param[in] substract_cloud 
     * @param[in] threshold 
     * @param[out] cloud_filtered 
     */
    static void removeFromCloud(PointCloudRGB::Ptr input_cloud, 
                                PointCloudRGB::Ptr substract_cloud,
                                double threshold, 
                                PointCloudRGB::Ptr cloud_filtered);

    /**
     * @brief Get all points from cloud_in that does not appear in indices save them in cloud_out
     * 
     * @param[in] cloud_in 
     * @param[in] cloud_out 
     * @param[out] indices 
     */
    static void extractIndicesNegative(PointCloudRGB::Ptr cloud_in, 
                                      PointCloudRGB::Ptr cloud_out,
                                      pcl::IndicesPtr indices);

    /**
     * @brief Get indices from cloud_ind and save them in cloud_out
     * 
     * @param[in] cloud_in 
     * @param[out] cloud_out 
     * @param[in] indices 
     */
    static void extractIndices(PointCloudRGB::Ptr cloud_in,
                              PointCloudRGB::Ptr cloud_out,
                              pcl::IndicesPtr indices);
    
    /**
     * @brief Get indices from normals cloud_ind and save them in cloud_out
     * 
     * @param[in] normals_in 
     * @param[out] normals_out 
     * @param[in] indices 
     */
    static void extractIndices(PointCloudNormal::Ptr normals_in, 
                              PointCloudNormal::Ptr normals_out,
                              pcl::IndicesPtr indices);           

private:
    /** @brief pcl::VoxelGrid leaf size */
    double leaf_size_;

    /** @brief pcl::CropBox size */
    double noise_threshold_;

    /** @brief point cloud center for Noise filter */
    Eigen::Vector4f cloud_center_;

    /** @brief set to true if specified center. If false, compute cloud centroid */
    bool given_center_;

    /** @brief pcl::SACSegmentation distance threshold */
    double floor_threshold_;
};

#endif
