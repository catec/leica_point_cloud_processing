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

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "pcl/common/angles.h"
#include "pcl/segmentation/sac_segmentation.h"

#endif

class Filter
{
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
  ~Filter(){};

  /** @brief pcl::VoxelGrid leaf size */
  double leaf_size_;

  /** @brief pcl::CropBox size */
  double noise_filter_threshold_;

  /** @brief pcl::SACSegmentation distance threshold */
  double floor_filter_threshold_;

  Eigen::Vector3f cloud_center_;

  bool user_given_center_;

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
   * @brief Apply leaf_size_ to downsample input cloud.
   *
   * @param[in] cloud
   * @param[out] cloud_downsampled
   */
  void downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled);

  /**
   * @brief Apply noise_filter_threshold_ to filter noise in cloud.
   *        \n Everything out of a box with given size (threshold) is consider noise.
   *
   * @param[in] threshold
   * @param[in] cloud
   * @param[out] cloud_filtered
   */
  void filter_noise(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);

  void filter_noise(Eigen::Vector3f cloud_center, double threshold, PointCloudRGB::Ptr cloud,
                    PointCloudRGB::Ptr cloud_filtered);

  /**
   * @brief Apply floor_filter_threshold_ to search floor in cloud and filter it.
   *        \n Floor is consider a plane perpendicular to z axis.
   *
   * @param[in] threshold
   * @param[in] cloud
   * @param[out] cloud_filtered
   */
  void filter_floor(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);
};