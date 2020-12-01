/**
 * @file FODDetector.h
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
#ifndef _FOD_DETECTOR_H
#define _FOD_DETECTOR_H

#include <Utils.h>

class FODDetector
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

public:
  /**
   * @brief Construct a new FODDetector object with given tolerance and minimal fod points.
   *
   * @param tolerance
   */
  FODDetector(PointCloudRGB::Ptr cloud, double cluster_tolerance, double min_fod_points);

  /**
   * @brief Destroy the FODDetector object
   *
   */
  ~FODDetector(){};

  /**
   * @brief Set the Cluster Tolerance object
   * 
   * @param tolerance 
   */
  void setClusterTolerance(double tolerance);

  /**
   * @brief Set the Min FOD points object
   * 
   * @param min_fod_points 
   */
  void setMinFODpoints(double min_fod_points);

  /**
   * @brief Extract cluster indices from input cloud with resolution set. Each cluster is a possible FOD.
   *
   * @param[in] cloud
   * @param[out] cluster_indices
   */
  void clusterPossibleFODs();

  /**
   * @brief Get the FOD indices computed in clusterPossibleFODs()
   * 
   * @param fod_indices 
   */
  void getFODIndices(std::vector<pcl::PointIndices>& fod_indices);

  /**
   * @brief Save each cluster (possible FOD) in a PointCloud2 msg. Give back an array of all generated msgs.
   *        \n Returns the number of possible FODs.
   *
   * @param[in] cluster_indices
   * @param[in] cloud
   * @param[out] cluster_msg_array
   * @return int number_of_fod
   */
  int fodIndicesToROSMsg(std::vector<sensor_msgs::PointCloud2>& cluster_msg_array);

  /**
   * @brief Save each cluster (possible FOD) in a pcl::PointCloudRGB. Give back an array of all generated clouds.
   *        \n Returns the number of possible FODs.
   *
   * @param[in] cluster_indices
   * @param[in] cloud
   * @param[out] cluster_msg_array
   * @return int number_of_fod
   */
  int fodIndicesToPointCloud(std::vector<PointCloudRGB::Ptr>& cloud_array);                        

private:
  /** @brief Tolerance to set as cluster tolerance. */
  double cluster_tolerance_;

  /** @brief Min number of points to identify a cluster */
  double min_cluster_size_;

  /** @brief Input cloud */
  PointCloudRGB::Ptr cloud_;

  /** @brief Cluster indices to be identified as FODs */
  std::vector<pcl::PointIndices> cluster_indices_;
};

#endif