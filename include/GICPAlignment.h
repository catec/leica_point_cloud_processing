/**
 * @file GICPAlignment.h
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
#ifndef _GCIP_ALIGNMENT_H
#define _GCIP_ALIGNMENT_H

#include <Utils.h>

#include <pcl/features/from_meshes.h>
#include <pcl/registration/gicp.h>

#endif

/**
 * @brief Perform a fine alignment from source to target cloud with Generalized Iterative Closest Point.
 *
 * POINTCLOUDS:
 * - \b target pointcloud: should be obtained from converting a part's CAD. Use CADToPointCloud.
 * - \b source pointcloud: obtained from Leica scan and previously aligned with target using InitialAlignment.
 */
class GICPAlignment
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
  typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> CovariancesVector;

public:
  /**
   * @brief Construct a new GICPAlignment object
   *
   * @param[in] target_cloud
   * @param[in] source_cloud
   */
  GICPAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud);

  /**
   * @brief Destroy the GICPAlignment object
   *
   */
  ~GICPAlignment(){};

  /** @brief If true, fine transformation is finished. */
  bool transform_exists;

  /** @brief GICP object. */
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> _gicp;

  /**
   * @brief Perform GICP alignment
   *
   */
  void run();

  /**
   * @brief Do more iterations of GICP alignment
   *
   */
  void iterate();

  /**
   * @brief Undo last iteration and back up results
   *
   */
  void undo();

  /**
   * @brief Get the fine transform object
   *
   * @return Eigen::Matrix4f
   */
  Eigen::Matrix4f getFineTransform();

  /**
   * @brief Get the aligned cloud object
   *
   * @param[out] aligned_cloud
   */
  void getAlignedCloud(PointCloudRGB::Ptr aligned_cloud);

  /**
   * @brief Get the aligned cloud msg in PointCloud2 format
   *
   * @param[out] aligned_cloud_msg
   */
  void getAlignedCloudROSMsg(sensor_msgs::PointCloud2& aligned_cloud_msg);

  /**
   * @brief Once GICP is finished, apply fine transformation to source cloud.
   *
   * @param[out] cloud
   */
  void applyTFtoCloud(PointCloudRGB::Ptr cloud);

private:
  /** @brief Transformation matrix as result of GICP alignment. */
  Eigen::Matrix4f _fine_tf;

  /** @brief Radius to compute normals. */
  double _normal_radius;

  /** @brief Target pointcloud. */
  PointCloudRGB::Ptr _target_cloud;

  /** @brief Source pointcloud. */
  PointCloudRGB::Ptr _source_cloud;

  /** @brief Source pointcloud aligned with target cloud. */
  PointCloudRGB::Ptr _aligned_cloud;

  /** @brief Save aligned pointcloud to restore if undo. */
  PointCloudRGB::Ptr _backup_cloud;

  /**
   * @brief Will set parameters to compute GICP alignment based on clouds data.
   *
   */
  void configParameters();

  /**
   * @brief Apply covariances to perform fine alignment.
   *
   * @param[in] source_cloud
   * @param[in] target_cloud
   */
  void fineAlignment(PointCloudRGB::Ptr source_cloud, PointCloudRGB::Ptr target_cloud);

  /**
   * @brief Get the covariances from cloud
   *
   * @param[in] cloud
   * @param[out] covs
   */
  void getCovariances(PointCloudRGB::Ptr cloud, boost::shared_ptr<CovariancesVector> covs);

  /**
   * @brief Re-iterate GICP and save new aligned cloud.
   *
   * @param[out] cloud
   */
  void iterateFineAlignment(PointCloudRGB::Ptr cloud);

  /**
   * @brief Store cloud as backup cloud
   *
   * @param[in] cloud
   */
  void backUp(PointCloudRGB::Ptr cloud);
};