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
#ifndef _GICP_ALIGNMENT_H
#define _GICP_ALIGNMENT_H

#include <Utils.h>
#include <pcl/registration/gicp.h>

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
  typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
  typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> CovariancesVector;

public:
  /**
   * @brief Construct a new GICPAlignment object
   *
   * @param[in] target_cloud
   * @param[in] source_cloud
   * @param[in] use_covariances
   */
  GICPAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud, bool use_covariances);

  /**
   * @brief Destroy the GICPAlignment object
   *
   */
  ~GICPAlignment(){};

  /** @brief If true, fine transformation is finished. */
  bool transform_exists_;


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

  /**
   * @brief Set the Source Cloud object
   * 
   * @param source_cloud 
   */
  void setSourceCloud(PointCloudRGB::Ptr source_cloud);

  /**
   * @brief Set the Target Cloud object
   * 
   * @param target_cloud 
   */
  void setTargetCloud(PointCloudRGB::Ptr target_cloud);

  /**
   * @brief Set the Max Iterations object
   * 
   * @param iterations 
   */
  void setMaxIterations(int iterations);

  /**
   * @brief Set the Tf Epsilon object
   * 
   * @param tf_epsilon 
   */
  void setTfEpsilon(double tf_epsilon);
  
  /**
   * @brief Set the Max Correspondence Distance object
   * 
   * @param max_corresp_distance 
   */
  void setMaxCorrespondenceDistance(int max_corresp_distance);
  
  /**
   * @brief Set the RANSAC Outlier threshold
   * 
   * @param ransac_threshold 
   */
  void setRANSACOutlierTh(int ransac_threshold);

private:

  /** @brief If true, perform GICP with convariances. */
  bool covariances_;
  
  /** @brief GICP object. */
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp_;

  /** @brief Transformation matrix as result of GICP alignment. */
  Eigen::Matrix4f fine_tf_;

  
  /** @brief Maximum number of GICP iterations */
  int max_iter_;

  /** @brief Value for TFEpsilon for GICP. Define the convergence criterion */
  double tf_epsilon_;

  /** @brief Value for Max CorrespondenceDistance for GICP */
  double max_corresp_distance_;

  /** @brief Value for RANSAC outlier threshold */
  double ransac_outlier_th_;

  /** @brief Target pointcloud. */
  PointCloudRGB::Ptr target_cloud_;

  /** @brief Source pointcloud. */
  PointCloudRGB::Ptr source_cloud_;

  /** @brief Source pointcloud aligned with target cloud. */
  PointCloudRGB::Ptr aligned_cloud_;

  /** @brief Save aligned pointcloud to restore if undo. */
  PointCloudRGB::Ptr backup_cloud_;

  /**
   * @brief Set parameters to compute GICP alignment.
   *
   */
  void configParameters();

  /**
   * @brief Apply GICP to perform fine alignment.
   *
   * @param[in] source_cloud
   * @param[in] target_cloud
   */
  void fineAlignment();

  /**
   * @brief Get the covariances from cloud
   *
   * @param[in] cloud
   * @param[out] covs
   */
  void getCovariances(PointCloudRGB::Ptr cloud, boost::shared_ptr<CovariancesVector> covs);

  /**
   * @brief Apply the covariances before running GICP
   * 
   */
  void applyCovariances();
  
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

#endif