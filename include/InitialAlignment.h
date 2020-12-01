/**
 * @file InitialAlignment.h
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
#ifndef _INITIAL_ALIGNMENT_H
#define _INITIAL_ALIGNMENT_H

#include <Utils.h>

/**
 * @brief Perform a rigid alignment from source to target cloud.
 *
 * POINTCLOUDS:
 * - \b target pointcloud: should be obtained from converting a part's CAD. Use CADToPointCloud.
 * - \b source pointcloud: result from scanning the same CAD part with Leica scanstation C5 (either in reality or in
 * gazebo simulator).
 *
 * WORKFLOW:
 * 1. Before using this algorithm prepare clouds: Filter pointclouds to enhance results.
 * 2. Extract features and keypoints from pointcloud and it's normals (FPFH descriptor and
 * pcl::MultiscaleFeaturePersistence).
 * 3. Use features and keypoints to determine correspondences between clouds (pcl::CorrespondenceEstimation).
 * 4. Estimate rigid transformation as initial alignment (pcl::TransformationEstimationSVD).
 * 5. Then, apply GICP to refine transformation (pcl::GeneralizedIterativeClosestPoint)
 *
 */

namespace AlignmentMethods  
{
  /**
   * @brief Define possible Alignment methods
   * 
   */
  enum AlignmentMethod
  {
    HARRIS,
    BOUNDARY,
    MULTISCALE,
    NORMALS,
    NONE
  };
}
typedef AlignmentMethods::AlignmentMethod AlignmentMethod;

class InitialAlignment
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
  typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
  typedef pcl::PointCloud<pcl::FPFHSignature33> PointCloudFPFH;

public:
  /**
   * @brief Construct a new Initial Alignment object.
   *
   * @param[in] target_cloud
   * @param[in] source_cloud
   */
  InitialAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud);

  /**
   * @brief Destroy the Initial Alignment object.
   *
   */
  ~InitialAlignment(){};

  /** @brief If true, rigid transformation is complete. */
  bool transform_exists_;

  /**
   * @brief Perform initial alignment.
   *
   */
  void run();

  /**
   * @brief Get the rigid transform object
   *
   * @return Eigen::Matrix4f
   */
  Eigen::Matrix4f getRigidTransform();

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
   * @brief Once initial alignment is finished, apply rigid transformation to given cloud.
   *
   * @param[out] cloud
   */
  void applyTFtoCloud(PointCloudRGB::Ptr cloud);

  /**
   * @brief Apply given tf to cloud
   * 
   * @param[out] cloud 
   * @param[in] tf 
   */
  void applyTFtoCloud(PointCloudRGB::Ptr cloud, Eigen::Matrix4f tf);

  /**
   * @brief Set the AlignmentMethod object
   * 
   * @param method 
   */
  void setMethod(AlignmentMethod method);

  /**
   * @brief Set the Radius Factor object
   * 
   * @param align_factor 
   */
  void setRadiusFactor(double align_factor);

private:
  /** @brief Transformation matrix as result of initial alignment. */
  Eigen::Matrix4f rigid_tf_;

  /** @brief Radius to compute normals. */
  double normal_radius_;

  /** @brief Factor to apply to radius for computations. */
  double radius_factor_;

  /** @brief Target pointcloud. */
  PointCloudRGB::Ptr target_cloud_;

  /** @brief Source pointcloud. */
  PointCloudRGB::Ptr source_cloud_;

  /** @brief Source pointcloud aligned with target cloud. */
  PointCloudRGB::Ptr aligned_cloud_;

  /** @brief Source normals. */
  PointCloudNormal::Ptr source_normals_;

  /** @brief Target normals. */
  PointCloudNormal::Ptr target_normals_;

  /** @brief Source dominant normals. Just used int NORMALS method*/
  PointCloudNormal::Ptr source_dominant_normals_;

  /** @brief Target dominant normals. Just used int NORMALS method */
  PointCloudNormal::Ptr target_dominant_normals_;

  /** @brief Target keypoints. */
  PointCloudRGB::Ptr target_keypoints_;
  
  /** @brief Source keypoints. */
  PointCloudRGB::Ptr source_keypoints_;

  /** @brief Target features. */
  PointCloudFPFH::Ptr target_features_;

  /** @brief Source features. */
  PointCloudFPFH::Ptr source_features_;

  /** @brief Correspondences between source and target. */
  pcl::CorrespondencesPtr correspondences_;

  /** @brief The alignment method. */
  AlignmentMethod method_;

  /** @brief Source centroid computed. */
  Eigen::Vector4f source_centroid_;

  /** @brief Target centroid computed. */
  Eigen::Vector4f target_centroid_;

  /**
   * @brief Once initial alignment is finished, apply rigid transformation to source cloud.
   *
   */
  void applyTFtoCloud();

  /**
   * @brief If set method is NORMALS run algorithm. Cluster normals and found correspondences between dominant normals. Get transform between clouds 
   * 
   */
  void runNormalsBasedAlgorithm();

  /**
   * @brief Run keypoints based algorithm. Compute keypoints and features, found correspondences and get transform.
   * 
   */
  void runKeypointsBasedAlgorithm();

  /**
   * @brief Cluster clouds normals. 
   * 
   * @param[in] cloud 
   * @param[in] normals 
   * @param[out] cluster_indices 
   */
  void obtainNormalsCluster(PointCloudRGB::Ptr cloud,
                            PointCloudNormal::Ptr normals,
                            std::vector<pcl::PointIndices>& cluster_indices);

  /**
   * @brief Obtain dominant normal in given cluster
   * 
   * @param[in] normals 
   * @param[in] cluster_indices 
   * @param[out] dominant_normals 
   */
  void obtainDominantNormals(PointCloudNormal::Ptr normals,
                             std::vector<pcl::PointIndices> cluster_indices,
                             PointCloudNormal::Ptr dominant_normals);                 

  /**
   * @brief Found correspondences between dominant normals.
   * 
   * @param[in] source_dominant_normal 
   * @param[in] target_dominant_normal 
   * @param[out] source_idx 
   * @param[out] target_idx 
   * @return true 
   * @return false 
   */
  bool normalsCorrespondences(PointCloudNormal::Ptr source_dominant_normal,
                              PointCloudNormal::Ptr target_dominant_normal,
                              std::vector<int>& source_idx, std::vector<int>& target_idx);

  /**
   * @brief Get transformation between cloud based on normals correspondences
   * 
   * @param[in] source_normals 
   * @param[in] target_normals 
   */
  void getTransformationFromNormals(PointCloudNormal::Ptr source_normals,
                                    PointCloudNormal::Ptr target_normals);

  /**
   * @brief Function to get index for Source matrix value in Target matrix.
   * 
   * @param[in] source_m source matrix form by angle between normals
   * @param[in] target_m target matrix form by angle between normals
   * @param[in] source_idx_m source matrix form by index of source_m
   * @param[in] target_idx_m target matrix form by index of target_m
   * @param[out] source_indx source indexes that correspond with target indexes
   * @param[out] target_indx source indexes that correspond with target indexes
   * @return true 
   * @return false 
   */
  bool findIdx(Eigen::MatrixXd source_m, Eigen::MatrixXd target_m,
               Eigen::MatrixXi source_idx_m, Eigen::MatrixXi target_idx_m,
               std::vector<int>& source_indx, std::vector<int>& target_indx);

  /**
   * @brief Create coordinate system from corresponden normals
   * 
   * @param normals 
   * @param index 
   * @return Eigen::Matrix3f 
   */
  Eigen::Matrix3f getCoordinateSystem(PointCloudNormal::Ptr normals, 
                                      std::vector<int>& index);

  /**
   * @brief Construct matrix based on angles between normals
   * 
   * @param[in] normal 
   * @param[in] size 
   * @param[out] matrix 
   * @param[out] index_matrix 
   */
  void matrixFromAngles(PointCloudNormal::Ptr normal,
                        size_t size,
                        Eigen::MatrixXd& matrix,
                        Eigen::MatrixXi& index_matrix);                                                                                                   
  
  /**
   * @brief Get next matrix vector with extracted current col and row
   * 
   * @param matrix 
   * @param current_col 
   * @param current_row 
   * @return Eigen::VectorXd 
   */
  Eigen::VectorXd nextVector(Eigen::MatrixXd matrix,
                             int current_col, int current_row);

  /**
   * @brief Obtain cloud keypoints based on harris method
   * 
   * @param cloud 
   * @param normals 
   * @param keypoints_cloud 
   * @param keypoints_indices 
   * @return int 
   */
  int obtainHarrisKeypoints(PointCloudRGB::Ptr cloud,
                            PointCloudNormal::Ptr normals,
                            PointCloudRGB::Ptr keypoints_cloud,
                            pcl::IndicesPtr keypoints_indices);  

  /**
   * @brief Obtain cloud keypoints based on boundary method
   * 
   * @param[in] cloud 
   * @param[in] normals 
   * @param[out] keypoints_cloud 
   * @param[out] keypoints_indices 
   * @param[out] non_keypoints_indices 
   * @return int 
   */
  int obtainBoundaryKeypoints(PointCloudRGB::Ptr cloud,
                              PointCloudNormal::Ptr normals,
                              PointCloudRGB::Ptr keypoints_cloud,
                              pcl::IndicesPtr keypoints_indices,
                              pcl::IndicesPtr non_keypoints_indices);

  /**
   * @brief Obtain cloud keypoints based on multiscale method
   * 
   * @param[in] cloud 
   * @param[in] normals 
   * @param[out] keypoints_cloud 
   * @param[out] keypoints_indices 
   * @param[out] features 
   * @return int 
   */
  int obtainMultiScaleKeypointsAndFeatures(PointCloudRGB::Ptr cloud,
                                           PointCloudNormal::Ptr normals,
                                           PointCloudRGB::Ptr keypoints_cloud,
                                           pcl::IndicesPtr keypoints_indices,
                                           PointCloudFPFH::Ptr features);

  /**
   * @brief Obtain cloud features based on given keypoints
   * 
   * @param[in] cloud 
   * @param[in] normals 
   * @param[in] keypoints_indices 
   * @param[out] features 
   * @return int 
   */
  int obtainFeatures(PointCloudRGB::Ptr cloud,
                     PointCloudNormal::Ptr normals,
                     pcl::IndicesPtr keypoints_indices,
                     PointCloudFPFH::Ptr features);                                                                                               

  /**
   * @brief Obtain keypoints and features based on set method
   * 
   * @param cloud 
   * @param normals 
   * @param keypoints_cloud 
   * @param features_cloud 
   * @return int 
   */
  int obtainKeypointsAndFeatures(PointCloudRGB::Ptr cloud, 
                                 PointCloudNormal::Ptr normals,
                                 PointCloudRGB::Ptr keypoints_cloud,
                                 PointCloudFPFH::Ptr features_cloud);

  /**
   * @brief Get correspondences between clouds from computed features
   * 
   */
  void obtainCorrespondences();

  /**
   * @brief Reject computed correspondences
   * 
   */
  void rejectCorrespondences();

  /**
   * @brief Reject computed correspondences
   * 
   */
  void rejectOneToOneCorrespondences();

  /**
   * @brief Get transform from computed correspondences
   * 
   */
  void estimateTransform();                                                                              
};

#endif