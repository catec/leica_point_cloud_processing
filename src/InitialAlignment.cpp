/**
 * @file InitialAlignment.cpp
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

#include <InitialAlignment.h>

InitialAlignment::InitialAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud)
  : aligned_cloud_(new PointCloudRGB)
{
  target_cloud_ = target_cloud;
  source_cloud_ = source_cloud;
  configParameters();
  transform_exists_ = false;
  rigid_tf_ = Eigen::Matrix4f::Zero();
}

void InitialAlignment::run()
{
  // Viewer v;
  // v.addPCToViewer<pcl::PointXYZRGB>(source_cloud_, "source");
  // v.addPCToViewer<pcl::PointXYZRGB>(target_cloud_, "target");

  pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
  getNormals(source_cloud_, normal_radius_, source_normals);
  getNormals(target_cloud_, normal_radius_, target_normals);

  // v.addNormalsToViewer<pcl::PointXYZRGB, pcl::Normal>(source_cloud_, source_normals, "n_source");
  // v.addNormalsToViewer<pcl::PointXYZRGB, pcl::Normal>(target_cloud_, target_normals, "n_target");

  PointCloudRGB::Ptr target_keypoints(new PointCloudRGB);
  PointCloudRGB::Ptr source_keypoints(new PointCloudRGB);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>());
  getKeypointsAndFeatures(source_cloud_, source_normals, source_keypoints, source_features);
  getKeypointsAndFeatures(target_cloud_, target_normals, target_keypoints, target_features);

  Utils::colorizeCloud(source_keypoints, 0, 255, 0);
  Utils::colorizeCloud(target_keypoints, 0, 255, 0);
  // v.addPCToViewer<pcl::PointXYZRGB>(source_keypoints,"key");
  // v.addPCToViewer<pcl::PointXYZRGB>(target_keypoints,"tkey");

  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  performInitialAlingment(source_features, target_features, source_keypoints, target_keypoints, correspondences);

  applyTFtoCloud();

  // v.addCorrespondencesToViewer<pcl::PointXYZRGB>(source_keypoints, target_keypoints, correspondences);
  // v.deletePCFromViewer("source");
  // v.addPCToViewer<pcl::PointXYZRGB>(aligned_cloud_,"aligned");
}

Eigen::Matrix4f InitialAlignment::getRigidTransform()
{
  if (!transform_exists_)
    ROS_ERROR("No transform yet. Please run algorithm");
  return rigid_tf_;
}

void InitialAlignment::configParameters()
{
  // si los parametros son el mismo para ambas tiene que depender de las dos
  double target_res = Utils::computeCloudResolution(target_cloud_);
  double source_res = Utils::computeCloudResolution(source_cloud_);

  normal_radius_ = (target_res + source_res) * 2.0;  // 2 times higher
  feature_radius_ = normal_radius_ * 1.20;           // 20% higher
  inlier_threshold_ = 1e-5;                          // 2.5;

  // ROS_INFO("Parameters: \n\tnormal radius: %f, \   n\tfeature radius: %f",normal_radius_, feature_radius_);
}

bool InitialAlignment::getNormals(PointCloudRGB::Ptr& cloud, double normal_radius,
                                  pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
  ROS_INFO("Computing normals with radius: %f", normal_radius);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(normal_radius);
  ne.compute(*normals);

  // As we compute normal for each point in cloud, must have same size
  bool success = normals->points.size() == cloud->points.size() ? true : false;

  return success;
}

std::vector<float> InitialAlignment::getScaleValues(PointCloudRGB::Ptr cloud)
{
  std::vector<float> scale_values;

  double cloud_resolution = Utils::computeCloudResolution(cloud);
  // scale_values.push_back((float)cloud_resolution*0.1);
  scale_values.push_back((float)cloud_resolution * 1);
  scale_values.push_back((float)cloud_resolution * 2);
  scale_values.push_back((float)cloud_resolution * 3);
  scale_values.push_back((float)cloud_resolution * 4);
  scale_values.push_back((float)cloud_resolution * 5);
  scale_values.push_back((float)cloud_resolution * 6);

  return scale_values;
}

void InitialAlignment::printScaleValues(const std::vector<float>& scale_values)
{
  ROS_INFO("scale:");
  for (int i = 0; i < scale_values.size(); i++)
  {
    ROS_INFO("%f", scale_values[i]);
  }
}

void InitialAlignment::getKeypointsAndFeatures(PointCloudRGB::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,
                                               PointCloudRGB::Ptr keypoints_cloud,
                                               pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
  // SELECTED DESCRIPTOR: FPFH
  ROS_INFO("1. Set descriptor FPFH with radius: %f", feature_radius_);
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr fest(
      new pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
  fest->setInputCloud(cloud);
  fest->setInputNormals(normals);
  // fest->setRadiusSearch(feature_radius_);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  fest->setSearchMethod(tree);

  ROS_INFO("2. Define feature persistence");
  pcl::MultiscaleFeaturePersistence<pcl::PointXYZRGB, pcl::FPFHSignature33> fper;
  boost::shared_ptr<std::vector<int> > keypoints(new std::vector<int>);  // Interest points
  std::vector<float> scale_values = getScaleValues(cloud);
  // printScaleValues(scale_values);
  fper.setScalesVector(scale_values);
  fper.setAlpha(0.6f);
  fper.setFeatureEstimator(fest);
  fper.setDistanceMetric(pcl::CS);

  ROS_INFO("3. Extracting keypoints");
  fper.determinePersistentFeatures(*features, keypoints);

  // ROS_INFO("keypoints: %zu", keypoints->size());
  bool success = keypoints->size() == features->size() ? true : false;

  Utils::indicesFilter(cloud, keypoints_cloud, keypoints);
}

void InitialAlignment::performInitialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                                               pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                                               PointCloudRGB::Ptr source_keypoints, PointCloudRGB::Ptr target_keypoints,
                                               pcl::CorrespondencesPtr corr_filtered)
{
  ROS_INFO("4. Use descriptor FPFH to compute correspondences");
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> cest;
  cest.setInputSource(source_features);
  cest.setInputTarget(target_features);
  cest.determineCorrespondences(*correspondences);

  ROS_INFO("5. Get correspondences with inlier threshold: %f", inlier_threshold_);
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector;
  rejector.setInputSource(source_keypoints);
  rejector.setInputTarget(target_keypoints);
  rejector.setInlierThreshold(inlier_threshold_);
  rejector.setMaximumIterations(10000);
  rejector.setRefineModel(false);
  rejector.setInputCorrespondences(correspondences);
  rejector.getCorrespondences(*corr_filtered);

  ROS_INFO("6. Get rigid transformation");
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> trans_est;
  trans_est.estimateRigidTransformation(*source_keypoints, *target_keypoints, *corr_filtered, rigid_tf_);
  transform_exists_ = true;
}

void InitialAlignment::applyTFtoCloud()
{
  pcl::transformPointCloud(*source_cloud_, *aligned_cloud_, rigid_tf_);
}

void InitialAlignment::applyTFtoCloud(PointCloudRGB::Ptr cloud)
{
  pcl::transformPointCloud(*cloud, *cloud, rigid_tf_);
}

void InitialAlignment::getAlignedCloud(PointCloudRGB::Ptr aligned_cloud)
{
  pcl::copyPointCloud(*aligned_cloud_, *aligned_cloud);
}

void InitialAlignment::getAlignedCloudROSMsg(sensor_msgs::PointCloud2& aligned_cloud_msg)
{
  pcl::toROSMsg(*aligned_cloud_, aligned_cloud_msg);
  aligned_cloud_msg.header.frame_id = Utils::frame_id_;
  aligned_cloud_msg.header.stamp = ros::Time::now();
}