/**
 * @file GICPAlignment.cpp
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

#include <GICPAlignment.h>

GICPAlignment::GICPAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud)
  : _aligned_cloud(new PointCloudRGB), _backup_cloud(new PointCloudRGB)
{
  _target_cloud = target_cloud;
  _source_cloud = source_cloud;
  transform_exists = false;
  _fine_tf = Eigen::Matrix4f::Zero();

  configParameters();
}

void GICPAlignment::run()
{
  fineAlignment(_source_cloud, _target_cloud);

  applyTFtoCloud(_source_cloud);
}

void GICPAlignment::iterate()
{
  iterateFineAlignment(_aligned_cloud);
}

Eigen::Matrix4f GICPAlignment::getFineTransform()
{
  if (!transform_exists)
    ROS_ERROR("No transform yet. Please run algorithm");
  return _fine_tf;
}

void GICPAlignment::getAlignedCloud(PointCloudRGB::Ptr aligned_cloud)
{
  pcl::copyPointCloud(*_aligned_cloud, *aligned_cloud);
}

void GICPAlignment::getAlignedCloudROSMsg(sensor_msgs::PointCloud2& aligned_cloud_msg)
{
  pcl::toROSMsg(*_aligned_cloud, aligned_cloud_msg);
  aligned_cloud_msg.header.frame_id = Utils::_frame_id;
  aligned_cloud_msg.header.stamp = ros::Time::now();
}

void GICPAlignment::configParameters()
{
  double target_res = Utils::computeCloudResolution(_target_cloud);
  double source_res = Utils::computeCloudResolution(_source_cloud);

  _normal_radius = (target_res + source_res) * 2.0;     // 2 times higher
  double threshold = (target_res + source_res) * 0.95;  // just below resolution

  // setup Generalized-ICP
  _gicp.setMaxCorrespondenceDistance(100 * threshold);
  _gicp.setMaximumIterations(100);
  _gicp.setEuclideanFitnessEpsilon(1);   // divergence criterion
  _gicp.setTransformationEpsilon(1e-9);  // convergence criterion
  _gicp.setRANSACOutlierRejectionThreshold(threshold);
}

void GICPAlignment::getCovariances(PointCloudRGB::Ptr cloud, boost::shared_ptr<CovariancesVector> covs)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  // pcl::features::computeApproximateNormals(*cloud, mesh->polygons, *normals); // NOT WORKING
  Utils::getNormals(cloud, _normal_radius, normals);
  // filter NaN values
  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);  // Interest points
  pcl::removeNaNFromPointCloud(*cloud, *cloud, *indices);
  pcl::removeNaNNormalsFromPointCloud(*normals, *normals, *indices);
  Utils::indicesFilter(cloud, cloud, indices);

  // get covariances
  pcl::features::computeApproximateCovariances(*cloud, *normals, *covs);
  bool success = normals->points.size() == covs->size() ? true : false;
}

void GICPAlignment::fineAlignment(PointCloudRGB::Ptr source_cloud, PointCloudRGB::Ptr target_cloud)
{
  ROS_INFO("7. Extract covariances from clouds");
  boost::shared_ptr<CovariancesVector> source_covariances(new CovariancesVector);
  boost::shared_ptr<CovariancesVector> target_covariances(new CovariancesVector);

  getCovariances(source_cloud, source_covariances);
  getCovariances(target_cloud, target_covariances);

  ROS_INFO("8. Perform GICP with %d iterations", _gicp.getMaximumIterations());
  ros::Time begin = ros::Time::now();
  _gicp.setInputSource(source_cloud);
  _gicp.setInputTarget(target_cloud);
  _gicp.setSourceCovariances(source_covariances);
  _gicp.setTargetCovariances(target_covariances);
  // run Alignment and get transformation
  PointCloudRGB::Ptr aligned_cloud(new PointCloudRGB);
  _gicp.align(*aligned_cloud);

  ros::Duration exec_time = ros::Time::now() - begin;
  ROS_INFO("GICP time: %lf s", exec_time.toSec());

  if (_gicp.hasConverged())
  {
    ROS_INFO("Converged in %f", _gicp.getFitnessScore());
    _fine_tf = _gicp.getFinalTransformation();
    ROS_INFO("9. Transform result of gicp: ");
    transform_exists = true;
    _gicp.setMaximumIterations(10);  // for future iterations
  }
  else
  {
    ROS_ERROR("NO CONVERGE");
  }
}

void GICPAlignment::iterateFineAlignment(PointCloudRGB::Ptr cloud)
{
  backUp(cloud);  // save a copy to restore in case iteration give wrong results

  Eigen::Matrix4f temp_tf;
  ROS_INFO("Computing iteration...");
  _gicp.align(*cloud);

  if (_gicp.hasConverged())
  {
    temp_tf = _gicp.getFinalTransformation();
    _fine_tf = temp_tf * _fine_tf;
    // Utils::printTransform(_fine_tf);
    ROS_INFO("Converged in %f", _gicp.getFitnessScore());
  }
  else
  {
    ROS_ERROR("no converge");
  }
}

void GICPAlignment::backUp(PointCloudRGB::Ptr cloud)
{
  pcl::copyPointCloud(*cloud, *_backup_cloud);
}

void GICPAlignment::undo()
{
  pcl::copyPointCloud(*_backup_cloud, *_aligned_cloud);
}

void GICPAlignment::applyTFtoCloud(PointCloudRGB::Ptr cloud)
{
  pcl::transformPointCloud(*cloud, *_aligned_cloud, _fine_tf);
}