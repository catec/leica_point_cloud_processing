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
  : aligned_cloud_(new PointCloudRGB), backup_cloud_(new PointCloudRGB)
{
  target_cloud_ = target_cloud;
  source_cloud_ = source_cloud;
  transform_exists_ = false;
  fine_tf_ = Eigen::Matrix4f::Zero();

  configParameters();
}

void GICPAlignment::run()
{
  fineAlignment(source_cloud_, target_cloud_);

  applyTFtoCloud(source_cloud_);
}

void GICPAlignment::iterate()
{
  iterateFineAlignment(aligned_cloud_);
}

Eigen::Matrix4f GICPAlignment::getFineTransform()
{
  if (!transform_exists_)
    ROS_ERROR("No transform yet. Please run algorithm");
  return fine_tf_;
}

void GICPAlignment::getAlignedCloud(PointCloudRGB::Ptr aligned_cloud)
{
  pcl::copyPointCloud(*aligned_cloud_, *aligned_cloud);
}

void GICPAlignment::getAlignedCloudROSMsg(sensor_msgs::PointCloud2& aligned_cloud_msg)
{
  pcl::toROSMsg(*aligned_cloud_, aligned_cloud_msg);
  aligned_cloud_msg.header.frame_id = Utils::frame_id_;
  aligned_cloud_msg.header.stamp = ros::Time::now();
}

void GICPAlignment::configParameters()
{
  double target_res = Utils::computeCloudResolution(target_cloud_);
  double source_res = Utils::computeCloudResolution(source_cloud_);

  normal_radius_ = (target_res + source_res) * 2.0;     // 2 times higher
  double threshold = (target_res + source_res) * 0.95;  // just below resolution

  // setup Generalized-ICP
  gicp_.setMaxCorrespondenceDistance(100 * threshold);
  gicp_.setMaximumIterations(100);
  gicp_.setEuclideanFitnessEpsilon(1);   // divergence criterion
  gicp_.setTransformationEpsilon(1e-9);  // convergence criterion
  gicp_.setRANSACOutlierRejectionThreshold(threshold);
}

void GICPAlignment::getCovariances(PointCloudRGB::Ptr cloud, boost::shared_ptr<CovariancesVector> covs)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  // pcl::features::computeApproximateNormals(*cloud, mesh->polygons, *normals); // NOT WORKING
  Utils::getNormals(cloud, normal_radius_, normals);
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

  ROS_INFO("8. Perform GICP with %d iterations", gicp_.getMaximumIterations());
  ros::Time begin = ros::Time::now();
  gicp_.setInputSource(source_cloud);
  gicp_.setInputTarget(target_cloud);
  gicp_.setSourceCovariances(source_covariances);
  gicp_.setTargetCovariances(target_covariances);
  // run Alignment and get transformation
  PointCloudRGB::Ptr aligned_cloud(new PointCloudRGB);
  gicp_.align(*aligned_cloud);

  ros::Duration exec_time = ros::Time::now() - begin;
  ROS_INFO("GICP time: %lf s", exec_time.toSec());

  if (gicp_.hasConverged())
  {
    ROS_INFO("Converged in %f", gicp_.getFitnessScore());
    fine_tf_ = gicp_.getFinalTransformation();
    ROS_INFO("9. Transform result of gicp: ");
    transform_exists_ = true;
    gicp_.setMaximumIterations(10);  // for future iterations
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
  gicp_.align(*cloud);

  if (gicp_.hasConverged())
  {
    temp_tf = gicp_.getFinalTransformation();
    fine_tf_ = temp_tf * fine_tf_;
    // Utils::printTransform(fine_tf_);
    ROS_INFO("Converged in %f", gicp_.getFitnessScore());
  }
  else
  {
    ROS_ERROR("no converge");
  }
}

void GICPAlignment::backUp(PointCloudRGB::Ptr cloud)
{
  pcl::copyPointCloud(*cloud, *backup_cloud_);
}

void GICPAlignment::undo()
{
  pcl::copyPointCloud(*backup_cloud_, *aligned_cloud_);
}

void GICPAlignment::applyTFtoCloud(PointCloudRGB::Ptr cloud)
{
  pcl::transformPointCloud(*cloud, *aligned_cloud_, fine_tf_);
}