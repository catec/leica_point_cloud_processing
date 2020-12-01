/**
 * @file FODDetector.cpp
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

#include <FODDetector.h>
#include <pcl/segmentation/extract_clusters.h>

FODDetector::FODDetector(PointCloudRGB::Ptr cloud, double cluster_tolerance, double min_fod_points)
    : cloud_(cloud)
{
  setClusterTolerance(cluster_tolerance);
  min_cluster_size_ = min_fod_points;
}

void FODDetector::setClusterTolerance(double tolerance)
{
  if(tolerance == 0)
  {
    ROS_ERROR("FODDetector: invalid tolerance value: %f", tolerance);
    cluster_tolerance_ = 4e-3; //default
    ROS_WARN("FODDetector: cluster tolerance set to: %f", cluster_tolerance_);
  }
  else
    cluster_tolerance_ = tolerance;
}

void FODDetector::setMinFODpoints(double min_fod_points)
{
  min_cluster_size_ = min_fod_points;
}

void FODDetector::clusterPossibleFODs()
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  // ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_);
  ec.extract(cluster_indices_);
  ROS_INFO("cluster_indices_size: %zd", cluster_indices_.size());
}

int FODDetector::fodIndicesToPointCloud(std::vector<PointCloudRGB::Ptr>& fod_cloud_array)
{

  int n_fods = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices_.begin(); it!=cluster_indices_.end(); ++it)
  {
    PointCloudRGB::Ptr cloud_cluster(new PointCloudRGB);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud_->points[*pit]);  
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    ROS_INFO("cluster_size: %zd", cloud_cluster->points.size());

    fod_cloud_array.push_back(cloud_cluster);
    n_fods++;
  }
  return n_fods;
}

int FODDetector::fodIndicesToROSMsg(std::vector<sensor_msgs::PointCloud2>& fod_msg_array)
{
  std::vector<PointCloudRGB::Ptr> fod_cloud_array;
  int n_fods = fodIndicesToPointCloud(fod_cloud_array);

  sensor_msgs::PointCloud2 cluster_msg;
  for (const auto &cloud : fod_cloud_array)
  {
    Utils::cloudToROSMsg(cloud, cluster_msg);
    fod_msg_array.push_back(cluster_msg);
  }

  ROS_INFO("n_fods: %d", n_fods);
  ROS_INFO("msg_size: %zd", fod_msg_array.size());

  // int n_fods = 0;

  // for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices_.begin(); it!=cluster_indices_.end(); ++it)
  // {
  //   PointCloudRGB::Ptr cloud_cluster(new PointCloudRGB);
  //   for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
  //     cloud_cluster->points.push_back(cloud_->points[*pit]);  
  //   cloud_cluster->width = cloud_cluster->points.size();
  //   cloud_cluster->height = 1;
  //   cloud_cluster->is_dense = true;

  //   n_fods++;

  // }
  return n_fods;
}

void FODDetector::getFODIndices(std::vector<pcl::PointIndices>& fod_indices)
{
  fod_indices = cluster_indices_;
}