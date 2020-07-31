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

FODDetector::FODDetector(double resolution)
{
  _voxel_resolution = resolution;
}

void FODDetector::clusterPossibleFODs(PointCloudRGB::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(_voxel_resolution);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
}

int FODDetector::clusterIndicesToROSMsg(const std::vector<pcl::PointIndices>& cluster_indices, PointCloudRGB::Ptr cloud,
                                        std::vector<sensor_msgs::PointCloud2>& cluster_msg_array)
{
  int n_fods = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    PointCloudRGB::Ptr cloud_cluster(new PointCloudRGB);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);  //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::string cluster_name = "/fod" + std::to_string(n_fods);
    n_fods++;

    sensor_msgs::PointCloud2 cluster_msg;
    pcl::toROSMsg(*cloud_cluster, cluster_msg);
    cluster_msg.header.frame_id = Utils::_frame_id;
    cluster_msg.header.stamp = ros::Time::now();
    cluster_msg_array.push_back(cluster_msg);
  }
  return n_fods;
}