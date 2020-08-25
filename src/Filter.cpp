/**
 * @file Filter.cpp
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

#include <Filter.h>

Filter::Filter(double leaf_size)
{
  leaf_size_ = leaf_size;
  noise_filter_threshold_ = 0;
  floor_filter_threshold_ = 0;
  user_given_center_ = false;
}

Filter::Filter(double leaf_size, double noise_threshold)
{
  leaf_size_ = leaf_size;
  noise_filter_threshold_ = noise_threshold;
  floor_filter_threshold_ = 0;
  user_given_center_ = false;
}

Filter::Filter(double leaf_size, double noise_threshold, double floor_threshold)
{
  leaf_size_ = leaf_size;
  noise_filter_threshold_ = noise_threshold;
  floor_filter_threshold_ = floor_threshold;
  user_given_center_ = false;
}

Filter::Filter(Eigen::Vector3f cloud_center, double leaf_size, double noise_threshold, double floor_threshold)
{
  cloud_center_ = cloud_center;
  leaf_size_ = leaf_size;
  noise_filter_threshold_ = noise_threshold;
  floor_filter_threshold_ = floor_threshold;
  user_given_center_ = true;
}

void Filter::downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled)
{
  ROS_INFO("Filtering with leaf size: %f", leaf_size_);
  double res = Utils::computeCloudResolution(cloud);
  // ROS_INFO("Pointcloud size before downsampling: %zu",cloud->points.size());
  ROS_INFO("Pointcloud resolution before downsampling: %f", res);

  pcl::VoxelGrid<pcl::PointXYZRGB> downsampling_filter;
  downsampling_filter.setInputCloud(cloud);
  const Eigen::Vector4f downsampling_leaf_size(leaf_size_, leaf_size_, leaf_size_, 0.0f);
  downsampling_filter.setLeafSize(downsampling_leaf_size);
  downsampling_filter.filter(*cloud_downsampled);

  res = Utils::computeCloudResolution(cloud_downsampled);
  // ROS_INFO("Pointcloud size after downsampling: %zu",cloud_downsampled->points.size());
  ROS_INFO("Pointcloud resolution after downsampling: %f", res);
}

void Filter::filter_noise(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
  // ROS_INFO("create box with threshold: %f",threshold);
  pcl::CropBox<pcl::PointXYZRGB> boxFilter;

  // get box center
  Eigen::Vector3f boxTranslatation;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  boxTranslatation[0] = centroid[0];
  boxTranslatation[1] = centroid[1];
  boxTranslatation[2] = centroid[2];
  ROS_INFO("Point cloud center: (%f,%f,%f)", centroid[0], centroid[1], centroid[2]);

  // apply filter
  boxFilter.setTranslation(boxTranslatation);
  boxFilter.setMin(Eigen::Vector4f(-threshold, -threshold, -threshold, 1.0));
  boxFilter.setMax(Eigen::Vector4f(threshold, threshold, threshold, 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*cloud_filtered);
}

void Filter::filter_noise(Eigen::Vector3f cloud_center, double threshold, PointCloudRGB::Ptr cloud,
                          PointCloudRGB::Ptr cloud_filtered)
{
  pcl::CropBox<pcl::PointXYZRGB> boxFilter;

  ROS_INFO("Point cloud center: (%f,%f,%f)", cloud_center[0], cloud_center[1], cloud_center[2]);

  // apply filter
  boxFilter.setTranslation(cloud_center);
  boxFilter.setMin(Eigen::Vector4f(-threshold, -threshold, -threshold, 1.0));
  boxFilter.setMax(Eigen::Vector4f(threshold, threshold, threshold, 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*cloud_filtered);
}

void Filter::filter_floor(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
  // Move cloud up if it has negative values
  bool displace = false;
  for (size_t i = 0; i < cloud->points.size() && !displace; i++)
  {
    if (cloud->points[i].z < 0)
      displace = true;
  }

  // Apply z_offset
  if (displace)
    Utils::displaceCloud(cloud, cloud, 0, 0, 0.35);

  // ROS_INFO("create segmenter with threshold: %f",threshold);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  // Search for a plane perpendicular to z axis, 1 degree tolerance
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(1.0));
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  seg.setInputCloud(cloud);

  pcl::ModelCoefficients coeff;
  pcl::PointIndices indices_internal;
  seg.segment(indices_internal, coeff);

  if (indices_internal.indices.size() == 0)
  {
    ROS_ERROR("Unable to find floor.");
  }
  else
  {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(boost::make_shared<std::vector<int>>(indices_internal.indices));
    extract_indices_filter.setNegative(true);
    extract_indices_filter.filter(*cloud_filtered);
  }
}

void Filter::setLeafSize(double new_leaf_size)
{
  leaf_size_ = new_leaf_size;
}

void Filter::run(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
  pcl::copyPointCloud(*cloud, *cloud_filtered);

  // Filter floor
  if (floor_filter_threshold_ > 0)
    filter_floor(floor_filter_threshold_, cloud_filtered, cloud_filtered);

  // Filter noise
  if (noise_filter_threshold_ > 0)
  {
    // with given center
    if (user_given_center_)
      filter_noise(cloud_center_, noise_filter_threshold_, cloud_filtered, cloud_filtered);
    // without given center
    else
      filter_noise(noise_filter_threshold_, cloud_filtered, cloud_filtered);
  }

  // Downsample
  downsampleCloud(cloud_filtered, cloud_filtered);
}