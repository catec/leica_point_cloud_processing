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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/angles.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>

Filter::Filter(double leaf_size)
{
  leaf_size_ = leaf_size;
  noise_threshold_ = 0;
  floor_threshold_ = 0;
  given_center_ = false;
}

Filter::Filter(double leaf_size, double noise_threshold, double floor_threshold)
{
  leaf_size_ = leaf_size;
  noise_threshold_ = noise_threshold;
  floor_threshold_ = floor_threshold;
  given_center_ = false;
}

void Filter::setLeafSize(double leaf_size)
{
  leaf_size_ = leaf_size;
}

void Filter::setCloudCenter(Eigen::Vector3f cloud_center)
{
  cloud_center_[0] = cloud_center[0];
  cloud_center_[1] = cloud_center[1];
  cloud_center_[2] = cloud_center[2];
  given_center_ = true;
}

void Filter::setNoiseThreshold(double noise_th)
{
  noise_threshold_ = noise_th;
}

void Filter::setFloorThreshold(double floor_th)
{
  floor_threshold_ = floor_th;
}

void Filter::run(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
  // outlierRemove(cloud, cloud_filtered);
  pcl::copyPointCloud(*cloud, *cloud_filtered);

  // Filter floor
  if (floor_threshold_ > 0)
    filterFloor(cloud_filtered, cloud_filtered);

  // Filter noise
  if (noise_threshold_ > 0)
    filterNoise(cloud_filtered, cloud_filtered);
}

void Filter::outlierRemove(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
  // ROS_INFO("Filtering outliers");
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  // sor.setInputCloud(cloud);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*cloud_filtered);

  // ROS_INFO("Filtering radius");
  // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
  // ror.setInputCloud(cloud);
  // ror.setRadiusSearch(noise_threshold_);
  // ror.setMinNeighborsInRadius(10);
  // ror.filter(*cloud_filtered);
}

void Filter::downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled)
{
  ROS_INFO("Downsample cloud with leaf_size : %f", leaf_size_);
  double res = Utils::computeCloudResolution(cloud);
  ROS_INFO("Pointcloud resolution before downsampling: %f", res);

  const Eigen::Vector4f downsampling_leaf_size(leaf_size_, leaf_size_, leaf_size_, 0.0f);
  pcl::VoxelGrid<pcl::PointXYZRGB> downsampling_filter;
  downsampling_filter.setInputCloud(cloud);
  downsampling_filter.setLeafSize(downsampling_leaf_size);
  downsampling_filter.filter(*cloud_downsampled);

  res = Utils::computeCloudResolution(cloud_downsampled);
  ROS_INFO("Pointcloud resolution after downsampling: %f", res);
}

void Filter::filterNoise(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
  pcl::CropBox<pcl::PointXYZRGB> boxFilter;

  if(!given_center_)
    pcl::compute3DCentroid(*cloud, cloud_center_);
 
  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = cloud_center_[0];
  boxTranslatation[1] = cloud_center_[1];
  boxTranslatation[2] = cloud_center_[2];
  ROS_INFO("Point cloud center: (%f,%f,%f)", cloud_center_[0], cloud_center_[1], cloud_center_[2]);
  ROS_INFO("Filter noise with threshold: (%f,%f,%f)", noise_threshold_, noise_threshold_, noise_threshold_/5);
  
  boxFilter.setTranslation(boxTranslatation);
  boxFilter.setMin(Eigen::Vector4f(-noise_threshold_, -noise_threshold_, -noise_threshold_/5, 1.0));
  boxFilter.setMax(Eigen::Vector4f(noise_threshold_, noise_threshold_, noise_threshold_/5, 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*cloud_filtered);

  if (!Utils::isValidCloud(cloud_filtered))
  {
    ROS_ERROR("Unable to filter noise. Unfiltered cloud");
    pcl::copyPointCloud(*cloud, *cloud_filtered);
  }  
}

void Filter::filterFloor(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
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
    Utils::translateCloud(cloud, cloud, 0, 0, 0.35);

  // Search for a plane perpendicular to z axis
  Eigen::Vector3f axis(0, 0, 1);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(1.0)); // 1 degree tolerance
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(floor_threshold_);
  seg.setInputCloud(cloud);

  pcl::ModelCoefficients coeff;
  pcl::PointIndices floor;
  seg.segment(floor, coeff);
  
  if(floor.indices.size() == 0)
  {
    ROS_ERROR("Unable to find floor. Unfiltered cloud");
    pcl::copyPointCloud(*cloud, *cloud_filtered);
  }
  else
  {
    Filter::extractIndicesNegative(cloud, cloud_filtered, 
                                  boost::make_shared<std::vector<int>>(floor.indices));
  }
}

void Filter::removeFromCloud(PointCloudRGB::Ptr input_cloud, 
                             PointCloudRGB::Ptr substract_cloud,
                             double threshold, 
                             PointCloudRGB::Ptr cloud_filtered)
{
  ROS_INFO("difference from segment with threshold: %f", threshold);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::SegmentDifferences<pcl::PointXYZRGB> segment;
  segment.setInputCloud(input_cloud);
  segment.setTargetCloud(substract_cloud);
  segment.setSearchMethod(tree);
  segment.setDistanceThreshold(threshold);
  segment.segment(*cloud_filtered);
}

void Filter::extractIndicesNegative(PointCloudRGB::Ptr cloud_in, 
                                    PointCloudRGB::Ptr cloud_out,
                                    pcl::IndicesPtr indices)
{
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
  extract_indices_filter.setInputCloud(cloud_in);
  extract_indices_filter.setIndices(indices);
  extract_indices_filter.setNegative(true);
  extract_indices_filter.filter(*cloud_out);
}

void Filter::extractIndices(PointCloudRGB::Ptr cloud_in,
                            PointCloudRGB::Ptr cloud_out,
                            pcl::IndicesPtr indices)
{
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
  extract_indices_filter.setInputCloud(cloud_in);
  extract_indices_filter.setIndices(indices);
  extract_indices_filter.filter(*cloud_out);
}

void Filter::extractIndices(PointCloudNormal::Ptr normals_in, 
                            PointCloudNormal::Ptr normals_out,
                            pcl::IndicesPtr indices)
{
  pcl::ExtractIndices<pcl::Normal> extract_indices_filter;
  extract_indices_filter.setInputCloud(normals_in);
  extract_indices_filter.setIndices(indices);
  extract_indices_filter.filter(*normals_out);
}