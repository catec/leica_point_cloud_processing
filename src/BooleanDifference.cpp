/**
 * @file BooleanDifference.cpp
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

#include <BooleanDifference.h>

BooleanDifference::BooleanDifference(PointCloudRGB::Ptr cloud)
  : diff_indices_(new IndicesVector), result_cloud_(new PointCloudRGB)
{
  cloud_ = cloud;
  computeResolution(cloud);
  substract_error_ = false;
}

void BooleanDifference::substract(PointCloudRGB::Ptr cloud_to_substract)
{
  setOctreeAndGetIndices(cloud_to_substract);
  int res = computeResultCloud();
  if (res != 0)
    substract_error_ = true;
}

double BooleanDifference::getVoxelResolution()
{
  return voxel_resolution_;
}

void BooleanDifference::getResultCloud(PointCloudRGB::Ptr result_cloud)
{
  pcl::copyPointCloud(*result_cloud_, *result_cloud);
}

void BooleanDifference::computeResolution(PointCloudRGB::Ptr cloud)
{
  double res = Utils::computeCloudResolution(cloud);
  voxel_resolution_ = res * 3;  // 3 times higher to cover more than two neighbors
  ROS_INFO("Voxelize octree with resolution: %f", voxel_resolution_);
}

void BooleanDifference::setOctreeAndGetIndices(PointCloudRGB::Ptr cloud_to_substract)
{
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(voxel_resolution_);
  octree.setInputCloud(cloud_to_substract);
  octree.addPointsFromInputCloud();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers();

  octree.setInputCloud(cloud_);
  octree.addPointsFromInputCloud();

  ROS_INFO("Extracting differences");
  octree.getPointIndicesFromNewVoxels(*diff_indices_);  // Diferentiated points
}

int BooleanDifference::computeResultCloud()
{
  PointCloudRGB::Ptr result_cloud(new PointCloudRGB);
  Utils::indicesFilter(cloud_, result_cloud, diff_indices_);
  // pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
  // extract_indices_filter.setInputCloud(cloud_);
  // extract_indices_filter.setIndices(diff_indices_);
  // extract_indices_filter.filter(*result_cloud);

  if (result_cloud->size() <= 0)
  {
    ROS_ERROR("No differences found. Try a small resolution");
    return -1;
  }

  pcl::copyPointCloud(*result_cloud, *result_cloud_);
  return 0;
}
