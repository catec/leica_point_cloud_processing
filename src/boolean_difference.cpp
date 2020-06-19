#include <boolean_difference.h>

BooleanDifference::BooleanDifference(PointCloudRGB::Ptr cloud)
      : _diff_indices(new IndicesVector), _result_cloud(new PointCloudRGB)
{
  _cloud = cloud;
  computeResolution(cloud);
  substract_error = false;
}

void BooleanDifference::substract(PointCloudRGB::Ptr cloud_to_substract)
{
  setOctreeAndGetIndices(cloud_to_substract);
  int res = computeResultCloud();
  if (res!=0)
    substract_error = true;
}


double BooleanDifference::getVoxelResolution()
{
  return _voxel_resolution;
}

void BooleanDifference::getResultCloud(PointCloudRGB::Ptr result_cloud)
{
  pcl::copyPointCloud(*_result_cloud, *result_cloud);
}

void BooleanDifference::computeResolution(PointCloudRGB::Ptr cloud)
{
  double res = Utils::computeCloudResolution(cloud);
  _voxel_resolution = res * 3; // 3 times higher to cover more than two neighbors
  ROS_INFO("Creating octree with resolution: %f", _voxel_resolution);
}


void BooleanDifference::setOctreeAndGetIndices(PointCloudRGB::Ptr cloud_to_substract)
{
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(_voxel_resolution);
  octree.setInputCloud(cloud_to_substract);
  octree.addPointsFromInputCloud();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers();

  octree.setInputCloud(_cloud);
  octree.addPointsFromInputCloud();

  ROS_INFO("Extracting differences");
  octree.getPointIndicesFromNewVoxels(*_diff_indices);// Diferentiated points
}

int BooleanDifference::computeResultCloud()
{
  PointCloudRGB::Ptr result_cloud(new PointCloudRGB);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
  extract_indices_filter.setInputCloud(_cloud);
  extract_indices_filter.setIndices(_diff_indices);
  extract_indices_filter.filter(*result_cloud);

  if (result_cloud->size() <= 0)
  {
    ROS_ERROR("No differences found. Try a small resolution");
    return -1;
  }

  pcl::copyPointCloud(*result_cloud, *_result_cloud);
  return 0;
}

