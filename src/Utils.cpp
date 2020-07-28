#include <Utils.h>

void Utils::setPCpath(const std::string &pointcloud_folder_path)
{
  _pc_path = pointcloud_folder_path;
}

bool Utils::getNormals(PointCloudRGB::Ptr &cloud,
                       double normal_radius,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals)
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

bool Utils::isValidCloud(PointCloudXYZ::Ptr cloud)
{
  if (cloud->size()<=0)
  {
    return false;
  }
  return true;
}

bool Utils::isValidCloud(PointCloudRGB::Ptr cloud)
{
  if (cloud->size()<=0)
  {
    return false;
  }
  return true;
}

bool Utils::isValidCloudMsg(const sensor_msgs::PointCloud2 &cloud_msg)
{
  // int len = sizeof(cloud_msg.data)/sizeof(cloud_msg.data[0]);  // not working
  int len = cloud_msg.row_step * cloud_msg.height; 
  if (len == 0)
  {
    return false;
  }
  return true;
}

void Utils::colorizeCloud(PointCloudRGB::Ptr cloud_rgb,
                          int R, int G, int B)
{
  for (size_t i = 0; i < cloud_rgb->points.size(); i++)
  {
    cloud_rgb->points[i].r = R;
    cloud_rgb->points[i].g = G;
    cloud_rgb->points[i].b = B;
  }
}

void Utils::cloudToXYZRGB(PointCloudXYZ::Ptr cloud,
                          PointCloudRGB::Ptr cloud_rgb,
                          int R, int G, int B)
{
  pcl::copyPointCloud(*cloud, *cloud_rgb);
  colorizeCloud(cloud_rgb, R,G,B);
}

void Utils::cloudToROSMsg(PointCloudRGB::Ptr cloud,
                          sensor_msgs::PointCloud2 &cloud_msg)
{
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = _frame_id;
  cloud_msg.header.stamp = ros::Time::now();
}

double Utils::computeCloudResolution(PointCloudXYZ::Ptr cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);

  for (std::size_t i = 0; i < cloud->size(); ++i)
  {
    if (!std::isfinite((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

double Utils::computeCloudResolution(PointCloudRGB::Ptr cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);
  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  tree.setInputCloud(cloud);

  for (std::size_t i = 0; i < cloud->size(); ++i)
  {
    if (!std::isfinite((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void Utils::printTransform(const Eigen::Matrix4f &transform)
{
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "\t\t\t\t[", "]");
  std::cout << transform.format(CleanFmt) << std::endl;
}

void Utils::indicesFilter(PointCloudRGB::Ptr cloud_in,
                          PointCloudRGB::Ptr cloud_out, 
                          boost::shared_ptr<std::vector<int> >  indices)
{
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud_in);
    extract_indices_filter.setIndices(indices);
    extract_indices_filter.filter(*cloud_out);
}

void Utils::displaceCloud(PointCloudRGB::Ptr cloud_in,
                          PointCloudRGB::Ptr cloud_out, 
                          double x_offset, 
                          double y_offset, 
                          double z_offset)
{
  pcl::copyPointCloud(*cloud_in, *cloud_out);
  for (size_t i = 0; i < cloud_out->points.size(); i++)
  {
    cloud_out->points[i].x += x_offset;
    cloud_out->points[i].y += y_offset;
    cloud_out->points[i].z += z_offset;
  }
}

// Initialization
std::string Utils::_pc_path = "";
std::string Utils::_frame_id = "world";