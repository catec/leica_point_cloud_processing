/**
 * @file Utils.cpp
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


#include <Utils.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

const std::string TARGET_CLOUD_TOPIC = "/cad/cloud";
const std::string SOURCE_CLOUD_TOPIC = "/scan/cloud";

bool Utils::getNormals(PointCloudRGB::Ptr& cloud, double normal_radius, PointCloudNormal::Ptr& normals)
{
  ROS_INFO("Computing normals with radius: %f", normal_radius);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);

  PointCloudNormal::Ptr cloud_normals(new PointCloudNormal);
  ne.setRadiusSearch(normal_radius);
  ne.compute(*normals);

  // As we compute normal for each point in cloud, must have same size
  bool success = normals->points.size() == cloud->points.size() ? true : false;

  return success;
}

bool Utils::isValidCloud(PointCloudXYZ::Ptr cloud)
{
  return (cloud->size() > 1);
}

bool Utils::isValidCloud(PointCloudRGB::Ptr cloud)
{
  return (cloud->size() > 1);
}

bool Utils::isValidCloud(PointCloudNormal::Ptr normals)
{
  return (normals->size() > 1);
}

bool Utils::isValidMesh(pcl::PolygonMesh::Ptr mesh)
{
  return (mesh->cloud.row_step*mesh->cloud.height == 0);
}

bool Utils::isValidCloudMsg(const sensor_msgs::PointCloud2& cloud_msg)
{
  // int len = sizeof(cloud_msg.data)/sizeof(cloud_msg.data[0]);  // not working
  int len = cloud_msg.row_step * cloud_msg.height;
  if (len == 0)
  {
    return false;
  }
  return true;
}

bool Utils::isValidTransform(Eigen::Matrix4f transform)
{
  for (int i=0; i<transform.rows(); i++)
  {
    for (int j=0; j<transform.cols(); j++)
    {
      if (std::isnan(transform(i,j)))
        return false;
    }
  }
  return true;
}

void Utils::colorizeCloud(PointCloudRGB::Ptr cloud_rgb, int R, int G, int B)
{
  for (size_t i = 0; i < cloud_rgb->points.size(); i++)
  {
    cloud_rgb->points[i].r = R;
    cloud_rgb->points[i].g = G;
    cloud_rgb->points[i].b = B;
  }
}

void Utils::cloudToXYZRGB(PointCloudXYZ::Ptr cloud, PointCloudRGB::Ptr cloud_rgb, int R, int G, int B)
{
  pcl::copyPointCloud(*cloud, *cloud_rgb);
  colorizeCloud(cloud_rgb, R, G, B);
}

// TODO return sensor_msgs::PointCloud2
void Utils::cloudToROSMsg(PointCloudRGB::Ptr cloud, sensor_msgs::PointCloud2& cloud_msg, const std::string& frameid)
{
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = frameid;
  cloud_msg.header.stamp = ros::Time::now();
}

void Utils::cloudToROSMsg(const pcl::PCLPointCloud2& cloud, sensor_msgs::PointCloud2& cloud_msg, const std::string& frameid)
{
  pcl_conversions::fromPCL(cloud, cloud_msg);
  cloud_msg.header.frame_id = frameid;
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
    // Considering the second neighbor since the first is the point itself.
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
    // Considering the second neighbor since the first is the point itself.
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

void Utils::printTransform(const Eigen::Matrix4f& transform)
{
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "\t\t\t\t[", "]");
  std::cout << transform.format(CleanFmt) << std::endl;
}

void Utils::printMatrix(const Eigen::Ref<const Eigen::MatrixXf>& matrix, const int size)
{
  Eigen::IOFormat CleanFmt(size, 0, ", ", "\n", "\t\t\t\t[", "]");
  std::cout << matrix.format(CleanFmt) << std::endl;
}

void Utils::onePointCloud(PointCloudRGB::Ptr cloud, int size,
                                     PointCloudRGB::Ptr one_point_cloud)
{
  // single point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::PointXYZRGB p;
  p.x = centroid[0];
  p.y = centroid[1];
  p.z = centroid[2];
  
  for(int i=0; i<size; i++)
    one_point_cloud->points.push_back(p);
}              

void Utils::translateCloud(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, 
                          double x_offset, double y_offset, double z_offset)
{
  pcl::copyPointCloud(*cloud_in, *cloud_out);
  for (size_t i = 0; i < cloud_out->points.size(); i++)
  {
    cloud_out->points[i].x += x_offset;
    cloud_out->points[i].y += y_offset;
    cloud_out->points[i].z += z_offset;
  }
}

void Utils::rotateCloud(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, 
                        double roll, double pitch, double yaw)
{
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Quaternion<float> q = rollAngle * pitchAngle * yawAngle;
  
  Eigen::Matrix3f rotation = q.matrix();
  
  Eigen::Matrix4f T;
  T.setIdentity();
  T.block<3,3>(0,0) = rotation;
  pcl::transformPointCloud(*cloud_in, *cloud_out, T);

  Utils::printTransform(T);
}

void Utils::getVectorFromNormal(PointCloudNormal::Ptr normal, double idx,
                                           Eigen::Vector3f& vector)
{
    double x = normal->points[idx].normal_x;
    double y = normal->points[idx].normal_y;
    double z = normal->points[idx].normal_z;

    vector << x, y, z;
    vector.normalize();
}   

bool Utils::searchForSameRows(Eigen::MatrixXd source_m, Eigen::MatrixXd target_m,
                                         std::vector<int>& source_indx, std::vector<int>& target_indx)
{
  double th = 4e-2;
  int s_row_size = source_m.row(0).size();
  int t_row_size = target_m.row(0).size();
  int size = s_row_size < t_row_size ? s_row_size : t_row_size;
  
  int count=0;
  int s_row, t_row;
  for(int i=0; i<size && count<2; i++)
  {
    s_row = i;
    t_row = findOnMatrix(source_m.row(s_row), target_m, th);
    if(t_row != -1)
    {
      // if row found in matrix, both rows are coincident 
      source_indx.push_back(s_row);
      target_indx.push_back(t_row);
      count++;
    }
  }
  
  if(count<2) return false; 

  return true;
}

bool Utils::areSameVectors(const Eigen::VectorXd v1, const Eigen::VectorXd v2, double threshold)
{
    if(v1.size() != v2.size())
      return false;

    if (v1.isApprox(v2, threshold))
        return true;
    
    return false;
}

int Utils::findOnVector(double value, const Eigen::VectorXd v, double threshold)
{
    int index;

    if (v.size()!=0 && (v.array() - value).abs().minCoeff(&index) <= threshold)
        return index;
    
    return -1;
}

int Utils::findOnMatrix(const Eigen::VectorXd v, const Eigen::MatrixXd m,  double threshold)
{
    int index = -1;
    
    for(int i=0; i<v.size() && index==-1; i++)
      if(areSameVectors(v, m.row(i), threshold)) index = i;
    
    return index;
}

void Utils::extractColFromMatrix(Eigen::MatrixXd& matrix, int col)
{
  int nrows = matrix.rows();
  int ncols = matrix.cols()-1;
  
  if(col < ncols)
      matrix.block(0,col,nrows,ncols-col) = matrix.rightCols(ncols-col);

  matrix.conservativeResize(nrows,ncols);

}

void Utils::extractRowFromMatrix(Eigen::MatrixXd& matrix, int row)
{
  int nrows = matrix.rows()-1;
  int ncols = matrix.cols();
  
  if(row < nrows)
      matrix.block(row,0,nrows-row,ncols) = matrix.bottomRows(nrows-row);

  matrix.conservativeResize(nrows,ncols);
}