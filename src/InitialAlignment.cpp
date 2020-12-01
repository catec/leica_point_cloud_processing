/**
 * @file InitialAlignment.cpp
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

#include <InitialAlignment.h>
#include <Filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

InitialAlignment::InitialAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud)
  : aligned_cloud_(new PointCloudRGB),
    source_normals_(new PointCloudNormal),
    target_normals_(new PointCloudNormal),
    source_dominant_normals_(new PointCloudNormal),
    target_dominant_normals_(new PointCloudNormal),
    target_keypoints_(new PointCloudRGB),
    source_keypoints_(new PointCloudRGB),
    target_features_(new PointCloudFPFH),
    source_features_(new PointCloudFPFH),
    correspondences_(new pcl::Correspondences)
{
  target_cloud_ = target_cloud;
  source_cloud_ = source_cloud;

  pcl::compute3DCentroid(*source_cloud_, source_centroid_);
  pcl::compute3DCentroid(*target_cloud_, target_centroid_);

  ROS_INFO("Source cloud center: (%f, %f, %f)", source_centroid_[0], source_centroid_[1], source_centroid_[2]);
  ROS_INFO("Target cloud center: (%f, %f, %f)", target_centroid_[0], target_centroid_[1], target_centroid_[2]);

  double target_res = Utils::computeCloudResolution(target_cloud_);
  double source_res = Utils::computeCloudResolution(source_cloud_);
  normal_radius_ = (target_res + source_res) * 10.0;  // 10 times higher
  transform_exists_ = false;
  rigid_tf_ = Eigen::Matrix4f::Identity();
  method_ = AlignmentMethod::BOUNDARY; // default
  radius_factor_ = 1.4;
}

void InitialAlignment::run()
{
  if (method_ >= AlignmentMethod::NONE)
    ROS_INFO("Not running initial alignment method");
  
  else
  {
    Utils::getNormals(source_cloud_, normal_radius_, source_normals_);
    Utils::getNormals(target_cloud_, normal_radius_, target_normals_);

    if (method_ == AlignmentMethod::NORMALS)// TODO control errors
      runNormalsBasedAlgorithm();
    else
      runKeypointsBasedAlgorithm();
  }
  
  applyTFtoCloud();
}

void InitialAlignment::runNormalsBasedAlgorithm()
{
  ROS_INFO("Initial alignment method: NORMALS");

  // filter edges from cloud
  pcl::IndicesPtr source_non_boundary_indices(new std::vector<int>);
  pcl::IndicesPtr target_non_boundary_indices(new std::vector<int>);
  pcl::IndicesPtr keypoints_indices(new std::vector<int>);
  PointCloudRGB::Ptr keypoints_cloud(new PointCloudRGB);
  obtainBoundaryKeypoints(source_cloud_, source_normals_, keypoints_cloud, keypoints_indices, source_non_boundary_indices);
  obtainBoundaryKeypoints(target_cloud_, target_normals_, keypoints_cloud, keypoints_indices, target_non_boundary_indices);
  
  PointCloudRGB::Ptr source_cloud_no_boundary(new PointCloudRGB);
  PointCloudRGB::Ptr target_cloud_no_boundary(new PointCloudRGB);
  PointCloudNormal::Ptr source_normals_no_boundary(new PointCloudNormal);
  PointCloudNormal::Ptr target_normals_no_boundary(new PointCloudNormal);
  Filter::extractIndices(source_cloud_, source_cloud_no_boundary, source_non_boundary_indices);
  Filter::extractIndices(target_cloud_, target_cloud_no_boundary, target_non_boundary_indices); 
  Filter::extractIndices(source_normals_, source_normals_no_boundary, source_non_boundary_indices);
  Filter::extractIndices(target_normals_, target_normals_no_boundary, target_non_boundary_indices);

  // start normal algorithm
  std::vector<pcl::PointIndices> source_cluster_indices;
  std::vector<pcl::PointIndices> target_cluster_indices;
  obtainNormalsCluster(source_cloud_no_boundary, source_normals_no_boundary, source_cluster_indices);
  obtainNormalsCluster(target_cloud_no_boundary, target_normals_no_boundary, target_cluster_indices);

  obtainDominantNormals(source_normals_no_boundary, source_cluster_indices, source_dominant_normals_);
  obtainDominantNormals(target_normals_no_boundary, target_cluster_indices, target_dominant_normals_);

  if(Utils::isValidCloud(source_dominant_normals_) && Utils::isValidCloud(target_dominant_normals_))
    getTransformationFromNormals(source_dominant_normals_, target_dominant_normals_);
  else
    ROS_ERROR("Failed to obtain dominant normals");
}

void InitialAlignment::runKeypointsBasedAlgorithm()
{
  obtainKeypointsAndFeatures(source_cloud_, source_normals_, source_keypoints_, source_features_);
  obtainKeypointsAndFeatures(target_cloud_, target_normals_, target_keypoints_, target_features_);

  obtainCorrespondences();
  rejectCorrespondences();
  if (correspondences_->size() >  (source_cloud_->size()/2))
    rejectOneToOneCorrespondences();
  
  estimateTransform();
}

void InitialAlignment::obtainNormalsCluster(PointCloudRGB::Ptr cloud,
                                            PointCloudNormal::Ptr normals,
                                            std::vector<pcl::PointIndices>& cluster_indices)
{
  // move all points from cloud to cloud centroid
  PointCloudRGB::Ptr cloud_one(new PointCloudRGB);
  Utils::onePointCloud(cloud, cloud->size(), cloud_one);

  // parameters to cluster normals
  const float tolerance = 0.05f; // 5cm tolerance
  const double eps_angle = 20 * (M_PI / 180.0); // 20 degree tolerance
  const double PERCENTAJE = 0.025;
  const unsigned int min_cluster_size = (int)normals->size()*PERCENTAJE;

  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
  tree->setInputCloud(cloud);
  ROS_INFO("Clustering normals with min cluster size: %d...", min_cluster_size);

  pcl::extractEuclideanClusters(*cloud, *normals, tolerance, tree, cluster_indices, eps_angle, min_cluster_size);

  ROS_INFO("Clusters found: %zd", cluster_indices.size());

  if(cluster_indices.size() == 0)
  {
    ROS_ERROR("Failed to obtain normals cluster");
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
    for(int i=0; i<cloud->size(); i++)
      point_indices->indices.push_back(i);

    cluster_indices.push_back(*point_indices);
  } 
}

void InitialAlignment::obtainDominantNormals(PointCloudNormal::Ptr normals,
                                             std::vector<pcl::PointIndices> cluster_indices,
                                             PointCloudNormal::Ptr dominant_normals)
{
  for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); it++)
  {
    PointCloudNormal::Ptr normal_cluster(new PointCloudNormal);
    for (const auto &index : it->indices)
      normal_cluster->push_back((*normals)[index]); 

    Eigen::Vector4f vector_dominant_normal=normal_cluster->points[0].getNormalVector4fMap();
    for (const auto &normal : normal_cluster->points)
    {
      Eigen::Vector4f vector_normal = normal.getNormalVector4fMap();
      vector_dominant_normal += vector_normal;
    }
    vector_dominant_normal.normalize();
    pcl::Normal p;
    p.normal_x = vector_dominant_normal.x();
    p.normal_y = vector_dominant_normal.y();
    p.normal_z = vector_dominant_normal.z();

    dominant_normals->points.push_back(p);
  }
}

void InitialAlignment::getTransformationFromNormals(PointCloudNormal::Ptr source_normals,
                                              PointCloudNormal::Ptr target_normals)
{
  std::vector<int> s_idx, t_idx;
  normalsCorrespondences(source_normals, target_normals, s_idx, t_idx);

  if(s_idx.size() != t_idx.size() || s_idx.size()<2) ROS_ERROR("Index sizes not match");
  else 
  {
    ROS_INFO("Found correspondent normals");

    Eigen::Matrix4f transform;
    transform.setIdentity();
    // set rotation
    Eigen::Matrix3f source_rot = getCoordinateSystem(source_normals, s_idx);
    Eigen::Matrix3f target_rot = getCoordinateSystem(target_normals, t_idx);
    transform.block<3,3>(0,0) = target_rot * source_rot.transpose();
    // set translation
    Eigen::Vector4f diff_pose =  target_centroid_ - source_centroid_;
    Eigen::Vector4f translation = transform * diff_pose; 
    transform(0,3) = translation[0];
    transform(1,3) = translation[1];
    transform(2,3) = translation[2];
    Utils::printTransform(transform);

    if(transform != Eigen::Matrix4f::Zero())
    {
      transform_exists_ = true;
      rigid_tf_ = transform;
    } 
  }
} 


Eigen::VectorXd InitialAlignment::nextVector(Eigen::MatrixXd matrix,
                                             int current_col, int current_row)
{
  int ncols = matrix.row(0).size() - (current_col+1);
  Eigen::MatrixXd v(1, ncols);
  v = matrix.block(current_row, current_col+1, 1, ncols);

  return v.row(0);
}

bool InitialAlignment::findIdx(Eigen::MatrixXd source_m, Eigen::MatrixXd target_m,
                               Eigen::MatrixXi source_idx_m, Eigen::MatrixXi target_idx_m,
                               std::vector<int>& source_indx, std::vector<int>& target_indx)
{
    double th = 4e-2;

    if(source_m.rows()<2 || target_m.rows()<2)
    {
      ROS_WARN("Couldn't start findIdx on matrix of size < 2");
      source_indx.push_back(0);
      target_indx.push_back(0);
      return false;
    }

    int count = 0; // count number of coincidences
    int t_col = -1;
    for(int s_col=0; s_col<(source_m.cols()-2) && count<2; s_col++)
    {
      for (int s_row=0; s_row<source_m.rows(); s_row++)
      {
        bool found = false;
        if (std::find(source_indx.begin(), source_indx.end(), s_row) == source_indx.end())
        {
          for (int t_row=0; t_row<target_m.rows() && !found; t_row++)
          {
            if (std::find(target_indx.begin(), target_indx.end(), t_row) == target_indx.end())
            {
              t_col = Utils::findOnVector(source_m(s_row, s_col), target_m.row(t_row), th);
              if (t_col != -1)
              {
                // if value found, check for next value
                Eigen::VectorXd v = nextVector(target_m, t_col, t_row);
                if(Utils::findOnVector(source_m(s_row, s_col+1), v, th) == -1) 
                  t_col = -1; // if next value not found, rows are not coincident
                else
                {
                  // if next value found, both rows are coincident
                  source_indx.push_back(s_row);
                  target_indx.push_back(t_row);
                  count++;
                  source_indx.push_back(source_idx_m(s_row,s_col));
                  target_indx.push_back(target_idx_m(t_row,t_col));
                  count++;
                  found = true;
                }
              }
            }
          }
        }
      }
    }
    if(count<2) return false;

    return true;
}  

bool InitialAlignment::normalsCorrespondences(PointCloudNormal::Ptr source_dominant_normal,
                                   PointCloudNormal::Ptr target_dominant_normal,
                                   std::vector<int>& source_idx, std::vector<int>& target_idx)
{
  size_t source_size = source_dominant_normal->size();
  int source_m_size = (int)source_size - 1 ;
  size_t target_size = target_dominant_normal->size();
  int target_m_size = (int)target_size - 1 ;

  if(source_m_size==1 || target_m_size==1) ROS_WARN("Matrix size=one could case some errors");

  std::vector<double> source_angle_vector, target_angle_vector;
  Eigen::MatrixXd source_m(source_m_size, source_m_size);
  Eigen::MatrixXd target_m(target_m_size, target_m_size);
  Eigen::MatrixXi source_idx_m(source_m_size, source_m_size);
  Eigen::MatrixXi target_idx_m(target_m_size, target_m_size);

  matrixFromAngles(source_dominant_normal, source_m_size, source_m, source_idx_m);
  matrixFromAngles(target_dominant_normal, target_m_size, target_m, target_idx_m);

  if(source_m.row(0).size() == target_m.row(0).size())
    if (Utils::searchForSameRows(source_m, target_m, source_idx, target_idx)) return true;

  ROS_INFO("No rows coincident. Search for approximations...");

  if(source_m.rows() < target_m.rows())
  {
    if(!findIdx(source_m, target_m, source_idx_m, target_idx_m, source_idx, target_idx))
        return false;
  }
  else
  {
    if(!findIdx(target_m, source_m, target_idx_m, source_idx_m, target_idx, source_idx))
        return false;
  }  
  return true;
}

Eigen::Matrix3f InitialAlignment::getCoordinateSystem(PointCloudNormal::Ptr normals, 
                                                      std::vector<int>& index)
{
  Eigen::Vector3f v1, v2;

  Utils::getVectorFromNormal(normals, index[0], v1);
  Utils::getVectorFromNormal(normals, index[1], v2);

  Eigen::Vector3f orto_v = v1.cross(v2);
  orto_v.normalize();

  Eigen::Matrix3f coord_sys;
  coord_sys.col(0) = v1;
  coord_sys.col(1) = orto_v;
  coord_sys.col(2) = v1.cross(orto_v).normalized();
  return coord_sys;
}

void InitialAlignment::matrixFromAngles(PointCloudNormal::Ptr normal,
                                        size_t size,
                                        Eigen::MatrixXd& matrix,
                                        Eigen::MatrixXi& index_matrix)
{
    for (int row=0; row<size; row++)
    {
        pcl::Normal n = normal->points[row];
        Eigen::Vector4f init_v = n.getNormalVector4fMap();

        for(int col=0, j=row+1; col<size; col++, j++)
        {
            if (j>size) j=0;
            n = normal->points[j];
            Eigen::Vector4f v = n.getNormalVector4fMap();
            matrix(row,col) = pcl::getAngle3D(init_v, v);
            index_matrix(row,col) = j;
        }
    }
}

int InitialAlignment::obtainKeypointsAndFeatures(PointCloudRGB::Ptr cloud, 
                                                 PointCloudNormal::Ptr normals,
                                                 PointCloudRGB::Ptr keypoints_cloud,
                                                 PointCloudFPFH::Ptr features_cloud)
{
  pcl::IndicesPtr keypoints_indices(new std::vector<int>);

  if (method_ == AlignmentMethod::HARRIS)// TODO control errors
  {
    ROS_INFO("Initial alignment method: HARRIS");
    obtainHarrisKeypoints(cloud, normals, keypoints_cloud, keypoints_indices); 
    obtainFeatures(cloud, normals, keypoints_indices, features_cloud);
  }

  if (method_ == AlignmentMethod::BOUNDARY)
  {
    ROS_INFO("Initial alignment method: BOUNDARY");
    pcl::IndicesPtr non_keypoints_indices(new std::vector<int>);
    if (obtainBoundaryKeypoints(cloud, normals, keypoints_cloud, keypoints_indices, non_keypoints_indices) == -1) 
      ROS_ERROR("obtainBoundaryKeypoints");
    if (obtainFeatures(cloud, normals, keypoints_indices, features_cloud) == -1) 
      ROS_ERROR("obtainFeatures");
  }

  if(method_ == AlignmentMethod::MULTISCALE)
  {
    ROS_INFO("Initial alignment method: MULTISCALE");
    obtainMultiScaleKeypointsAndFeatures(cloud, normals, keypoints_cloud, keypoints_indices, features_cloud);  
  }

  Utils::colorizeCloud(keypoints_cloud, 0, 255, 0);
}

int InitialAlignment::obtainHarrisKeypoints(PointCloudRGB::Ptr cloud,
                                            PointCloudNormal::Ptr normals,
                                            PointCloudRGB::Ptr keypoints_cloud,
                                            pcl::IndicesPtr keypoints_indices)
{
  double cloud_res = Utils::computeCloudResolution(cloud);
  double keypoint_radius = cloud_res * radius_factor_;
    
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> detector;
  detector.setNonMaxSupression(true);
  // detector.setRefine(true);
  detector.setInputCloud(cloud);
  detector.setNormals(normals);
  detector.setThreshold(1e-6);
  detector.setRadius(keypoint_radius);
  detector.compute(*keypoints);
  pcl::PointIndicesConstPtr k_indices = detector.getKeypointsIndices();

  if (k_indices->indices.empty()) return -1;
    
  *keypoints_indices = k_indices->indices;
  Filter::extractIndices(cloud, keypoints_cloud, keypoints_indices);
  
  return 0;
}

int InitialAlignment::obtainBoundaryKeypoints(PointCloudRGB::Ptr cloud,
                                              PointCloudNormal::Ptr normals,
                                              PointCloudRGB::Ptr keypoints_cloud,
                                              pcl::IndicesPtr keypoints_indices,
                                              pcl::IndicesPtr non_keypoints_indices)
{
  double cloud_res = Utils::computeCloudResolution(cloud);
  double keypoint_radius = cloud_res * radius_factor_;
  ROS_INFO("obtainBoundaryKeypoints:  cloud_res %f, keypoint_radius %f", cloud_res, keypoint_radius);

  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> detector;
  detector.setInputCloud(cloud);
  detector.setInputNormals(normals);
  detector.setKSearch(30); 
  detector.setAngleThreshold(1.047f); // angle close to 1.57rad to detect corners
  detector.compute(boundaries);

  if(boundaries.size() != cloud->size()) return -1;

  for (int i=0; i<cloud->size(); i++)
  {
    if (boundaries.points[i].boundary_point == 1)
      {
        keypoints_indices->push_back(i);
        keypoints_cloud->points.push_back(cloud->points[i]);  
      }
    else
      non_keypoints_indices->push_back(i);
	}

  if(keypoints_indices->size() == 0) return -1;

  return 0;
}                                            

int InitialAlignment::obtainMultiScaleKeypointsAndFeatures(PointCloudRGB::Ptr cloud,
                                                           PointCloudNormal::Ptr normals,
                                                           PointCloudRGB::Ptr keypoints_cloud,
                                                           pcl::IndicesPtr keypoints_indices,
                                                           PointCloudFPFH::Ptr features)
{
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr fest(
      new pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
  fest->setInputCloud(cloud);
  fest->setInputNormals(normals);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  fest->setSearchMethod(tree);

  std::vector<float> scale_values{0.5f, 1.0f, 1.5f};
  pcl::MultiscaleFeaturePersistence<pcl::PointXYZRGB, pcl::FPFHSignature33> fper;
  fper.setScalesVector(scale_values);
  fper.setAlpha(0.45f);
  fper.setFeatureEstimator(fest);
  fper.setDistanceMetric(pcl::CS);
  fper.determinePersistentFeatures(*features, keypoints_indices);
  
  Filter::extractIndices(cloud, keypoints_cloud, keypoints_indices);
  
  if(keypoints_indices->size() == 0) return -1;
  if(keypoints_indices->size() != features->size()) return -1;

  return 0;
}                                            

int InitialAlignment::obtainFeatures(PointCloudRGB::Ptr cloud,
                                     PointCloudNormal::Ptr normals,
                                     pcl::IndicesPtr keypoints_indices,
                                     PointCloudFPFH::Ptr features)
{
  double cloud_res = Utils::computeCloudResolution(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr fest(
    new pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
  fest->setInputCloud(cloud);
  fest->setInputNormals(normals);
  fest->setIndices(keypoints_indices);
  fest->setSearchMethod(tree);    
  fest->setRadiusSearch(10*cloud_res); // TODO
  fest->compute(*features);

  if (features->size() == 0) return -1;

  return 0;
}

void InitialAlignment::obtainCorrespondences()
{
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> cest;
    cest.setInputSource(source_features_);
    cest.setInputTarget(target_features_);
    cest.determineCorrespondences(*correspondences_);
    ROS_INFO("Found %zd correspondences", correspondences_->size());
}

void InitialAlignment::rejectCorrespondences()
{
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector;
    rejector.setInputSource(source_keypoints_);
    rejector.setInputTarget(target_keypoints_);
    rejector.setInlierThreshold(1.5);
    rejector.setRefineModel(true);
    rejector.setInputCorrespondences(correspondences_);
    rejector.getCorrespondences(*correspondences_);
    ROS_INFO("th: %f", rejector.getInlierThreshold());
    ROS_INFO("Filtered %zd correspondences", correspondences_->size());
}

void InitialAlignment::rejectOneToOneCorrespondences()
{
    pcl::registration::CorrespondenceRejectorOneToOne rejector;
    rejector.setInputCorrespondences(correspondences_);
    rejector.getCorrespondences(*correspondences_);
    ROS_INFO("Found %zd one to one correspondences:", correspondences_->size());
}

void InitialAlignment::estimateTransform()
{
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> trans_est;
    trans_est.estimateRigidTransformation(*source_keypoints_, *target_keypoints_, *correspondences_, rigid_tf_);
    if(rigid_tf_ != Eigen::Matrix4f::Zero()) transform_exists_ = true;
}

void InitialAlignment::applyTFtoCloud()
{
  pcl::transformPointCloud(*source_cloud_, *aligned_cloud_, rigid_tf_);
}

void InitialAlignment::applyTFtoCloud(PointCloudRGB::Ptr cloud)
{
  pcl::transformPointCloud(*cloud, *cloud, rigid_tf_);
}

void InitialAlignment::applyTFtoCloud(PointCloudRGB::Ptr cloud, Eigen::Matrix4f tf)
{
  pcl::transformPointCloud(*cloud, *cloud, tf);
}

void InitialAlignment::getAlignedCloud(PointCloudRGB::Ptr aligned_cloud)
{
  pcl::copyPointCloud(*aligned_cloud_, *aligned_cloud);
}

void InitialAlignment::getAlignedCloudROSMsg(sensor_msgs::PointCloud2& aligned_cloud_msg)
{
  Utils::cloudToROSMsg(aligned_cloud_, aligned_cloud_msg);
}

Eigen::Matrix4f InitialAlignment::getRigidTransform()
{
  return rigid_tf_;
}

void InitialAlignment::setMethod(AlignmentMethod method)
{
  method_ = method;
}

void InitialAlignment::setRadiusFactor(double radius_factor)
{
  radius_factor_ = radius_factor;
}