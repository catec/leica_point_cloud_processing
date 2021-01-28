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
#include <Filter.h>
#include <pcl/features/from_meshes.h>
#include <pcl/filters/filter.h>

GICPAlignment::GICPAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud, bool use_covariances)
  : aligned_cloud_(new PointCloudRGB), backup_cloud_(new PointCloudRGB)
{
    target_cloud_ = target_cloud;
    source_cloud_ = source_cloud;
    covariances_ = use_covariances;
    tf_epsilon_ = 4e-3;
    max_iter_ = 100;
    max_corresp_distance_ = 4e-2;
    ransac_outlier_th_ = 1.0;
    transform_exists_ = false;
    fine_tf_ = Eigen::Matrix4f::Identity();
}

void GICPAlignment::run()
{
    configParameters();

    if(covariances_)
        applyCovariances();

    fineAlignment();
    applyTFtoCloud(source_cloud_);
}

void GICPAlignment::configParameters()
{
    gicp_.setMaximumIterations(max_iter_);
    gicp_.setMaxCorrespondenceDistance(max_corresp_distance_);
    gicp_.setTransformationEpsilon(tf_epsilon_);  
    gicp_.setRANSACOutlierRejectionThreshold(ransac_outlier_th_); 
}

void GICPAlignment::getCovariances(PointCloudRGB::Ptr cloud, boost::shared_ptr<CovariancesVector> covs)
{
    double target_res = Utils::computeCloudResolution(target_cloud_);
    double source_res = Utils::computeCloudResolution(source_cloud_);
    double normal_radius = (target_res + source_res) * 2.0; // doubled

    PointCloudNormal::Ptr normals(new PointCloudNormal);
    Utils::getNormals(cloud, normal_radius, normals);

    pcl::IndicesPtr nan_indices(new std::vector<int>);
    pcl::removeNaNNormalsFromPointCloud(*normals, *normals, *nan_indices);
    Filter::extractIndices(cloud, cloud, nan_indices); 

    // get covariances
    pcl::features::computeApproximateCovariances(*cloud, *normals, *covs);
}

void GICPAlignment::applyCovariances()
{
    boost::shared_ptr<CovariancesVector> source_covariances(new CovariancesVector);
    boost::shared_ptr<CovariancesVector> target_covariances(new CovariancesVector);

    ROS_INFO("Extract covariances from clouds");
    getCovariances(source_cloud_, source_covariances);
    getCovariances(target_cloud_, target_covariances);

    gicp_.setSourceCovariances(source_covariances);
    gicp_.setTargetCovariances(target_covariances);
}

void GICPAlignment::fineAlignment()
{
    ROS_INFO("Perform GICP with %d iterations", gicp_.getMaximumIterations());
    gicp_.setInputSource(source_cloud_);
    gicp_.setInputTarget(target_cloud_);

    ros::Time begin = ros::Time::now();
    
    PointCloudRGB::Ptr aligned_cloud(new PointCloudRGB);
    ROS_INFO("This step may take a while ...");
    gicp_.align(*aligned_cloud);
    
    ros::Duration exec_time = ros::Time::now() - begin;
    ROS_INFO("GICP time: %lf s", exec_time.toSec());

    if (gicp_.hasConverged())
    {
        ROS_INFO("Converged in %f FitnessScore", gicp_.getFitnessScore());
        fine_tf_ = gicp_.getFinalTransformation();
        transform_exists_ = Utils::isValidTransform(fine_tf_);
    }
    else
        ROS_ERROR("GICP no converge");
}

void GICPAlignment::iterateFineAlignment(PointCloudRGB::Ptr cloud)
{
    backUp(cloud);  // save a copy to restore in case iteration give wrong results

    ROS_INFO("Computing iteration...");
    gicp_.align(*cloud);

    if (gicp_.hasConverged())
    {
        Eigen::Matrix4f temp_tf = gicp_.getFinalTransformation();
        fine_tf_ = temp_tf * fine_tf_;
        Utils::printTransform(fine_tf_);
        ROS_INFO("Converged in %f FitnessScore", gicp_.getFitnessScore());
    }
    else
        ROS_ERROR("GICP no converge");
}

void GICPAlignment::iterate()
{
    iterateFineAlignment(aligned_cloud_);
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
    Utils::cloudToROSMsg(aligned_cloud_, aligned_cloud_msg);
}

void GICPAlignment::setSourceCloud(PointCloudRGB::Ptr source_cloud)
{
    source_cloud_ = source_cloud;
}

void GICPAlignment::setTargetCloud(PointCloudRGB::Ptr target_cloud)
{
    target_cloud_ = target_cloud;
}

void GICPAlignment::setMaxIterations(int iterations)
{
    max_iter_ = iterations;
    configParameters();
}

void GICPAlignment::setTfEpsilon(double tf_epsilon)
{
    tf_epsilon_ = tf_epsilon;
    configParameters();
}

void GICPAlignment::setMaxCorrespondenceDistance(int max_corresp_distance)
{
    max_corresp_distance_ = max_corresp_distance;
    configParameters();
}

void GICPAlignment::setRANSACOutlierTh(int ransac_threshold)
{
    ransac_outlier_th_ = ransac_threshold;
    configParameters();
}
