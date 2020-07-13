#include <initial_alignment.h>

InitialAlignment::InitialAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud)
    : _aligned_cloud(new PointCloudRGB)
{
    _target_cloud = target_cloud;
    _source_cloud = source_cloud;
    configParameters();
    transform_exists = false;
    _rigid_tf = Eigen::Matrix4f::Zero();
}

void InitialAlignment::run()
{
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
    getNormals(_source_cloud, _normal_radius, source_normals);
    getNormals(_target_cloud, _normal_radius, target_normals);

    PointCloudRGB::Ptr target_keypoints(new PointCloudRGB);
    PointCloudRGB::Ptr source_keypoints(new PointCloudRGB);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    getKeypointsAndFeatures(_source_cloud, source_normals, source_keypoints, source_features);
    getKeypointsAndFeatures(_target_cloud, target_normals, target_keypoints, target_features);

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    initialAlingment(source_features, target_features, source_keypoints, target_keypoints, correspondences);

    applyTFtoCloud();
}

Eigen::Matrix4f InitialAlignment::getRigidTransform()
{
    if(!transform_exists)
        ROS_ERROR("No transform yet. Please run algorithm");
    return _rigid_tf;
}

void InitialAlignment::configParameters()
{
    // si los parametros son el mismo para ambas tiene que depender de las dos
    double target_res = Utils::computeCloudResolution(_target_cloud);
    double source_res = Utils::computeCloudResolution(_source_cloud);

    _normal_radius = (target_res + source_res) * 2.0; // 2 times higher
    _feature_radius = _normal_radius*1.20; // 20% higher
    _inlier_threshold = 2.5;

    // ROS_INFO("Parameters: \n\tnormal radius: %f, \   n\tfeature radius: %f",_normal_radius, _feature_radius);
}

bool InitialAlignment::getNormals(PointCloudRGB::Ptr &cloud,
                                  double normal_radius,
                                  pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    ROS_INFO("Computing normals with radius: %f",normal_radius);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(normal_radius);
    ne.compute(*normals);

    // As we compute normal for each point in cloud, must have same size
    bool success = normals->points.size()==cloud->points.size() ? true : false;

    return success;
}

std::vector<float> InitialAlignment::getScaleValues(PointCloudRGB::Ptr cloud)
{
    std::vector<float> scale_values;

    double cloud_resolution = Utils::computeCloudResolution(cloud);
    scale_values.push_back((float)cloud_resolution*2);
    scale_values.push_back((float)cloud_resolution*3);
    scale_values.push_back((float)cloud_resolution*4);

    return scale_values;
}

void InitialAlignment::printScaleValues(std::vector<float> scale_values)
{
    ROS_INFO("scale:");
    for (int i=0; i<scale_values.size(); i++)
    {
        ROS_INFO("%f",scale_values[i]);
    }
}

void InitialAlignment::getKeypointsAndFeatures(PointCloudRGB::Ptr cloud, 
                                                pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                PointCloudRGB::Ptr keypoints_cloud,
                                                pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    // SELECTED DESCRIPTOR: FPFH
    ROS_INFO("1. Set descriptor FPFH with radius: %f",_feature_radius);
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr  
        fest(new pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
    fest->setInputCloud(cloud);
    fest->setInputNormals(normals);
    // fest->setRadiusSearch(_feature_radius);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    fest->setSearchMethod(tree);

    ROS_INFO("2. Define feature persistence");
    pcl::MultiscaleFeaturePersistence<pcl::PointXYZRGB, pcl::FPFHSignature33> fper;
    boost::shared_ptr<std::vector<int> > keypoints(new std::vector<int>); // Interest points
    std::vector<float> scale_values = getScaleValues(cloud);
    // printScaleValues(scale_values);
    fper.setScalesVector(scale_values);
    fper.setAlpha(0.6f);
    fper.setFeatureEstimator(fest);
    fper.setDistanceMetric(pcl::CS);

    ROS_INFO("3. Extracting keypoints");
    fper.determinePersistentFeatures(*features, keypoints);

    // ROS_INFO("keypoints: %zu", keypoints->size());   
    bool success = keypoints->size()==features->size() ? true : false;

    Utils::indicesFilter(cloud, keypoints_cloud, keypoints);
    // pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
    // extract_indices_filter.setInputCloud(cloud);
    // extract_indices_filter.setIndices(keypoints);
    // extract_indices_filter.filter(*keypoints_cloud);
}


void InitialAlignment::initialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                                           PointCloudRGB::Ptr source_keypoints,
                                           PointCloudRGB::Ptr target_keypoints,
                                           pcl::CorrespondencesPtr corr_filtered)
{
    ROS_INFO("4. Use descriptor FPFH to compute correspondences");
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> cest;
    cest.setInputSource(source_features);
    cest.setInputTarget(target_features);
    cest.determineCorrespondences(*correspondences);

    ROS_INFO("5. Get correspondences with inlier threshold: %f",_inlier_threshold);
    // pcl::CorrespondencesPtr corr_filtered(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector;
    rejector.setInputSource(source_keypoints);
    rejector.setInputTarget(target_keypoints);
    rejector.setInlierThreshold(_inlier_threshold);
    rejector.setMaximumIterations(100000);
    rejector.setRefineModel(false);
    rejector.setInputCorrespondences(correspondences);
    rejector.getCorrespondences(*corr_filtered);
    
    ROS_INFO("6. Get rigid transformation: ");
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> trans_est;
    trans_est.estimateRigidTransformation(*source_keypoints,*target_keypoints, 
                                          *corr_filtered, _rigid_tf);
    transform_exists = true;
}

void InitialAlignment::applyTFtoCloud()
{
    pcl::transformPointCloud(*_source_cloud,*_aligned_cloud,_rigid_tf);
}

void InitialAlignment::getAlignedCloud(PointCloudRGB::Ptr aligned_cloud)
{
    pcl::copyPointCloud(*_aligned_cloud, *aligned_cloud);
}

void InitialAlignment::getAlignedCloudROSMsg(sensor_msgs::PointCloud2 &aligned_cloud_msg)
{   
    pcl::toROSMsg(*_aligned_cloud,aligned_cloud_msg);
    aligned_cloud_msg.header.frame_id = Utils::_frame_id;
    aligned_cloud_msg.header.stamp = ros::Time::now();
}
