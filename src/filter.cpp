#include <filter.h>

Filter::Filter(double leaf_size)
{
    _leaf_size = leaf_size;
    _noise_filter_threshold = 0;
    _floor_filter_threshold = 0;
}

Filter::Filter(double leaf_size, double noise_threshold)
{
    _leaf_size = leaf_size;
    _noise_filter_threshold = noise_threshold;
    _floor_filter_threshold = 0;
}

Filter::Filter(double leaf_size, double noise_threshold, double floor_threshold)
{
    _leaf_size = leaf_size;
    _noise_filter_threshold = noise_threshold;
    _floor_filter_threshold = floor_threshold;
}

void Filter::downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled)
{
    double res = Utils::computeCloudResolution(cloud);
    // ROS_INFO("Pointcloud size before downsampling: %zu",cloud->points.size());
    // ROS_INFO("Pointcloud resolution before downsampling: %f",res);

    pcl::VoxelGrid<pcl::PointXYZRGB> downsampling_filter;
    downsampling_filter.setInputCloud(cloud);
    const Eigen::Vector4f downsampling_leaf_size(_leaf_size, _leaf_size, _leaf_size, 0.0f);
    downsampling_filter.setLeafSize(downsampling_leaf_size);
    downsampling_filter.filter(*cloud_downsampled);

    res = Utils::computeCloudResolution(cloud_downsampled);
    // ROS_INFO("Pointcloud size after downsampling: %zu",cloud_downsampled->points.size());
    // ROS_INFO("Pointcloud resolution after downsampling: %f",res);
}

void Filter::filter_noise(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
    // ROS_INFO("create box with threshold: %f",threshold);
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    
    //get box center
    Eigen::Vector3f boxTranslatation;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    boxTranslatation[0]=centroid[0];  
    boxTranslatation[1]=centroid[1];  
    boxTranslatation[2]=centroid[2];
    // ROS_INFO("Point cloud center: (%f,%f,%f)",centroid[0],centroid[1],centroid[2]);

    // apply filter
    boxFilter.setTranslation(boxTranslatation);
    boxFilter.setMin(Eigen::Vector4f(-threshold, -threshold, -threshold, 1.0));
    boxFilter.setMax(Eigen::Vector4f(threshold, threshold, threshold, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud_filtered);
}

void Filter::filter_floor(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
    // ROS_INFO("create segmenter with threshold: %f",threshold);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Search for a plane perpendicular to z axis, 1 degree tolerance
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(1.0));seg.setOptimizeCoefficients(true);
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
    _leaf_size = new_leaf_size;
}

void Filter::run(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
    downsampleCloud(cloud, cloud_filtered);
    if (_noise_filter_threshold > 0)
        filter_noise(_noise_filter_threshold,cloud_filtered, cloud_filtered);
    if (_floor_filter_threshold > 0)
        filter_floor(_floor_filter_threshold,cloud_filtered, cloud_filtered);
}