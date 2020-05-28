#include <stdlib.h> 
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/filters/extract_indices.h>
#include <cad_to_pointcloud.h>

#define DEFAULT_THRESHOLD 0.1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FilterScanFloor");

    std::string file_name = "";
    double threshold;
    if (argc<2)
    {
        ROS_ERROR("Please specify file_name");
        return 0;
    }
    else
    {
        file_name = argv[1];
        if (file_name.find(".")!=std::string::npos)
        {
            ROS_ERROR("Please specify file_name without extension");
            return 0;
        }
        if (argc<3)
        {
            threshold = DEFAULT_THRESHOLD;
            ROS_WARN("Threshold not specify. Set to default: %f",threshold);
        }
        else 
        {
            threshold = atof(argv[2]);
            if (threshold < 0)
            {
                ROS_ERROR("Please specify non-negative threshold");
                return 0;
            }
        }
    }

    ROS_INFO("open file");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    CADToPointCloud cad_to_pointcloud;
    std::string f = cad_to_pointcloud._pc_path + file_name + ".pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *cloud);
    ROS_INFO("create segmenter with threshold: %f",threshold);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    // Search for a plane perpendicular to some axis (specified below).
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // Set the distance to the plane for a point to be an inlier.
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(cloud);
    ROS_INFO("axis");
    // Make sure that the plane is perpendicular to Z-axis, 1 degree tolerance.
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(1.0));
    
    ROS_INFO("apply");
    pcl::ModelCoefficients coeff;
    pcl::PointIndices indices_internal;
    seg.segment(indices_internal, coeff);

    if (indices_internal.indices.size() == 0) 
    {
        ROS_ERROR("Unable to find surface.");
        return 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr no_floor_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(boost::make_shared<std::vector<int>>(indices_internal.indices));
    extract_indices_filter.setNegative(true);
    extract_indices_filter.filter(*no_floor_cloud);
    
    ROS_INFO("save");
    f = cad_to_pointcloud._pc_path + file_name + "_no_floor.pcd";
    pcl::io::savePCDFile<pcl::PointXYZ> (f, *no_floor_cloud);

    ROS_INFO("view");
    pcl::visualization::PCLVisualizer viewer("No floor pointcloud");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addPointCloud(no_floor_cloud);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }


    return 0;
}