#include <stdlib.h> 
#include "pcl/common/angles.h"
// #include "pcl/sample_consensus/method_types.h"
// #include "pcl/sample_consensus/model_types.h"
#include <pcl/filters/crop_box.h>
#include <cad_to_pointcloud.h>

#define DEFAULT_THRESHOLD 30

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FilterScanNoise");

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_noise_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    CADToPointCloud cad_to_pointcloud;
    std::string f = cad_to_pointcloud._pc_path + file_name + ".pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *cloud);
    cad_to_pointcloud.visualizePointCloud(cloud, cad_to_pointcloud.PINK);

    ROS_INFO("create box with threshold: %f",threshold);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    
    //get box center
    Eigen::Vector3f boxTranslatation;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    boxTranslatation[0]=centroid[0];  
    boxTranslatation[1]=centroid[1];  
    boxTranslatation[2]=centroid[2];
    ROS_INFO("Point cloud center: (%f,%f,%f)",centroid[0],centroid[1],centroid[2]);

    // apply filter
    boxFilter.setTranslation(boxTranslatation);
    boxFilter.setMin(Eigen::Vector4f(-threshold, -threshold, -threshold, 1.0));
    boxFilter.setMax(Eigen::Vector4f(threshold, threshold, threshold, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*no_noise_cloud);
    
    ROS_INFO("save");
    f = cad_to_pointcloud._pc_path + file_name + "_no_noise.pcd";
    pcl::io::savePCDFile<pcl::PointXYZ> (f, *no_noise_cloud);

    cad_to_pointcloud.addPCToVisualizer(no_noise_cloud,cad_to_pointcloud.PINK,"cloud");

    return 0;
}