#include <stdlib.h> 
// #include "pcl/common/angles.h"
// #include "pcl/sample_consensus/method_types.h"
// #include "pcl/sample_consensus/model_types.h"
// #include <pcl/filters/crop_box.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/point_cloud.h> 
#include <cad_to_pointcloud.h>

#define NORMAL_RADIUS 0.1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CloudToMesh");

    std::string file_name = "";
    double radius_search;
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
            radius_search = NORMAL_RADIUS;
            ROS_WARN("Threshold not specified. Set to default: %f",radius_search);
        }
        else 
        {
            radius_search = atof(argv[2]);
            if (radius_search < 0)
            {
                ROS_ERROR("Please specify non-negative threshold");
                return 0;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("open file");
    CADToPointCloud cad_to_pointcloud;
    std::string f = cad_to_pointcloud._pc_path + "conjunto_estranio_scan_transformed_reconstructed.stl";
    std::string f2 = cad_to_pointcloud._pc_path + "conjunto_estranio_cad.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f2, *cad_pc);
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_pc);    

    cad_to_pointcloud.visualizePointCloud(cad_pc,cad_to_pointcloud.WHITE);
    cad_to_pointcloud.addPCToVisualizer(scan_pc,cad_to_pointcloud.PINK,"scan");

    

    return 0;
}